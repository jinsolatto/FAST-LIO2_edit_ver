#pragma once
// Minimal, self-contained adaptive voxel-plane map, ported in spirit from
// VoxelSLAM's OctoTree (voxel_map.hpp) for a "Stage 0" shadow comparison
// inside FAST-LIO2: it is built and queried alongside FAST-LIO2's own
// ikd-tree pipeline but never feeds into the EKF. Purpose: log, on the exact
// same points/timestamps FAST-LIO2 already processes, how many planes an
// adaptive-voxel map would have matched and how degenerate the matched-plane
// geometry would have been, for direct comparison against FAST-LIO2's own
// kNN-based degeneracy signal.
//
// This is intentionally simplified relative to VoxelSLAM's production
// OctoTree (no sliding windows, no multi-threading, no marginalization
// bookkeeping) since none of that was needed to reproduce the effect in the
// VoxelSLAM-side ablation (max_layer=0 vs max_points=5 experiments).

#include <Eigen/Dense>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <cstdint>

namespace voxel_shadow {

struct VoxelKey {
  int64_t x, y, z;
  bool operator==(const VoxelKey &o) const { return x == o.x && y == o.y && z == o.z; }
};
struct VoxelKeyHash {
  size_t operator()(const VoxelKey &k) const {
    size_t h = std::hash<int64_t>()(k.x);
    h = h * 131 + std::hash<int64_t>()(k.y);
    h = h * 131 + std::hash<int64_t>()(k.z);
    return h;
  }
};

struct Node {
  int layer;
  double half_len;
  Eigen::Vector3d center;
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Matrix3d sumsq = Eigen::Matrix3d::Zero();
  int n = 0;
  bool has_plane = false;
  bool split_done = false;
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> raw;
  Node *children[8] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

  Node(int l, const Eigen::Vector3d &c, double hl) : layer(l), half_len(hl), center(c) {}
  ~Node() { for (auto *c : children) delete c; }
};

class ShadowVoxelMap {
 public:
  double voxel_size = 1.0;
  int max_layer = 2;
  int min_split_points = 10;
  // Same spirit as VoxelSLAM's Odometry.min_eigen_value / plane_eigen_value_thre:
  // smallest eigenvalue of the point covariance must be tiny (tight planar fit)
  // and much smaller than the largest eigenvalue (flat, not a blob/edge).
  double min_eigen_value = 0.0025;
  double eigen_ratio_thre = 1.0 / 16.0;

  std::unordered_map<VoxelKey, Node *, VoxelKeyHash> map;

  ~ShadowVoxelMap() { for (auto &kv : map) delete kv.second; }

  VoxelKey key_of(const Eigen::Vector3d &p) const {
    VoxelKey k;
    k.x = (int64_t)std::floor(p.x() / voxel_size);
    k.y = (int64_t)std::floor(p.y() / voxel_size);
    k.z = (int64_t)std::floor(p.z() / voxel_size);
    return k;
  }

  static int child_index(Node *node, const Eigen::Vector3d &p) {
    int xi = p.x() > node->center.x() ? 1 : 0;
    int yi = p.y() > node->center.y() ? 1 : 0;
    int zi = p.z() > node->center.z() ? 1 : 0;
    return 4 * xi + 2 * yi + zi;
  }

  // Look up a plane using only what's already in the map (built from past
  // scans) -- mirrors matching a new scan against the existing map before
  // that scan's own points are inserted.
  bool query(const Eigen::Vector3d &p, Eigen::Vector3d &normal_out) const {
    auto it = map.find(key_of(p));
    if (it == map.end()) return false;
    Node *node = it->second;
    while (node) {
      if (!node->split_done) {
        if (node->has_plane) { normal_out = node->normal; return true; }
        return false;
      }
      node = node->children[child_index(node, p)];
    }
    return false;
  }

  void insert(const Eigen::Vector3d &p) {
    VoxelKey k = key_of(p);
    auto it = map.find(k);
    Node *node;
    if (it == map.end()) {
      Eigen::Vector3d center((k.x + 0.5) * voxel_size, (k.y + 0.5) * voxel_size, (k.z + 0.5) * voxel_size);
      node = new Node(0, center, voxel_size * 0.5);
      map[k] = node;
    } else {
      node = it->second;
    }
    insert_recursive(node, p);
  }

 private:
  void insert_recursive(Node *node, const Eigen::Vector3d &p) {
    if (node->split_done) {
      int idx = child_index(node, p);
      if (!node->children[idx]) {
        double hl = node->half_len * 0.5;
        Eigen::Vector3d c = node->center;
        c.x() += (idx & 4) ? hl : -hl;
        c.y() += (idx & 2) ? hl : -hl;
        c.z() += (idx & 1) ? hl : -hl;
        node->children[idx] = new Node(node->layer + 1, c, hl);
      }
      insert_recursive(node->children[idx], p);
      return;
    }

    node->sum += p;
    node->sumsq += p * p.transpose();
    node->n++;
    node->raw.push_back(p);

    if (node->n >= min_split_points) {
      Eigen::Vector3d mean = node->sum / node->n;
      Eigen::Matrix3d cov = node->sumsq / node->n - mean * mean.transpose();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
      Eigen::Vector3d eval = es.eigenvalues();  // ascending
      bool planar = (eval[0] < min_eigen_value) && (eval[2] > 1e-12) && (eval[0] / eval[2] < eigen_ratio_thre);
      if (planar) {
        node->has_plane = true;
        node->normal = es.eigenvectors().col(0);
      } else if (node->layer < max_layer) {
        node->split_done = true;
        node->has_plane = false;
        std::vector<Eigen::Vector3d> pts;
        pts.swap(node->raw);
        for (auto &pt : pts) insert_recursive(node, pt);
      } else {
        node->has_plane = false;
        if (node->raw.size() > 200) { node->raw.clear(); node->raw.shrink_to_fit(); }
      }
    }
  }
};

}  // namespace voxel_shadow
