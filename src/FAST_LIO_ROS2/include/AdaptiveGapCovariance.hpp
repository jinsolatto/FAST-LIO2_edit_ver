#pragma once
#include "GapProcessCovariance.hpp"
#include <Eigen/QR>
#include <array>
#include <fstream>
#include <iomanip>
#include <string>

// Gap-event adaptation in PHYSICAL DENSITY space, not the paper's native IMU Q.
// Unit-density covariance responses are transported through native F. The
// native dt*G is accumulated for auditing, never inverted as a square matrix.
class AdaptiveGapCovariance {
public:
 using P=gap_covariance::Mat23;
 using V=Eigen::Matrix<double,6,1>;
 using Responses=std::array<P,6>;
 bool enabled=false;
 double forgetting=.95;
 Responses response;
 Eigen::Matrix<double,23,12> native_fw_sum=Eigen::Matrix<double,23,12>::Zero();
 int gaps=0;double gap_duration=0.;
 std::ofstream log;
 AdaptiveGapCovariance(){beginEvent();}
 void configure(bool on,double a,const std::string& path){
  if(!std::isfinite(a)||a<0.||a>=1.)throw std::invalid_argument("gap forgetting must be in [0,1)");
  enabled=on;forgetting=a;
  if(!on)return;
  log.open(path,std::ios::out|std::ios::trunc);
  if(!log)throw std::runtime_error("Cannot open adaptive gap CSV: "+path);
  log<<"timestamp,gaps,integrated_gap_seconds,accepted,reason,relative_fit_error";
  for(auto prefix:{"old","target","new"})for(int j=0;j<6;j++)log<<','<<prefix<<'_'<<j;
  for(int j=0;j<9;j++)log<<",delta_"<<j;
  log<<",native_fw_gyro_norm,native_fw_accel_norm";
  for(int k=0;k<6;k++)for(int i=0;i<9;i++)for(int j=0;j<9;j++)log<<",U"<<k<<'_'<<i<<'_'<<j;
  log<<'\n'<<std::setprecision(17);
 }
 void beginEvent(){for(auto &u:response)u.setZero();native_fw_sum.setZero();gaps=0;gap_duration=0.;}
 void predict(const P& F,const Eigen::Matrix<double,23,12>& dtG){
  if(!enabled)return;
  for(auto &u:response)u=(F*u*F.transpose()).eval();
  native_fw_sum+=dtG; // Eq.(6) sum for audit only; physical responses use F transport.
 }
 void addGap(double dt){
  if(!enabled)return;
  for(int k=0;k<6;k++){
   Eigen::Matrix3d g=Eigen::Matrix3d::Zero(),a=g;
   if(k<3)g(k,k)=1.;else a(k-3,k-3)=1.;
   response[k]+=gap_covariance::embedFastLio(gap_covariance::discrete(dt,g,a));
  }
  gaps++;gap_duration+=dt;
 }
 static bool fit(const Responses& u,const Eigen::Matrix<double,23,1>& correction,V& target,double& relative_error){
  if(!correction.allFinite())return false;
  // Block-balanced least squares; normalizers depend only on unit responses,
  // never on current Q or initial Q. The active-set enumeration enforces PSD.
  Eigen::Matrix<double,27,6> A;Eigen::Matrix<double,27,1>b;
  const int start[3]={3,12,0};
  for(int block=0;block<3;block++){
   double scale2=0.;for(auto &v:u)scale2+=v.block<3,3>(start[block],start[block]).squaredNorm();
   if(!std::isfinite(scale2)||scale2<=1e-30)return false;
   double scale=std::sqrt(scale2);
   Eigen::Vector3d d=correction.segment<3>(start[block]);Eigen::Matrix3d C=d*d.transpose();
   for(int i=0;i<3;i++)for(int j=0;j<3;j++){
    int row=block*9+3*i+j;b[row]=C(i,j)/scale;
    for(int k=0;k<6;k++)A(row,k)=u[k](start[block]+i,start[block]+j)/scale;
   }
  }
  if(!A.allFinite()||!b.allFinite())return false;
  Eigen::ColPivHouseholderQR<Eigen::Matrix<double,27,6>> rank(A);
  rank.setThreshold(1e-10);if(rank.rank()<6)return false;
  double best=b.squaredNorm();target.setZero();
  for(int mask=1;mask<64;mask++){
   int n=0;for(int k=0;k<6;k++)if(mask&(1<<k))n++;
   Eigen::MatrixXd sub(27,n);int j=0;
   for(int k=0;k<6;k++)if(mask&(1<<k))sub.col(j++)=A.col(k);
   Eigen::VectorXd v=sub.colPivHouseholderQr().solve(b);
   if(!v.allFinite()||(v.array()<0.).any())continue;
   double loss=(sub*v-b).squaredNorm();
   if(loss<best){best=loss;target.setZero();j=0;for(int k=0;k<6;k++)if(mask&(1<<k))target[k]=v[j++];}
  }
  relative_error=std::sqrt(best)/std::max(b.norm(),1e-15);return target.allFinite();
 }
 bool finish(double timestamp,const Eigen::Matrix<double,23,1>& correction,bool lidar_valid,
             Eigen::Matrix3d& g,Eigen::Matrix3d& a){
  if(!enabled||gaps==0)return false;
  V old,target;old<<g.diagonal(),a.diagonal();target=old;double loss=std::numeric_limits<double>::quiet_NaN();
  bool accepted=lidar_valid&&fit(response,correction,target,loss);
  const char* reason=!lidar_valid?"invalid_lidar_update":(!accepted?"invalid_or_unidentifiable_response":"accepted");
  V next=accepted?(forgetting*old+(1.-forgetting)*target).eval():old;
  if(!next.allFinite()||(next.array()<0).any()){accepted=false;next=old;reason="nonfinite_density";}
  g=next.head<3>().asDiagonal();a=next.tail<3>().asDiagonal();
  log<<timestamp<<','<<gaps<<','<<gap_duration<<','<<accepted<<','<<reason<<','<<loss;
  for(const auto &v:{old,target,next})for(int k=0;k<6;k++)log<<','<<v[k];
  const int ids[9]={3,4,5,12,13,14,0,1,2};for(int i:ids)log<<','<<correction[i];
  log<<','<<native_fw_sum.block<23,3>(0,0).norm()<<','<<native_fw_sum.block<23,3>(0,3).norm();
  for(const auto &u:response)for(int i:ids)for(int j:ids)log<<','<<u(i,j);
  log<<'\n';log.flush();beginEvent();return accepted;
 }
};
