#pragma once
#include "use-ikfom.hpp"
#include <deque>
#include <vector>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <stdexcept>

// Causal, IMU-only virtual-gap calibration. No estimator state/P or LiDAR input.
// Isotropic densities avoid importing the estimator's world-frame orientation.
class ImuOnlyGapLearning {
public:
  struct Sample { double t; Eigen::Vector3d w,a; };
  bool enabled=false;
  double max_dt=.0125, min_h=.02, max_h=1., margin=.02, cadence=.1, tau=5.;
  std::vector<double> warmup_h{.05,.1,.2,.3};
  size_t accepted=0, breaks=0, short_gaps=0, long_gaps=0;
  double q_alpha=0., q_accel=0., q_accel_velocity=0.;
  double last_h=0., last_reference_end=0.;
  // Adaptive gap input. Candidate 0 = endpoint average of the two bounding samples
  // (the native integrator); candidate k>0 = mean of received samples in
  // [head-input_windows[k-1], head] plus the tail sample. On dense windows the learner
  // tracks, per channel and candidate, the virtual-gap error moments (dynamics part)
  // and the per-sample noise variance. At a real gap the propagator picks the
  // candidate minimizing dynamics + noise/N with the ACTUAL sample count N.
  // OFF => legacy zero-order-hold learner and endpoint input, unchanged.
  bool adaptive_input=false;
  std::vector<double> input_windows{.25,.5,1.,2.,4.,8.};
  std::vector<double> qr_c,qp_c,qv_c; // per-candidate densities (rotation, position, velocity)
  double noise_gyr=0.,noise_acc=0.;   // per-axis per-sample variance
  // true: noise from successive-sample differences (var(x_{k+1}-x_k)/2), which is insensitive
  // to real motion between 5 ms samples; false: spread about the window mean (includes motion).
  bool noise_from_differences=false;
  double maxWindow() const {return input_windows.empty()?0.:*std::max_element(input_windows.begin(),input_windows.end());}
  bool ready() const {return accepted>0;}
  void open(const std::string& path) {
    if (!(max_dt>0 && min_h>max_dt && max_h>=min_h && max_h<=5. && margin>=0 && cadence>0 && tau>0) ||
        !std::isfinite(max_dt+min_h+max_h+margin+cadence+tau) || warmup_h.empty())
      throw std::invalid_argument("Invalid IMU-only gap calibration configuration");
    for(double h:warmup_h) if(!std::isfinite(h)||h<min_h||h>max_h) throw std::invalid_argument("Invalid warmup horizon");
    if(adaptive_input){
      if(input_windows.empty())throw std::invalid_argument("Empty gap input windows");
      for(double w:input_windows)if(!std::isfinite(w)||w<=0||w>30.)throw std::invalid_argument("Invalid gap input window");
      const size_t K=input_windows.size()+1;cr.assign(K,0);cp.assign(K,0);cv.assign(K,0);qr_c.assign(K,0);qp_c.assign(K,0);qv_c.assign(K,0);
    }
    log.open(path);if(!log)throw std::runtime_error("Cannot open IMU-only calibration CSV");
    log<<"timestamp,event,requested_h,actual_h,reference_start,reference_end,samples,max_reference_dt,eR2,ev2,ep2,qalpha_sample,qa_position_sample,qa_velocity_sample,qalpha,qa_position,qa_velocity,accepted,breaks,observed_h_count,noise_gyr,noise_acc,qalpha_candidates,qv_candidates\n";
    log<<std::setprecision(17);log.flush();
  }
  // Called with the actual clipped integration duration, not record/header lag.
  void observeGap(double h,double timestamp) {
    if(!enabled)return;
    if(!std::isfinite(h)||h<min_h){++short_gaps;note(timestamp,"actual_gap_too_short",h);return;}
    if(h>max_h){++long_gaps;note(timestamp,"actual_gap_too_long",h);return;}
    horizons.push_back(h);if(horizons.size()>128)horizons.pop_front();
    note(timestamp,"actual_gap_accepted",h);
  }
  void push(const Sample& s) {
    if(!enabled)return;
    if(!std::isfinite(s.t)||!s.w.allFinite()||!s.a.allFinite()){
      history.clear();++breaks;note(s.t,"nonfinite_input",0);return;
    }
    if(!history.empty()) {
      double dt=s.t-history.back().t;
      if(dt<=0 || dt>max_dt){
        history.clear();++breaks;note(s.t,dt<=0?"nonmonotonic_or_duplicate":"reference_gap",dt);
        if(dt<=0){last_attempt=s.t;last_update=s.t;horizons.clear();}
      }
    }
    history.push_back(s);
    if(adaptive_input){
      // All received samples, including sparse ones: the operating input window.
      if(recent.empty()||s.t>recent.back().t)recent.push_back(s);
      while(!recent.empty() && s.t-recent.front().t>maxWindow()+max_h+margin+2*max_dt)recent.pop_front();
    }
    while(history.size()>4096 || (!history.empty() && s.t-history.front().t>max_h+margin+2*max_dt))history.pop_front();
    if(s.t-last_attempt<cadence || history.size()<4)return;
    last_attempt=s.t;
    double target=horizons.empty()?warmup_h[cursor++%warmup_h.size()]:horizons[cursor++%horizons.size()];
    // End at an observed sample with a real, already observed right margin.
    int end=int(history.size())-1;
    while(end>0 && s.t-history[end].t<margin)--end;
    int start=end;
    while(start>0 && history[end].t-history[start].t<target)--start;
    if(start<1 || end-start<3 || history[start].t-history.front().t<margin){note(s.t,"insufficient_history",target);return;}
    // Nearest ORIGINAL starting sample; no synthetic timestamp/measurement.
    if(start+1<end && std::abs(history[end].t-history[start+1].t-target)<std::abs(history[end].t-history[start].t-target))++start;
    double h=history[end].t-history[start].t;
    if(h<min_h||h>max_h||end-start<3){note(s.t,"unsupported_window",target);return;}
    double h3=h*h*h;
    if(adaptive_input){learnAdaptive(s.t,target,start,end,h,h3);return;}
    state_ikfom ref,hold; // identity local frame, zero biases/velocity, same gravity
    ref.pos.setZero();ref.vel.setZero();ref.bg.setZero();ref.ba.setZero();hold=ref;
    double largest=0;
    for(int k=start;k<end;++k){
      double dt=history[k+1].t-history[k].t;largest=std::max(largest,dt);
      input_ikfom u,z;
      u.gyro=.5*(history[k].w+history[k+1].w);u.acc=.5*(history[k].a+history[k+1].a);
      z.gyro=history[start].w;z.acc=history[start].a;
      auto fr=get_f(ref,u);auto fh=get_f(hold,z);ref.oplus(fr,dt);hold.oplus(fh,dt);
    }
    Eigen::Matrix<double,23,1> error;hold.boxminus(error,ref);
    double r2=error.segment<3>(3).squaredNorm(),v2=error.segment<3>(12).squaredNorm(),p2=error.head<3>().squaredNorm();
    if(!std::isfinite(r2+v2+p2)){note(s.t,"nonfinite_propagation",target);return;}
    // trace/3 isotropic projection of 3 e e^T / h^3.
    // EMA of numerator/denominator => h^3-weighted moment estimator.
    double a=ready()?std::exp(-(s.t-last_update)/tau):0.;
    nr=a*nr+(1-a)*r2;np=a*np+(1-a)*p2;den=a*den+(1-a)*h3;
    // Velocity candidate given the SAME h^3 weighting: h^3*(||ev||²/(3h)).
    nv=a*nv+(1-a)*v2*h*h/3.;
    q_alpha=nr/den;q_accel=np/den;q_accel_velocity=nv/den;
    ++accepted;last_update=s.t;last_h=h;last_reference_end=history[end].t;
    if(log){log<<s.t<<",accepted,"<<target<<','<<h<<','<<history[start].t<<','<<history[end].t<<','<<end-start+1<<','<<largest<<','<<r2<<','<<v2<<','<<p2<<','<<r2/h3<<','<<p2/h3<<','<<v2/(3*h)<<','<<q_alpha<<','<<q_accel<<','<<q_accel_velocity<<','<<accepted<<','<<breaks<<','<<horizons.size()<<",nan,nan,,\n";log.flush();}
  }
private:
  std::deque<Sample> recent;
  std::vector<double> cr,cp,cv;double nsg=0,nsa=0;
  void learnAdaptive(double now,double target,int start,int end,double h,double h3){
    const Sample& s0=history[start];const Sample& s1=history[end];
    const size_t K=input_windows.size()+1;
    std::vector<Eigen::Vector3d> mw(K),ma(K);
    mw[0]=.5*(s0.w+s1.w);ma[0]=.5*(s0.a+s1.a);
    for(size_t k=1;k<K;++k){
      Eigen::Vector3d w=s1.w,a=s1.a;int n=1;
      for(const auto& r:recent) if(r.t>=s0.t-input_windows[k-1] && r.t<=s0.t){w+=r.w;a+=r.a;++n;}
      mw[k]=w/n;ma[k]=a/n;
    }
    // Per-sample noise: per-axis variance about the window's own mean.
    Eigen::Vector3d wbar=Eigen::Vector3d::Zero(),abar=Eigen::Vector3d::Zero();
    for(int k=start;k<=end;++k){wbar+=history[k].w;abar+=history[k].a;}
    const int m=end-start+1;wbar/=m;abar/=m;double vg=0,va=0;
    for(int k=start;k<=end;++k){vg+=(history[k].w-wbar).squaredNorm();va+=(history[k].a-abar).squaredNorm();}
    vg/=3.*(m-1);va/=3.*(m-1);
    if(noise_from_differences){
      vg=0;va=0;
      for(int k=start;k<end;++k){vg+=(history[k+1].w-history[k].w).squaredNorm();va+=(history[k+1].a-history[k].a).squaredNorm();}
      vg/=6.*(m-1);va/=6.*(m-1);   // per axis: E||d||^2 = 3 * 2 sigma^2
    }
    // Rotation error depends only on gyro input; velocity/position errors are
    // evaluated per acc candidate with the reference gyro (separable).
    state_ikfom ref;ref.pos.setZero();ref.vel.setZero();ref.bg.setZero();ref.ba.setZero();
    std::vector<state_ikfom> hg(K,ref),ha(K,ref);double largest=0;
    for(int k=start;k<end;++k){
      double dt=history[k+1].t-history[k].t;largest=std::max(largest,dt);
      input_ikfom u;u.gyro=.5*(history[k].w+history[k+1].w);u.acc=.5*(history[k].a+history[k+1].a);
      auto fr=get_f(ref,u);ref.oplus(fr,dt);
      for(size_t c=0;c<K;++c){
        input_ikfom zg=u;zg.gyro=mw[c];auto fg=get_f(hg[c],zg);hg[c].oplus(fg,dt);
        input_ikfom za=u;za.acc=ma[c];auto fa=get_f(ha[c],za);ha[c].oplus(fa,dt);
      }
    }
    std::vector<double> r2(K),p2(K),v2(K);
    for(size_t c=0;c<K;++c){
      Eigen::Matrix<double,23,1> e;hg[c].boxminus(e,ref);r2[c]=e.segment<3>(3).squaredNorm();
      ha[c].boxminus(e,ref);v2[c]=e.segment<3>(12).squaredNorm();p2[c]=e.head<3>().squaredNorm();
      if(!std::isfinite(r2[c]+v2[c]+p2[c])){note(now,"nonfinite_propagation",target);return;}
    }
    // After a long silence a time-decay EMA would hand one sample the full weight;
    // cap each update's weight at that of one dense-cadence update.
    double a=ready()?std::max(std::exp(-(now-last_update)/tau),1.-cadence/tau):0.;
    den=a*den+(1-a)*h3;nsg=a*nsg+(1-a)*vg;nsa=a*nsa+(1-a)*va;noise_gyr=nsg;noise_acc=nsa;
    for(size_t c=0;c<K;++c){
      cr[c]=a*cr[c]+(1-a)*r2[c];cp[c]=a*cp[c]+(1-a)*p2[c];cv[c]=a*cv[c]+(1-a)*v2[c]*h*h/3.;
      qr_c[c]=cr[c]/den;qp_c[c]=cp[c]/den;qv_c[c]=cv[c]/den;
    }
    // Reported/fallback densities: endpoint (native) candidate.
    q_alpha=qr_c[0];q_accel=qp_c[0];q_accel_velocity=qv_c[0];
    ++accepted;last_update=now;last_h=h;last_reference_end=s1.t;
    if(log){log<<now<<",accepted,"<<target<<','<<h<<','<<s0.t<<','<<s1.t<<','<<m<<','<<largest<<','<<r2[0]<<','<<v2[0]<<','<<p2[0]<<','<<r2[0]/h3<<','<<p2[0]/h3<<','<<v2[0]/(3*h)<<','<<q_alpha<<','<<q_accel<<','<<q_accel_velocity<<','<<accepted<<','<<breaks<<','<<horizons.size()<<','<<noise_gyr<<','<<noise_acc<<',';
      for(size_t c=0;c<K;++c)log<<(c?";":"")<<qr_c[c];log<<',';
      for(size_t c=0;c<K;++c)log<<(c?";":"")<<qv_c[c];log<<'\n';log.flush();}
  }
  std::deque<Sample> history;
  std::deque<double> horizons;
  size_t cursor=0;
  double last_attempt=-1e100,last_update=0,nr=0,np=0,nv=0,den=0;
  std::ofstream log;
  void note(double t,const char* event,double h){if(log){log<<t<<','<<event<<','<<h<<",nan,nan,nan,0,nan,nan,nan,nan,nan,nan,nan,"<<q_alpha<<','<<q_accel<<','<<q_accel_velocity<<','<<accepted<<','<<breaks<<','<<horizons.size()<<",nan,nan,,\n";log.flush();}}
};
