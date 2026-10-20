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
  // Constant-velocity (CV) fill candidate (default off). Its error over a hidden interval is the true
  // velocity/position change, integrated in the body frame at the interval start with the filter's
  // current gravity-in-body and accel bias (roll/pitch assumed constant over <= max_h).
  bool learn_cv=false; Eigen::Vector3d cv_g_body=Eigen::Vector3d::Zero(), cv_ba=Eigen::Vector3d::Zero();
  double qp_cv=0., qv_cv=0., cp_cv=0., cv_cv=0.; size_t cv_updates=0;
  // Learner fixes (default off = unchanged).
  // skip_static: a virtual gap whose whole window is stationary (|w| < static_w and ||a|-g| < static_a for every
  //   sample) is not learned; a stationary start otherwise seeds near-zero densities that persist under sparse IMU.
  // short_h: when the target horizon does not fit the current dense run, learn on the longest window that does
  //   (>= short_min); densities are h^3-normalized, so shorter windows estimate the same q.
  bool skip_static=false; double static_w=.05, static_a=.15; size_t static_skipped=0;
  bool short_h=false; double short_min=.08; size_t short_used=0;
  // Effective number of learned samples under the same discounting as the moment estimates (0 before the first
  // update); used by the cold-start blend in IMU_Processing.
  double n_eff=0.;
  // v10: two-term fit of the endpoint-average ROTATION error over virtual gaps, |e|^2 ~ rot_a h^2 + rot_b h^3
  // (rot_a: endpoint-sample noise held over the gap, rot_b: unobserved angular-rate change). Same EMA weights.
  double rot_a=0., rot_b=0.; size_t rot_fit_updates=0;
  double s44=0., s45=0., s66=0., sr2=0., sr3=0.;
  void fitRotTwoTerm(){
    const double det=s44*s66-s45*s45;
    double A=0., B=0.;
    if(det>1e-12*std::max(s44*s66,1e-300)){A=(sr2*s66-sr3*s45)/det;B=(s44*sr3-s45*sr2)/det;}
    else{A=0.;B=s66>0?sr3/s66:0.;}
    if(A<0.){A=0.;B=s66>0?sr3/s66:0.;}
    if(B<0.){B=0.;A=s44>0?sr2/s44:0.;}
    rot_a=A;rot_b=B;
  }
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
  // cond_rot (default off): gyro-magnitude-conditioned rotation model |e|^2 ~ (cond_c0 + cond_c1 s^2) h^2,
  //   s = mean of the two endpoint |w|. Learned from a DENSE set of virtual gaps of the received stream: in every
  //   dense run, windows of each length in cond_h starting every cond_stride s (left margin), accepted once a sample
  //   >= margin past the window end has arrived; static windows skipped. EMA time of a window = its end + margin.
  //   Separate accumulators: the existing learner (rot_a/rot_b, dP, inputs) is unchanged.
  bool cond_rot=false; double cond_stride=.05; std::vector<double> cond_h{.08,.12,.16,.20,.25,.30};
  double cond_c0=0., cond_c1=0.; size_t cond_updates=0;
  double condRot(double h,double s) const {return (cond_c0+cond_c1*s*s)*h*h;}
  void push(const Sample& s) {
    if(!enabled)return;
    if(cond_rot)condPush(s);
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
    if(start<1 || end-start<3 || history[start].t-history.front().t<margin){
      if(!short_h){note(s.t,"insufficient_history",target);return;}
      start=1;   // short_h: longest window in the current dense run that keeps the left margin
      while(start<end && history[start].t-history.front().t<margin)++start;
      if(end-start<3 || history[end].t-history[start].t<short_min){note(s.t,"insufficient_history",target);return;}
      ++short_used;
    }
    // Nearest ORIGINAL starting sample; no synthetic timestamp/measurement.
    else if(start+1<end && std::abs(history[end].t-history[start+1].t-target)<std::abs(history[end].t-history[start].t-target))++start;
    double h=history[end].t-history[start].t;
    if(h<min_h||h>max_h||end-start<3){note(s.t,"unsupported_window",target);return;}
    double h3=h*h*h;
    if(skip_static){
      bool moving=false;
      for(int k=start;k<=end && !moving;++k)
        moving=history[k].w.norm()>=static_w || std::abs(history[k].a.norm()-9.81)>=static_a;
      if(!moving){++static_skipped;note(s.t,"static_window",h);return;}
    }
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
  std::deque<Sample> crun; std::vector<double> cstart; std::vector<bool> cdone;
  double cP[5]={0,0,0,0,0}, clast=0.;
  void condPush(const Sample& s){
    if(!std::isfinite(s.t)||!s.w.allFinite()||!s.a.allFinite()){crun.clear();return;}
    if(!crun.empty()){const double dt=s.t-crun.back().t;if(dt<=0||dt>max_dt)crun.clear();}
    if(crun.empty()){cdone.assign(cond_h.size(),false);cstart.assign(cond_h.size(),s.t+margin);} // left margin
    crun.push_back(s);
    const size_t n=crun.size();const double tl=s.t;
    auto firstAt=[&](double t){size_t i=0;while(i<n&&crun[i].t<t)++i;return i;};   // first index with time >= t
    struct W{double t,h,e2,s;};std::vector<W> ready;
    for(size_t c=0;c<cond_h.size();++c){
      while(!cdone[c]){
        const size_t j=firstAt(cstart[c]);if(j>=n)break;                 // start sample not received yet
        const size_t k=firstAt(crun[j].t+cond_h[c]);
        if(k>=n||tl-crun[k].t<margin)break;                              // end (and its right margin) not yet known
        if(k-j<4){cdone[c]=true;break;}
        bool moving=!skip_static;
        for(size_t i=j;i<=k&&!moving;++i)moving=crun[i].w.norm()>=static_w||std::abs(crun[i].a.norm()-9.81)>=static_a;
        if(moving){
          Eigen::Matrix3d R=Eigen::Matrix3d::Identity();
          for(size_t i=j;i<k;++i){const Eigen::Vector3d r=.5*(crun[i].w+crun[i+1].w)*(crun[i+1].t-crun[i].t);
            const double th=r.norm();if(th>1e-12)R=R*Eigen::AngleAxisd(th,r/th).toRotationMatrix();}
          const double h=crun[k].t-crun[j].t;const Eigen::Vector3d rf=.5*(crun[j].w+crun[k].w)*h;
          Eigen::Matrix3d F=Eigen::Matrix3d::Identity();if(rf.norm()>1e-12)F=Eigen::AngleAxisd(rf.norm(),rf/rf.norm()).toRotationMatrix();
          const double e=Eigen::AngleAxisd(F.transpose()*R).angle();
          if(std::isfinite(e))ready.push_back({crun[k].t+margin,h,e*e,.5*(crun[j].w.norm()+crun[k].w.norm())});
        }
        cstart[c]=crun[j].t+cond_stride;
      }
    }
    std::sort(ready.begin(),ready.end(),[](const W&x,const W&y){return x.t<y.t;});
    for(const auto& w:ready){
      const double a=cond_updates?std::max(std::exp(-std::max(0.,w.t-clast)/tau),1.-cadence/tau):0.;clast=std::max(clast,w.t);
      const double h2=w.h*w.h,h4=h2*h2,s2=w.s*w.s,b=1-a;
      cP[0]=a*cP[0]+b*h4;cP[1]=a*cP[1]+b*s2*h4;cP[2]=a*cP[2]+b*s2*s2*h4;cP[3]=a*cP[3]+b*w.e2*h2;cP[4]=a*cP[4]+b*w.e2*s2*h2;
      // same 2x2 normal equations and non-negative fallbacks as fitRotTwoTerm
      const double det=cP[0]*cP[2]-cP[1]*cP[1];double A=0.,B=0.;
      if(det>1e-12*std::max(cP[0]*cP[2],1e-300)){A=(cP[3]*cP[2]-cP[4]*cP[1])/det;B=(cP[0]*cP[4]-cP[1]*cP[3])/det;}
      else{A=0.;B=cP[2]>0?cP[4]/cP[2]:0.;}
      if(A<0.){A=0.;B=cP[2]>0?cP[4]/cP[2]:0.;}
      if(B<0.){B=0.;A=cP[0]>0?cP[3]/cP[0]:0.;}
      cond_c0=A;cond_c1=B;++cond_updates;
    }
    double oldest=tl;bool pending=false;                                 // drop samples no pending window can use
    for(size_t c=0;c<cond_h.size();++c)if(!cdone[c]){oldest=std::min(oldest,cstart[c]);pending=true;}
    if(!pending)oldest=tl;
    while(crun.size()>1&&crun.front().t<oldest-1e-9)crun.pop_front();
  }
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
    n_eff=a*n_eff+1.;
    den=a*den+(1-a)*h3;nsg=a*nsg+(1-a)*vg;nsa=a*nsa+(1-a)*va;noise_gyr=nsg;noise_acc=nsa;
    for(size_t c=0;c<K;++c){
      cr[c]=a*cr[c]+(1-a)*r2[c];cp[c]=a*cp[c]+(1-a)*p2[c];cv[c]=a*cv[c]+(1-a)*v2[c]*h*h/3.;
      qr_c[c]=cr[c]/den;qp_c[c]=cp[c]/den;qv_c[c]=cv[c]/den;
    }
    {const double h2=h*h,h4=h2*h2;
     s44=a*s44+(1-a)*h4;s45=a*s45+(1-a)*h4*h;s66=a*s66+(1-a)*h3*h3;sr2=a*sr2+(1-a)*r2[0]*h2;sr3=a*sr3+(1-a)*r2[0]*h3;
     fitRotTwoTerm();++rot_fit_updates;}
    if(learn_cv){
      Eigen::Matrix3d R=Eigen::Matrix3d::Identity();Eigen::Vector3d v=Eigen::Vector3d::Zero(),p=Eigen::Vector3d::Zero();
      for(int k=start;k<end;++k){
        const double dt=history[k+1].t-history[k].t;
        const Eigen::Vector3d w=.5*(history[k].w+history[k+1].w),acc=.5*(history[k].a+history[k+1].a);
        const Eigen::Vector3d f=R*(acc-cv_ba)+cv_g_body;
        p+=v*dt+.5*f*dt*dt;v+=f*dt;
        const double th=(w*dt).norm();
        if(th>1e-12)R=R*Eigen::AngleAxisd(th,(w*dt)/th).toRotationMatrix();
      }
      const double v2c=v.squaredNorm(),p2c=p.squaredNorm();
      if(std::isfinite(v2c+p2c)){cp_cv=a*cp_cv+(1-a)*p2c;cv_cv=a*cv_cv+(1-a)*v2c*h*h/3.;qp_cv=cp_cv/den;qv_cv=cv_cv/den;++cv_updates;}
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
