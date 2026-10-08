#pragma once
#include "use-ikfom.hpp"
#include <Eigen/Cholesky>
#include <fstream>
#include <iomanip>
#include <limits>

// Read-only diagnostics. No estimator setters, no modified measurements.
class GapConsistencyAudit {
public:
 bool enabled=false; double stamp=0.;
 Eigen::Matrix<double,23,23> prior;
 void configure(bool on,const std::string& prefix){
  enabled=on;if(!on)return;
  nis.open(prefix+"_nis.csv");poses.open(prefix+"_pose_cov.csv");
  if(!nis||!poses)throw std::runtime_error("Cannot open consistency audit logs");
  nis<<"timestamp,valid,rows,nis,nis_per_row,marginal_nis_per_row,marginal_95_fraction,measurement_R,HPH_trace,reason\n";
  poses<<"timestamp,stage,px,py,pz,qx,qy,qz,qw";
  for(int i=0;i<6;++i)for(int j=0;j<6;++j)poses<<",P"<<i<<j;
  for(int i=0;i<23;++i)poses<<",state_Pdiag"<<i;
  poses<<'\n';nis<<std::setprecision(17);poses<<std::setprecision(17);
 }
 struct Innovation {bool valid=false;double joint=NAN,marginal=NAN,coverage=NAN,hph=NAN;};
 static Innovation evaluate(const Eigen::MatrixXd& H,const Eigen::VectorXd& residual,
                            const Eigen::Matrix<double,12,12>& P,double R){
  Innovation out;
  if(H.rows()==0||H.cols()!=12||residual.size()!=H.rows()||!H.allFinite()||!residual.allFinite()||!P.allFinite()||!std::isfinite(R)||R<=0)return out;
  Eigen::Matrix<double,12,12> sym=(P+P.transpose())*.5;
  Eigen::LLT<Eigen::Matrix<double,12,12>> chol(sym);
  if(chol.info()!=Eigen::Success)return out; // No hidden covariance repair or jitter.
  Eigen::MatrixXd B=H*chol.matrixL().toDenseMatrix()/std::sqrt(R);
  Eigen::VectorXd y=residual/std::sqrt(R);
  Eigen::Matrix<double,12,12> K=Eigen::Matrix<double,12,12>::Identity()+B.transpose()*B;
  Eigen::LLT<Eigen::Matrix<double,12,12>> solve(K);
  if(solve.info()!=Eigen::Success)return out;
  Eigen::Matrix<double,12,1> z=solve.solve(B.transpose()*y);
  // Stable Woodbury form, avoiding cancellation in y'y - correction.
  out.joint=(y-B*z).squaredNorm()+z.squaredNorm();
  Eigen::VectorXd variance=(B.rowwise().squaredNorm().array()+1.)*R;
  out.marginal=(residual.array().square()/variance.array()).mean();
  out.coverage=(residual.array().abs()<=1.959963984540054*variance.array().sqrt()).cast<double>().mean();
  out.hph=(variance.array()-R).sum();out.valid=std::isfinite(out.joint+out.marginal+out.hph);
  return out;
 }
 void begin(double t,const state_ikfom& s,const Eigen::Matrix<double,23,23>& P){
  if(!enabled)return;stamp=t;prior=P;writePose("prior",s,P);
 }
 void innovation(const Eigen::MatrixXd& H,const Eigen::VectorXd& residual,double R){
  if(!enabled)return;
  auto x=evaluate(H,residual,prior.topLeftCorner<12,12>(),R);
  nis<<stamp<<','<<x.valid<<','<<H.rows()<<','<<x.joint<<','<<x.joint/std::max<Eigen::Index>(H.rows(),1)<<','<<x.marginal<<','<<x.coverage<<','<<R<<','<<x.hph<<','<<(x.valid?"ok":"invalid_input_or_nonSPD_prior")<<'\n';nis.flush();
 }
 void unavailable(){if(enabled){nis<<stamp<<",0,0,nan,nan,nan,nan,nan,nan,no_first_valid_evaluation\n";nis.flush();}}
 void writePose(const char* stage,const state_ikfom& s,const Eigen::Matrix<double,23,23>& P){
  if(!enabled)return;
  // FAST-LIO state: pos0,rot3,extrRot6,extrTrans9,vel12. Right SO3 errors.
  Eigen::Matrix<double,6,23> J=Eigen::Matrix<double,6,23>::Zero();
  const Eigen::Matrix3d RI=s.rot.toRotationMatrix(),E=s.offset_R_L_I.toRotationMatrix();
  vect3 t=s.offset_T_L_I;
  J.block<3,3>(0,0).setIdentity();J.block<3,3>(0,3)=-RI*MTK::hat(t);J.block<3,3>(0,9)=RI;
  J.block<3,3>(3,3)=E.transpose();J.block<3,3>(3,6).setIdentity();
  Eigen::Matrix<double,6,6>C=J*P*J.transpose();
  Eigen::Vector3d p=s.pos+RI*t;Eigen::Quaterniond q(RI*E);
  poses<<stamp<<','<<stage<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w();
  for(int i=0;i<6;++i)for(int j=0;j<6;++j)poses<<','<<C(i,j);
  for(int i=0;i<23;++i)poses<<','<<P(i,i);
  poses<<'\n';poses.flush();
 }
private:std::ofstream nis,poses;
};
