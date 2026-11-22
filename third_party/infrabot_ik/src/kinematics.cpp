/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "infrabot_ik/kinematics.hpp"
#include <algorithm>
namespace infrabot_ik {
Pose Kinematics::forward(const JointVector& q) const { Pose current{}; for(std::size_t i=0;i<7;++i){const auto& s=kJointSpecs[i];const Pose jf=compose(current,origin_pose(s.origin_xyz,s.origin_rpy));current=compose(jf,Pose{{0.0,0.0,0.0},axis_angle(s.axis,q[i])});}return compose(current,origin_pose(kFlangePosition,kFlangeRpy)); }
Jacobian Kinematics::jacobian(const JointVector& q) const { Jacobian out{};Pose current{};Vec3 jp[7]{},wa[7]{};for(std::size_t i=0;i<7;++i){const auto& s=kJointSpecs[i];const Pose jf=compose(current,origin_pose(s.origin_xyz,s.origin_rpy));jp[i]=jf.position;wa[i]=normalized(jf.rotation*s.axis);current=compose(jf,Pose{{0.0,0.0,0.0},axis_angle(s.axis,q[i])});}const Vec3 end=current.position+current.rotation*kFlangePosition;for(std::size_t i=0;i<7;++i){const Vec3 lv=cross(wa[i],end-jp[i]);out.linear[0][i]=lv.x;out.linear[1][i]=lv.y;out.linear[2][i]=lv.z;out.angular[0][i]=wa[i].x;out.angular[1][i]=wa[i].y;out.angular[2][i]=wa[i].z;}return out; }
bool Kinematics::within_limits(const JointVector& q,double margin) const {for(std::size_t i=0;i<7;++i)if(q[i]<kJointSpecs[i].lower+margin||q[i]>kJointSpecs[i].upper-margin)return false;return true;}
JointVector Kinematics::clamp(const JointVector& q,double margin) const {JointVector out=q;for(std::size_t i=0;i<7;++i){const double lo=kJointSpecs[i].lower+margin,hi=kJointSpecs[i].upper-margin;out[i]=std::max(lo,std::min(hi,out[i]));}return out;}
} // namespace infrabot_ik
