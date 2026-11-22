/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <algorithm>

namespace infrabot_ik {
struct Vec3 {
  double x{0.0}, y{0.0}, z{0.0};
  constexpr Vec3() = default;
  constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  constexpr Vec3 operator+(const Vec3& b) const { return {x+b.x,y+b.y,z+b.z}; }
  constexpr Vec3 operator-(const Vec3& b) const { return {x-b.x,y-b.y,z-b.z}; }
  constexpr Vec3 operator-() const { return {-x,-y,-z}; }
  constexpr Vec3 operator*(double s) const { return {x*s,y*s,z*s}; }
  constexpr Vec3 operator/(double s) const { return {x/s,y/s,z/s}; }
  Vec3& operator+=(const Vec3& b) { x+=b.x; y+=b.y; z+=b.z; return *this; }
};
inline constexpr Vec3 operator*(double s, const Vec3& v) { return v*s; }
inline constexpr double dot(const Vec3& a, const Vec3& b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline constexpr Vec3 cross(const Vec3& a, const Vec3& b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
inline double norm(const Vec3& v) { return std::sqrt(dot(v,v)); }
inline Vec3 normalized(const Vec3& v) { const double n=norm(v); return n>1e-15?v/n:Vec3{}; }

struct Mat3 {
  double m[3][3]{};
  static constexpr Mat3 identity() { Mat3 r{}; r.m[0][0]=r.m[1][1]=r.m[2][2]=1.0; return r; }
  constexpr Vec3 operator*(const Vec3& v) const { return {m[0][0]*v.x+m[0][1]*v.y+m[0][2]*v.z,m[1][0]*v.x+m[1][1]*v.y+m[1][2]*v.z,m[2][0]*v.x+m[2][1]*v.y+m[2][2]*v.z}; }
  constexpr Mat3 operator*(const Mat3& b) const { Mat3 r{}; for(std::size_t i=0;i<3;++i) for(std::size_t j=0;j<3;++j) for(std::size_t k=0;k<3;++k) r.m[i][j]+=m[i][k]*b.m[k][j]; return r; }
  constexpr Mat3 transpose() const { Mat3 r{}; for(std::size_t i=0;i<3;++i) for(std::size_t j=0;j<3;++j) r.m[i][j]=m[j][i]; return r; }
};
inline Mat3 rot_x(double a) { const double c=std::cos(a),s=std::sin(a); Mat3 r=Mat3::identity(); r.m[1][1]=c;r.m[1][2]=-s;r.m[2][1]=s;r.m[2][2]=c;return r; }
inline Mat3 rot_y(double a) { const double c=std::cos(a),s=std::sin(a); Mat3 r=Mat3::identity(); r.m[0][0]=c;r.m[0][2]=s;r.m[2][0]=-s;r.m[2][2]=c;return r; }
inline Mat3 rot_z(double a) { const double c=std::cos(a),s=std::sin(a); Mat3 r=Mat3::identity(); r.m[0][0]=c;r.m[0][1]=-s;r.m[1][0]=s;r.m[1][1]=c;return r; }
inline Mat3 rpy(double roll,double pitch,double yaw) { return rot_z(yaw)*rot_y(pitch)*rot_x(roll); }
inline Mat3 axis_angle(const Vec3& axis_in,double angle) { const Vec3 a=normalized(axis_in);const double c=std::cos(angle),s=std::sin(angle),t=1.0-c;Mat3 r{};r.m[0][0]=t*a.x*a.x+c;r.m[0][1]=t*a.x*a.y-s*a.z;r.m[0][2]=t*a.x*a.z+s*a.y;r.m[1][0]=t*a.x*a.y+s*a.z;r.m[1][1]=t*a.y*a.y+c;r.m[1][2]=t*a.y*a.z-s*a.x;r.m[2][0]=t*a.x*a.z-s*a.y;r.m[2][1]=t*a.y*a.z+s*a.x;r.m[2][2]=t*a.z*a.z+c;return r; }
struct Pose { Vec3 position{}; Mat3 rotation{Mat3::identity()}; };
inline Pose compose(const Pose& a,const Pose& b) { return {a.position+a.rotation*b.position,a.rotation*b.rotation}; }
inline Pose origin_pose(const Vec3& p,const Vec3& r) { return {p,rpy(r.x,r.y,r.z)}; }
inline Vec3 rotation_log(const Mat3& r) {
  const double c=std::max(-1.0,std::min(1.0,(r.m[0][0]+r.m[1][1]+r.m[2][2]-1.0)*0.5));
  const double theta=std::acos(c);
  const Vec3 v{r.m[2][1]-r.m[1][2],r.m[0][2]-r.m[2][0],r.m[1][0]-r.m[0][1]};
  if(theta<1e-8)return v*0.5;
  const double s=std::sin(theta);
  if(std::abs(s)>1e-6)return v*(theta/(2.0*s));
  // At pi the skew part is zero. Recover the axis from the largest diagonal
  // term instead of normalizing an almost-zero skew vector.
  Vec3 axis{};
  const double xx=std::max(0.0,(r.m[0][0]+1.0)*0.5);
  const double yy=std::max(0.0,(r.m[1][1]+1.0)*0.5);
  const double zz=std::max(0.0,(r.m[2][2]+1.0)*0.5);
  if(xx>=yy&&xx>=zz&&xx>1e-10){axis.x=std::sqrt(xx);axis.y=(r.m[0][1]+r.m[1][0])/(4.0*axis.x);axis.z=(r.m[0][2]+r.m[2][0])/(4.0*axis.x);}
  else if(yy>=zz&&yy>1e-10){axis.y=std::sqrt(yy);axis.x=(r.m[0][1]+r.m[1][0])/(4.0*axis.y);axis.z=(r.m[1][2]+r.m[2][1])/(4.0*axis.y);}
  else if(zz>1e-10){axis.z=std::sqrt(zz);axis.x=(r.m[0][2]+r.m[2][0])/(4.0*axis.z);axis.y=(r.m[1][2]+r.m[2][1])/(4.0*axis.z);}
  else axis={1.0,0.0,0.0};
  return normalized(axis)*theta;
}
using JointVector=std::array<double,7>;
struct Jacobian { double linear[3][7]{}; double angular[3][7]{}; };
}  // namespace infrabot_ik
