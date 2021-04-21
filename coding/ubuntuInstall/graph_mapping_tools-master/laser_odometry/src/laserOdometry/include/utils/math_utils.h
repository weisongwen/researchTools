

#ifndef LIO_MATH_UTILS_H_
#define LIO_MATH_UTILS_H_

#include <cmath>
#include <limits>

namespace mathutils {

template<typename T>
inline T RadToDeg(T rad) {
  return rad * 180.0 / M_PI;
}

template<typename T>
inline T NormalizeRad(T rad) {
  rad = fmod(rad + M_PI, 2 * M_PI);
  if (rad < 0) {
    rad += 2 * M_PI;
  }
  return rad - M_PI;
}

template<typename T>
inline T DegToRad(T deg) {
  return deg / 180.0 * M_PI;
}

template<typename T>
inline T NormalizeDeg(T deg) {
  deg = fmod(deg + 180.0, 360.0);
  if (deg < 0) {
    deg += 360.0;
  }
  return deg - 180.0;
}

inline bool RadLt(double a, double b) {
  return NormalizeRad(a - b) < 0;
}

inline bool RadGt(double a, double b) {
  return NormalizeRad(a - b) > 0;
}

template<typename PointT>
inline PointT ScalePoint(const PointT &p, float scale) {
  PointT p_o = p;
  p_o.x *= scale;
  p_o.y *= scale;
  p_o.z *= scale;
  return p_o;
}

template<typename PointT>
inline float CalcSquaredDiff(const PointT &a, const PointT &b) {
  float diff_x = a.x - b.x;
  float diff_y = a.y - b.y;
  float diff_z = a.z - b.z;

  return diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
}

template<typename PointT>
inline float CalcSquaredDiff(const PointT &a, const PointT &b, const float &wb) {
  float diff_x = a.x - b.x * wb;
  float diff_y = a.y - b.y * wb;
  float diff_z = a.z - b.z * wb;

  return diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
}

template<typename PointT>
inline float CalcPointDistance(const PointT &p) {
  return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

template<typename PointT>
inline float CalcSquaredPointDistance(const PointT &p) {
  return p.x * p.x + p.y * p.y + p.z * p.z;
}

///< Matrix calculations

static const Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();

template<typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> DeltaQ(const Eigen::MatrixBase<Derived> &theta) {
  typedef typename Derived::Scalar Scalar_t;

  Eigen::Quaternion<Scalar_t> dq;
  Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
  half_theta /= static_cast<Scalar_t>(2.0);
  dq.w() = static_cast<Scalar_t>(1.0);
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();
  return dq;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(const Eigen::MatrixBase<Derived> &v3d) {
  Eigen::Matrix<typename Derived::Scalar, 3, 3> m;
  m << typename Derived::Scalar(0), -v3d.z(), v3d.y(),
      v3d.z(), typename Derived::Scalar(0), -v3d.x(),
      -v3d.y(), v3d.x(), typename Derived::Scalar(0);
  return m;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> LeftQuatMatrix(const Eigen::QuaternionBase<Derived> &q) {
  Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
  Eigen::Matrix<typename Derived::Scalar, 3, 1> vq = q.vec();
  typename Derived::Scalar q4 = q.w();
  m.block(0, 0, 3, 3) << q4 * I3x3 + SkewSymmetric(vq);
  m.block(3, 0, 1, 3) << -vq.transpose();
  m.block(0, 3, 3, 1) << vq;
  m(3, 3) = q4;
  return m;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> RightQuatMatrix(const Eigen::QuaternionBase<Derived> &p) {
  Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
  Eigen::Matrix<typename Derived::Scalar, 3, 1> vp = p.vec();
  typename Derived::Scalar p4 = p.w();
  m.block(0, 0, 3, 3) << p4 * I3x3 - SkewSymmetric(vp);
  m.block(3, 0, 1, 3) << -vp.transpose();
  m.block(0, 3, 3, 1) << vp;
  m(3, 3) = p4;
  return m;
}

template<typename T>
inline Eigen::Matrix<T, 4, 4> LeftQuatMatrix(const Eigen::Matrix<T, 4, 1> &q) {
  Eigen::Matrix<T, 4, 4> m;
  Eigen::Matrix<T, 3, 1> vq{q.x(), q.y(), q.z()};
  T q4 = q.w();
  m.block(0, 0, 3, 3) << q4 * I3x3 + SkewSymmetric(vq);
  m.block(3, 0, 1, 3) << -vq.transpose();
  m.block(0, 3, 3, 1) << vq;
  m(3, 3) = q4;
  return m;
}

template<typename T>
inline Eigen::Matrix<T, 4, 4> RightQuatMatrix(const Eigen::Matrix<T, 4, 1> &p) {
  Eigen::Matrix<T, 4, 4> m;
  Eigen::Matrix<T, 3, 1> vp{p.x(), p.y(), p.z()};
  T p4 = p.w();
  m.block(0, 0, 3, 3) << p4 * I3x3 - SkewSymmetric(vp);
  m.block(3, 0, 1, 3) << -vp.transpose();
  m.block(0, 3, 3, 1) << vp;
  m(3, 3) = p4;
  return m;
}

// adapted from VINS-mono
inline Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);

  Eigen::Vector3d ypr(3);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr / M_PI * 180.0;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
{
  typedef typename Derived::Scalar Scalar_t;

  Scalar_t y = ypr(0) / 180.0 * M_PI;
  Scalar_t p = ypr(1) / 180.0 * M_PI;
  Scalar_t r = ypr(2) / 180.0 * M_PI;

  Eigen::Matrix<Scalar_t, 3, 3> Rz;
  Rz << cos(y), -sin(y), 0,
      sin(y), cos(y), 0,
      0, 0, 1;

  Eigen::Matrix<Scalar_t, 3, 3> Ry;
  Ry << cos(p), 0., sin(p),
      0., 1., 0.,
      -sin(p), 0., cos(p);

  Eigen::Matrix<Scalar_t, 3, 3> Rx;
  Rx << 1., 0., 0.,
      0., cos(r), -sin(r),
      0., sin(r), cos(r);

  return Rz * Ry * Rx;
}

} // namespance mathutils

#endif //LIO_MATH_UTILS_H_
