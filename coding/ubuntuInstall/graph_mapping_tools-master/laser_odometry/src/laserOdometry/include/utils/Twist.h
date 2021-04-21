

#ifndef LIO_TWIST_H_
#define LIO_TWIST_H_

#include <iostream>
#include <Eigen/Eigen>

namespace lio {

template<typename T>
struct Twist {
  Eigen::Quaternion<T> rot;
  Eigen::Matrix<T, 3, 1> pos;

  static Twist Identity() {
    return Twist();
  }

  Twist() {
    rot.setIdentity();
    pos.setZero();
  }

  Twist(Eigen::Quaternion<T> rot_in, Eigen::Matrix<T, 3, 1> pos_in) {
    this->rot = rot_in;
    this->pos = pos_in;
  }

  Twist(Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform) {
    this->rot = Eigen::Quaternion<T>{transform.linear()}.normalized();
    this->pos = transform.translation();
  }

  Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform() const {
    Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform;
    transform.linear() = rot.normalized().toRotationMatrix();
    transform.translation() = pos;
    return transform;
  }

  Twist inverse() const {
    Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform_inv = this->transform().inverse();
    Twist twist_inv;
    twist_inv.rot = transform_inv.linear();
    twist_inv.pos = transform_inv.translation();
    return twist_inv;
  }

  Twist operator*(const Twist &other) const {
    Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform_out = this->transform() * other.transform();
    return Twist(transform_out);
  }

  template<typename NewType>
  Twist<NewType> cast() const {
    Twist<NewType> twist_new{this->rot.template cast<NewType>(), this->pos.template cast<NewType>()};
    return twist_new;
  }

  friend std::ostream &operator<<(std::ostream &os, const Twist &twist) {
    os << twist.pos.x() << " " << twist.pos.y() << " " << twist.pos.z() << " " << twist.rot.w() << " " << twist.rot.x()
       << " " << twist.rot.y() << " " << twist.rot.z();
    return os;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; // class Twist

} // namespace lio

#endif //LIO_TWIST_H_
