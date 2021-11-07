#ifndef __CAD2CAV_TYPES_CAMERA_HPP__
#define __CAD2CAV_TYPES_CAMERA_HPP__

#include <cad2cav_msgs/CameraParam.h>

#include <Eigen/Dense>
#include <array>
#include <string>

namespace cad2cav {

struct CameraInfo {
  Eigen::Matrix3d intrinsic_;
  Eigen::Matrix4d extrinsic_;
  std::array<double, 5> distortion_coeff_;

  CameraInfo() {
    intrinsic_        = Eigen::Matrix3d::Zero();
    extrinsic_        = Eigen::Matrix4d::Zero();
    distortion_coeff_ = {0.0, 0.0, 0.0, 0.0, 0.0};
  }
};

class Camera {
public:
  Camera() = default;
  Camera(const CameraInfo& camera_info) : info_(camera_info) {}

  const CameraInfo& getInfo() const { return info_; }
  void setInfo(const CameraInfo& new_info) { info_ = new_info; }

  cad2cav_msgs::CameraParam toMsg() const {
    cad2cav_msgs::CameraParam param_msg;
    param_msg.id = id_;
    // store intrinsic matrix
    for (int row = 0; row < info_.intrinsic_.rows(); ++row) {
      for (int col = 0; col < info_.intrinsic_.cols(); ++col) {
        param_msg.intrinsic.at(info_.intrinsic_.cols() * row + col) =
            info_.intrinsic_(row, col);
      }
    }
    // store extrinsic rotation matrix
    for (int row = 0; row < info_.extrinsic_.rows() - 1; ++row) {
      for (int col = 0; col < info_.extrinsic_.cols() - 1; ++col) {
        param_msg.intrinsic.at((info_.intrinsic_.cols() - 1) * row + col) =
            info_.intrinsic_(row, col);
      }
    }
    // store extrinsic translation matrix
    for (int row = 0; row < info_.extrinsic_.rows() - 1; ++row) {
      param_msg.intrinsic.at(row) = info_.intrinsic_.rightCols(1)(row);
    }
    // store distortion coefficients
    for (size_t i = 0; i < info_.distortion_coeff_.size(); ++i) {
      param_msg.distortion_coeff.at(i) = info_.distortion_coeff_.at(i);
    }

    return param_msg;
  }

private:
  std::string id_;
  CameraInfo info_;
};

}  // namespace cad2cav

#endif /* __CAD2CAV_TYPES_CAMERA_HPP__ */
