
#include "mini_vio/sensor_sync.h"

namespace mini_vio {
std::optional<StereoImages> SensorSync::AddEvent(const SensorEvent& e) {
    const auto* img = std::get_if<CameraImage>(&e);
    if (img == nullptr) {
        return std::nullopt;  // IMU sample, skip
    }

    if (last_cam_img_.has_value() && last_cam_img_->camera != img->camera && last_cam_img_->t == img->t) {
        StereoImages si;
        si.t = img->t;
        si.left = img->camera == CameraId::kLeft ? img->image : last_cam_img_->image;
        si.right = img->camera == CameraId::kRight ? img->image : last_cam_img_->image;
        last_cam_img_.reset();
        return si;
    }
    last_cam_img_ = *img;
    return std::nullopt;
};

}  // namespace mini_vio
