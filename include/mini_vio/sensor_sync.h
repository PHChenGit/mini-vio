#ifndef MINI_VIO_SENSOR_SYNC_H_
#define MINI_VIO_SENSOR_SYNC_H_

#include <optional>
#include <span>
#include <variant>

#include "mini_vio/types.h"

namespace mini_vio {
class SensorSync {
public:
    // Feed one event; returns a pair once both images are in AND IMU covers t.
    std::optional<StereoImages> AddEvent(const SensorEvent& e);
    std::span<const ImuData> ImuBetween(Timestamp t0, Timestamp t1);

private:
    Timestamp last_image_t_;
    std::optional<CameraImage> last_cam_img_;
};
}  // namespace mini_vio
#endif  // MINI_VIO_SENSOR_SYNC_H_
