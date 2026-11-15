// The DatasetReader concept (docs/design.md §6). EurocReader is the only
// implementation in version 1; the main program takes the reader as a template
// parameter, so there is no virtual dispatch.
#ifndef MINI_VIO_DATASET_READER_H_
#define MINI_VIO_DATASET_READER_H_

#include <concepts>
#include <optional>

#include "mini_vio/types.h"

namespace mini_vio {

template <typename R>
concept DatasetReader = requires(R r) {
    // Next IMU sample or camera image in time order; std::nullopt at the end.
    { r.Next() } -> std::same_as<std::optional<SensorEvent>>;
    { r.CameraCalibration() } -> std::same_as<RawStereoCalibration>;
    { r.ImuNoise() } -> std::same_as<ImuNoiseParams>;
    // First timestamp of the whole sequence, the origin for "seconds since start".
    { r.SequenceStart() } -> std::same_as<Timestamp>;
};

}  // namespace mini_vio

#endif  // MINI_VIO_DATASET_READER_H_
