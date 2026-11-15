// Reader for EuRoC MAV sequences in ASL format (FR-1).
#ifndef MINI_VIO_EUROC_READER_H_
#define MINI_VIO_EUROC_READER_H_

#include <cstddef>
#include <filesystem>
#include <optional>
#include <vector>

#include "mini_vio/types.h"

namespace mini_vio {

struct EurocReaderOptions {
    // Sequence folder, the one that contains mav0/ (e.g. .../MH_01_easy).
    std::filesystem::path sequence_dir;
    // Debug only (FR-1): skip this many seconds from the sequence start. Acceptance runs use 0.
    double start_offset_s = 0.0;
    // Debug only: play this many seconds after the offset. Negative means until the end.
    double duration_s = -1.0;
};

// Reads mav0/cam0, mav0/cam1 and mav0/imu0 and emits their samples as one
// stream ordered by timestamp. At equal timestamps the order is IMU, left,
// right, so SensorSync already holds the IMU up to time t when the image at t
// arrives. Images are loaded from disk only when Next() reaches them.
//
// Left and right images are emitted separately; pairing them is SensorSync's
// job (FR-2). Ground truth is not read here; evo reads it directly.
//
// The constructor throws std::runtime_error if the sequence is missing or
// malformed. These are startup errors that end the program, so they are
// reported with the file and line instead of being returned.
class EurocReader {
public:
    explicit EurocReader(const EurocReaderOptions& options);

    std::optional<SensorEvent> Next();

    RawStereoCalibration CameraCalibration() const { return calibration_; }
    ImuNoiseParams ImuNoise() const { return imu_noise_; }
    Timestamp SequenceStart() const { return sequence_start_; }

private:
    struct ImageEntry {
        Timestamp t;
        std::filesystem::path path;
    };

    CameraImage LoadImage(const ImageEntry& entry, CameraId camera) const;

    RawStereoCalibration calibration_;
    ImuNoiseParams imu_noise_;
    Timestamp sequence_start_ = 0;

    std::vector<ImuData> imu_;
    std::vector<ImageEntry> left_;
    std::vector<ImageEntry> right_;
    // Next unread entry of each stream; all three already skip the start offset.
    std::size_t next_imu_ = 0;
    std::size_t next_left_ = 0;
    std::size_t next_right_ = 0;
    Timestamp end_time_ = 0;  // inclusive
};

}  // namespace mini_vio

#endif  // MINI_VIO_EUROC_READER_H_
