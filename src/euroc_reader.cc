#include "mini_vio/euroc_reader.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace mini_vio {
namespace {

namespace fs = std::filesystem;

[[noreturn]] void Fail(const std::string& message) { throw std::runtime_error("EurocReader: " + message); }

std::string_view Trim(std::string_view s) {
    constexpr std::string_view kSpace = " \t\r\n";
    const auto begin = s.find_first_not_of(kSpace);
    if (begin == std::string_view::npos) return {};
    const auto end = s.find_last_not_of(kSpace);
    return s.substr(begin, end - begin + 1);
}

// Where a value came from, for error messages: "file:line".
std::string Where(const fs::path& file, int line) { return file.string() + ":" + std::to_string(line); }

template <typename T>
T ParseNumber(std::string_view text, const std::string& where) {
    T value{};
    const auto [end, ec] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (ec != std::errc() || end != text.data() + text.size()) {
        Fail(where + ": cannot parse number '" + std::string(text) + "'");
    }
    return value;
}

// Calls on_row(fields, line_number) for every data row of a EuRoC CSV file.
// Skips the '#' header and blank lines and accepts both LF and CRLF line ends
// (EuRoC's camera CSVs use CRLF, its IMU CSV uses LF).
void ForEachCsvRow(const fs::path& file, std::size_t expected_fields,
                   const std::function<void(const std::vector<std::string_view>&, int)>& on_row) {
    std::ifstream in(file);
    if (!in) Fail("cannot open " + file.string());

    std::string line;
    std::vector<std::string_view> fields;
    for (int line_number = 1; std::getline(in, line); ++line_number) {
        const std::string_view row = Trim(line);
        if (row.empty() || row.front() == '#') continue;

        fields.clear();
        std::size_t start = 0;
        while (true) {
            const std::size_t comma = row.find(',', start);
            fields.push_back(Trim(row.substr(start, comma - start)));
            if (comma == std::string_view::npos) break;
            start = comma + 1;
        }
        if (fields.size() != expected_fields) {
            Fail(Where(file, line_number) + ": expected " + std::to_string(expected_fields) + " fields, got " +
                 std::to_string(fields.size()));
        }
        on_row(fields, line_number);
    }
}

void CheckIncreasing(Timestamp previous, Timestamp t, const std::string& where) {
    if (t <= previous) Fail(where + ": timestamp " + std::to_string(t) + " is not after " + std::to_string(previous));
}

// mav0/camN/data.csv: "timestamp [ns],filename".
template <typename Entry>
std::vector<Entry> ReadImageList(const fs::path& camera_dir) {
    const fs::path csv = camera_dir / "data.csv";
    std::vector<Entry> entries;
    ForEachCsvRow(csv, 2, [&](const std::vector<std::string_view>& f, int line) {
        const auto t = ParseNumber<Timestamp>(f[0], Where(csv, line));
        if (!entries.empty()) CheckIncreasing(entries.back().t, t, Where(csv, line));
        entries.push_back({t, camera_dir / "data" / fs::path(std::string(f[1]))});
    });
    if (entries.empty()) Fail(csv.string() + " has no images");
    return entries;
}

// mav0/imu0/data.csv: "timestamp [ns], gyro x y z [rad/s], accel x y z [m/s²]".
std::vector<ImuData> ReadImu(const fs::path& csv) {
    std::vector<ImuData> samples;
    ForEachCsvRow(csv, 7, [&](const std::vector<std::string_view>& f, int line) {
        const std::string where = Where(csv, line);
        ImuData s;
        s.t = ParseNumber<Timestamp>(f[0], where);
        for (int i = 0; i < 3; ++i) {
            s.gyro[i] = ParseNumber<double>(f[1 + i], where);
            s.accel[i] = ParseNumber<double>(f[4 + i], where);
        }
        if (!samples.empty()) CheckIncreasing(samples.back().t, s.t, where);
        samples.push_back(s);
    });
    if (samples.empty()) Fail(csv.string() + " has no samples");
    return samples;
}

YAML::Node LoadYaml(const fs::path& file) {
    if (!fs::exists(file)) Fail("missing " + file.string());
    try {
        return YAML::LoadFile(file.string());
    } catch (const YAML::Exception& e) {
        Fail(file.string() + ": " + e.what());
    }
}

template <typename T>
T Get(const YAML::Node& node, const std::string& key, const fs::path& file) {
    if (!node[key]) Fail(file.string() + ": missing key '" + key + "'");
    try {
        return node[key].as<T>();
    } catch (const YAML::Exception& e) {
        Fail(file.string() + ": bad value for '" + key + "': " + e.what());
    }
}

// T_BS: {rows: 4, cols: 4, data: [16 values, row-major]}.
Eigen::Isometry3d ReadTransform(const YAML::Node& yaml, const fs::path& file) {
    const YAML::Node node = yaml["T_BS"];
    if (!node) Fail(file.string() + ": missing key 'T_BS'");
    const auto data = Get<std::vector<double>>(node, "data", file);
    if (Get<int>(node, "rows", file) != 4 || Get<int>(node, "cols", file) != 4 || data.size() != 16) {
        Fail(file.string() + ": T_BS must be a 4x4 matrix");
    }
    const Eigen::Matrix4d m = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(data.data());
    if (!m.row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1))) Fail(file.string() + ": T_BS last row must be 0 0 0 1");
    const Eigen::Matrix3d R = m.topLeftCorner<3, 3>();
    if (!(R.transpose() * R).isIdentity(1e-6)) Fail(file.string() + ": T_BS rotation is not orthonormal");

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = R;
    T.translation() = m.topRightCorner<3, 1>();
    return T;
}

RawCameraCalibration ReadCamera(const fs::path& file) {
    const YAML::Node yaml = LoadYaml(file);
    if (Get<std::string>(yaml, "camera_model", file) != "pinhole") Fail(file.string() + ": only pinhole is supported");
    if (Get<std::string>(yaml, "distortion_model", file) != "radial-tangential") {
        Fail(file.string() + ": only radial-tangential distortion is supported");
    }
    const auto intrinsics = Get<std::vector<double>>(yaml, "intrinsics", file);
    const auto distortion = Get<std::vector<double>>(yaml, "distortion_coefficients", file);
    const auto resolution = Get<std::vector<int>>(yaml, "resolution", file);
    if (intrinsics.size() != 4 || distortion.size() != 4 || resolution.size() != 2) {
        Fail(file.string() + ": intrinsics and distortion_coefficients need 4 values, resolution 2");
    }

    RawCameraCalibration c;
    c.T_B_C = ReadTransform(yaml, file);
    c.fx = intrinsics[0];
    c.fy = intrinsics[1];
    c.cx = intrinsics[2];
    c.cy = intrinsics[3];
    c.distortion = Eigen::Vector4d(distortion[0], distortion[1], distortion[2], distortion[3]);
    c.width = resolution[0];
    c.height = resolution[1];
    c.rate_hz = Get<double>(yaml, "rate_hz", file);
    return c;
}

ImuNoiseParams ReadImuNoise(const fs::path& file) {
    const YAML::Node yaml = LoadYaml(file);
    ImuNoiseParams p;
    p.T_B_I = ReadTransform(yaml, file);
    p.rate_hz = Get<double>(yaml, "rate_hz", file);
    p.gyro_noise_density = Get<double>(yaml, "gyroscope_noise_density", file);
    p.gyro_random_walk = Get<double>(yaml, "gyroscope_random_walk", file);
    p.accel_noise_density = Get<double>(yaml, "accelerometer_noise_density", file);
    p.accel_random_walk = Get<double>(yaml, "accelerometer_random_walk", file);
    return p;
}

Timestamp SecondsToNs(double s) { return static_cast<Timestamp>(std::llround(s * 1e9)); }

// Index of the first entry with t >= begin.
template <typename Entry>
std::size_t FirstAtOrAfter(const std::vector<Entry>& entries, Timestamp begin) {
    const auto it =
        std::lower_bound(entries.begin(), entries.end(), begin, [](const Entry& e, Timestamp t) { return e.t < t; });
    return static_cast<std::size_t>(it - entries.begin());
}

}  // namespace

EurocReader::EurocReader(const EurocReaderOptions& options) {
    const fs::path mav0 = options.sequence_dir / "mav0";
    if (!fs::is_directory(mav0)) Fail("no mav0/ folder in " + options.sequence_dir.string());
    if (options.start_offset_s < 0.0) Fail("start_offset_s must not be negative");

    calibration_.left = ReadCamera(mav0 / "cam0" / "sensor.yaml");
    calibration_.right = ReadCamera(mav0 / "cam1" / "sensor.yaml");
    imu_noise_ = ReadImuNoise(mav0 / "imu0" / "sensor.yaml");

    imu_ = ReadImu(mav0 / "imu0" / "data.csv");
    left_ = ReadImageList<ImageEntry>(mav0 / "cam0");
    right_ = ReadImageList<ImageEntry>(mav0 / "cam1");

    sequence_start_ = std::min({imu_.front().t, left_.front().t, right_.front().t});
    const Timestamp begin = sequence_start_ + SecondsToNs(options.start_offset_s);
    end_time_ =
        options.duration_s < 0.0 ? std::numeric_limits<Timestamp>::max() : begin + SecondsToNs(options.duration_s);

    next_imu_ = FirstAtOrAfter(imu_, begin);
    next_left_ = FirstAtOrAfter(left_, begin);
    next_right_ = FirstAtOrAfter(right_, begin);
}

std::optional<SensorEvent> EurocReader::Next() {
    // Timestamp of a stream's next entry, or nothing if it is used up or past the end time.
    const auto head = [this](const auto& entries, std::size_t next) -> std::optional<Timestamp> {
        if (next >= entries.size() || entries[next].t > end_time_) return std::nullopt;
        return entries[next].t;
    };
    const std::optional<Timestamp> imu_t = head(imu_, next_imu_);
    const std::optional<Timestamp> left_t = head(left_, next_left_);
    const std::optional<Timestamp> right_t = head(right_, next_right_);

    // The earliest stream wins; ties go to IMU, then left, then right.
    const auto not_after = [](std::optional<Timestamp> a, std::optional<Timestamp> b) { return !b || (a && *a <= *b); };
    if (imu_t && not_after(imu_t, left_t) && not_after(imu_t, right_t)) return imu_[next_imu_++];
    if (left_t && not_after(left_t, right_t)) return LoadImage(left_[next_left_++], CameraId::kLeft);
    if (right_t) return LoadImage(right_[next_right_++], CameraId::kRight);
    return std::nullopt;
}

CameraImage EurocReader::LoadImage(const ImageEntry& entry, CameraId camera) const {
    CameraImage out;
    out.t = entry.t;
    out.camera = camera;
    out.image = cv::imread(entry.path.string(), cv::IMREAD_GRAYSCALE);
    if (out.image.empty()) Fail("cannot read image " + entry.path.string());
    return out;
}

}  // namespace mini_vio
