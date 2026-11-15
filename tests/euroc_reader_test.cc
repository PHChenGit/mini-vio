#include "mini_vio/euroc_reader.h"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "mini_vio/dataset_reader.h"

namespace mini_vio {
namespace {

namespace fs = std::filesystem;

static_assert(DatasetReader<EurocReader>);

constexpr Timestamp kT0 = 1'000'000'000'000;  // first timestamp of the fake sequence
constexpr Timestamp kMs = 1'000'000;          // one millisecond in ns

void WriteFile(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream(path, std::ios::binary) << text;
}

std::string CameraYaml(double tx) {
    return "sensor_type: camera\n"
           "T_BS:\n  cols: 4\n  rows: 4\n"
           "  data: [0.0, -1.0, 0.0, " +
           std::to_string(tx) +
           ",\n         1.0, 0.0, 0.0, 0.0,\n"
           "         0.0, 0.0, 1.0, 0.0,\n         0.0, 0.0, 0.0, 1.0]\n"
           "rate_hz: 20\nresolution: [8, 4]\ncamera_model: pinhole\n"
           "intrinsics: [458.654, 457.296, 367.215, 248.375] #fu, fv, cu, cv\n"
           "distortion_model: radial-tangential\n"
           "distortion_coefficients: [-0.28, 0.07, 0.0002, 1.8e-05]\n";
}

const char* const kImuYaml =
    "sensor_type: imu\n"
    "T_BS:\n  cols: 4\n  rows: 4\n"
    "  data: [1.0, 0.0, 0.0, 0.0,\n         0.0, 1.0, 0.0, 0.0,\n"
    "         0.0, 0.0, 1.0, 0.0,\n         0.0, 0.0, 0.0, 1.0]\n"
    "rate_hz: 200\n"
    "gyroscope_noise_density: 1.6968e-04\ngyroscope_random_walk: 1.9393e-05\n"
    "accelerometer_noise_density: 2.0000e-3\naccelerometer_random_walk: 3.0000e-3\n";

// A small sequence shaped like EuRoC: images every 50 ms (left and right at the
// same times, camera CSVs with CRLF), IMU every 5 ms starting at the first image
// time, so IMU and images share timestamps.
class EurocReaderTest : public ::testing::Test {
protected:
    static constexpr int kNumImages = 4;
    static constexpr int kNumImu = 31;  // 0 .. 150 ms

    void SetUp() override {
        dir_ = fs::temp_directory_path() /
               ("euroc_reader_test_" + std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()));
        fs::remove_all(dir_);
        const fs::path mav0 = dir_ / "mav0";

        for (int cam = 0; cam < 2; ++cam) {
            const fs::path cam_dir = mav0 / ("cam" + std::to_string(cam));
            std::string csv = "#timestamp [ns],filename\r\n";
            for (int i = 0; i < kNumImages; ++i) {
                const Timestamp t = kT0 + static_cast<Timestamp>(i) * 50 * kMs;
                const std::string name = std::to_string(t) + ".png";
                csv += std::to_string(t) + "," + name + "\r\n";
                fs::create_directories(cam_dir / "data");
                // Pixel value encodes camera and index, so tests can tell images apart.
                cv::imwrite((cam_dir / "data" / name).string(), cv::Mat(4, 8, CV_8UC1, cv::Scalar(100 * cam + i)));
            }
            WriteFile(cam_dir / "data.csv", csv);
            WriteFile(cam_dir / "sensor.yaml", CameraYaml(cam == 0 ? -0.02 : 0.09));
        }

        std::string imu =
            "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
            "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n";
        for (int i = 0; i < kNumImu; ++i) {
            imu += std::to_string(kT0 + static_cast<Timestamp>(i) * 5 * kMs) + ",0.1,0.2," + std::to_string(i) +
                   ",9.81,0,-0.5\n";
        }
        WriteFile(mav0 / "imu0" / "data.csv", imu);
        WriteFile(mav0 / "imu0" / "sensor.yaml", kImuYaml);
    }

    void TearDown() override { fs::remove_all(dir_); }

    EurocReaderOptions Options() const { return {.sequence_dir = dir_}; }

    static std::vector<SensorEvent> ReadAll(EurocReader& reader) {
        std::vector<SensorEvent> events;
        while (std::optional<SensorEvent> e = reader.Next()) events.push_back(std::move(*e));
        return events;
    }

    static Timestamp TimeOf(const SensorEvent& e) {
        return std::visit([](const auto& v) { return v.t; }, e);
    }

    fs::path dir_;
};

TEST_F(EurocReaderTest, ReadsCameraCalibration) {
    const RawStereoCalibration calib = EurocReader(Options()).CameraCalibration();
    EXPECT_DOUBLE_EQ(calib.left.fx, 458.654);
    EXPECT_DOUBLE_EQ(calib.left.cy, 248.375);
    EXPECT_DOUBLE_EQ(calib.left.distortion[3], 1.8e-05);
    EXPECT_EQ(calib.left.width, 8);
    EXPECT_EQ(calib.left.height, 4);
    EXPECT_DOUBLE_EQ(calib.left.rate_hz, 20.0);
    // T_BS is row-major: the first row is [0 -1 0 tx].
    EXPECT_DOUBLE_EQ(calib.left.T_B_C.linear()(0, 1), -1.0);
    EXPECT_DOUBLE_EQ(calib.left.T_B_C.linear()(1, 0), 1.0);
    EXPECT_DOUBLE_EQ(calib.left.T_B_C.translation().x(), -0.02);
    EXPECT_DOUBLE_EQ(calib.right.T_B_C.translation().x(), 0.09);
}

TEST_F(EurocReaderTest, ReadsImuNoise) {
    const ImuNoiseParams noise = EurocReader(Options()).ImuNoise();
    EXPECT_DOUBLE_EQ(noise.rate_hz, 200.0);
    EXPECT_DOUBLE_EQ(noise.gyro_noise_density, 1.6968e-04);
    EXPECT_DOUBLE_EQ(noise.gyro_random_walk, 1.9393e-05);
    EXPECT_DOUBLE_EQ(noise.accel_noise_density, 2.0e-3);
    EXPECT_DOUBLE_EQ(noise.accel_random_walk, 3.0e-3);
    EXPECT_TRUE(noise.T_B_I.isApprox(Eigen::Isometry3d::Identity()));
}

TEST_F(EurocReaderTest, EmitsEveryEventInTimeOrderWithImuFirstOnTies) {
    EurocReader reader(Options());
    EXPECT_EQ(reader.SequenceStart(), kT0);
    const std::vector<SensorEvent> events = ReadAll(reader);
    ASSERT_EQ(events.size(), static_cast<std::size_t>(kNumImu + 2 * kNumImages));

    for (std::size_t i = 1; i < events.size(); ++i) EXPECT_LE(TimeOf(events[i - 1]), TimeOf(events[i]));

    // At kT0 the IMU sample, the left image and the right image share a timestamp.
    ASSERT_TRUE(std::holds_alternative<ImuData>(events[0]));
    ASSERT_TRUE(std::holds_alternative<CameraImage>(events[1]));
    ASSERT_TRUE(std::holds_alternative<CameraImage>(events[2]));
    EXPECT_EQ(std::get<CameraImage>(events[1]).camera, CameraId::kLeft);
    EXPECT_EQ(std::get<CameraImage>(events[2]).camera, CameraId::kRight);
}

TEST_F(EurocReaderTest, ParsesImuValues) {
    EurocReader reader(Options());
    const std::optional<SensorEvent> first = reader.Next();
    ASSERT_TRUE(first.has_value());
    const auto& imu = std::get<ImuData>(*first);
    EXPECT_EQ(imu.t, kT0);
    EXPECT_TRUE(imu.gyro.isApprox(Eigen::Vector3d(0.1, 0.2, 0.0)));
    EXPECT_TRUE(imu.accel.isApprox(Eigen::Vector3d(9.81, 0.0, -0.5)));
}

TEST_F(EurocReaderTest, LoadsGrayscaleImages) {
    EurocReader reader(Options());
    int left = 0;
    int right = 0;
    while (std::optional<SensorEvent> e = reader.Next()) {
        if (const auto* img = std::get_if<CameraImage>(&*e)) {
            ASSERT_EQ(img->image.type(), CV_8UC1);
            ASSERT_EQ(img->image.size(), cv::Size(8, 4));
            const int index = static_cast<int>((img->t - kT0) / (50 * kMs));
            const int cam = img->camera == CameraId::kLeft ? 0 : 1;
            EXPECT_EQ(img->image.at<uint8_t>(0, 0), 100 * cam + index);
            (cam == 0 ? left : right)++;
        }
    }
    EXPECT_EQ(left, kNumImages);
    EXPECT_EQ(right, kNumImages);
}

TEST_F(EurocReaderTest, StartOffsetAndDurationLimitTheWindow) {
    EurocReaderOptions options = Options();
    options.start_offset_s = 0.05;  // first event at kT0 + 50 ms
    options.duration_s = 0.05;      // last event at kT0 + 100 ms (inclusive)
    EurocReader reader(options);
    EXPECT_EQ(reader.SequenceStart(), kT0);  // still the start of the whole sequence

    const std::vector<SensorEvent> events = ReadAll(reader);
    ASSERT_FALSE(events.empty());
    EXPECT_EQ(TimeOf(events.front()), kT0 + 50 * kMs);
    EXPECT_EQ(TimeOf(events.back()), kT0 + 100 * kMs);
    // IMU at 50, 55, ..., 100 ms (11 samples) and images at 50 and 100 ms (2 per camera).
    EXPECT_EQ(events.size(), 11u + 4u);
}

TEST_F(EurocReaderTest, ThrowsWhenSequenceFolderHasNoMav0) {
    EXPECT_THROW(EurocReader({.sequence_dir = dir_ / "nowhere"}), std::runtime_error);
}

TEST_F(EurocReaderTest, ThrowsWithLineNumberOnMalformedCsv) {
    WriteFile(dir_ / "mav0" / "imu0" / "data.csv", "#header\n1000,0.1,0.2,0.3,9.8,0,0\n2000,0.1,oops,0.3,9.8,0,0\n");
    try {
        EurocReader reader(Options());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("data.csv:3"), std::string::npos) << e.what();
        EXPECT_NE(std::string(e.what()).find("oops"), std::string::npos) << e.what();
    }
}

TEST_F(EurocReaderTest, ThrowsOnTimestampsOutOfOrder) {
    WriteFile(dir_ / "mav0" / "imu0" / "data.csv", "2000,0,0,0,0,0,0\n1000,0,0,0,0,0,0\n");
    EXPECT_THROW(EurocReader{Options()}, std::runtime_error);
}

TEST_F(EurocReaderTest, ThrowsWhenImageIsMissing) {
    const fs::path missing = dir_ / "mav0" / "cam1" / "data" / (std::to_string(kT0) + ".png");
    fs::remove(missing);
    EurocReader reader(Options());
    EXPECT_THROW(ReadAll(reader), std::runtime_error);
}

// Runs on the real MH_01_easy when the dataset is mounted (it is at /data/euroc
// in the container; override with EUROC_DIR). Reads the first 10 s only.
TEST(EurocReaderRealData, Mh01FirstTenSeconds) {
    const char* env = std::getenv("EUROC_DIR");
    const fs::path seq = fs::path(env != nullptr ? env : "/data/euroc") / "machine_hall" / "MH_01_easy";
    if (!fs::is_directory(seq / "mav0")) GTEST_SKIP() << "MH_01_easy not found at " << seq;

    EurocReader reader({.sequence_dir = seq, .duration_s = 10.0});
    const RawStereoCalibration calib = reader.CameraCalibration();
    EXPECT_DOUBLE_EQ(calib.left.fx, 458.654);
    EXPECT_EQ(calib.left.width, 752);
    EXPECT_DOUBLE_EQ(reader.ImuNoise().rate_hz, 200.0);

    int imu = 0;
    int left = 0;
    int right = 0;
    Timestamp last = 0;
    std::optional<ImuData> first_imu;
    while (std::optional<SensorEvent> e = reader.Next()) {
        const Timestamp t = std::visit([](const auto& v) { return v.t; }, *e);
        ASSERT_GE(t, last);
        last = t;
        if (const auto* s = std::get_if<ImuData>(&*e)) {
            if (!first_imu) first_imu = *s;
            ++imu;
        } else {
            const auto& img = std::get<CameraImage>(*e);
            ASSERT_EQ(img.image.size(), cv::Size(752, 480));
            (img.camera == CameraId::kLeft ? left : right)++;
        }
    }
    // 10 s at 200 Hz and 20 Hz, give or take the samples on the window edges.
    EXPECT_NEAR(imu, 2000, 2);
    EXPECT_NEAR(left, 200, 1);
    EXPECT_EQ(left, right);
    // At the start of MH_01 the IMU's x axis points roughly up (gravity on +x).
    ASSERT_TRUE(first_imu.has_value());
    EXPECT_GT(first_imu->accel.x(), 7.0);
}

}  // namespace
}  // namespace mini_vio
