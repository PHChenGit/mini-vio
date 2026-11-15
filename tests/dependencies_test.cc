// Smoke tests for the build environment: each dependency compiles, links and
// does one small real computation. If one of these fails, fix the container
// before looking for bugs in mini-VIO itself.

#include <ceres/ceres.h>
#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rerun.hpp>

TEST(Dependencies, Eigen) {
    Eigen::Matrix3d A;
    A << 4, 1, 0, 1, 3, 1, 0, 1, 2;
    const Eigen::Vector3d b(1, 2, 3);
    const Eigen::Vector3d x = A.ldlt().solve(b);
    EXPECT_TRUE((A * x).isApprox(b));
}

TEST(Dependencies, OpenCV) {
    cv::Mat img = cv::Mat::zeros(48, 64, CV_8UC1);
    cv::circle(img, {32, 24}, 10, cv::Scalar(255), cv::FILLED);
    EXPECT_GT(cv::countNonZero(img), 0);
}

TEST(Dependencies, YamlCpp) {
    // Same layout as T_BS in EuRoC's sensor.yaml.
    const YAML::Node n = YAML::Load("T_BS: {cols: 4, rows: 4, data: [1, 0, 0, 0.5]}");
    EXPECT_EQ(n["T_BS"]["rows"].as<int>(), 4);
    EXPECT_DOUBLE_EQ(n["T_BS"]["data"][3].as<double>(), 0.5);
}

// Two poses, a prior and an odometry factor, solved by the same smoother the
// back end will use.
TEST(Dependencies, GtsamFixedLagSmoother) {
    using gtsam::symbol_shorthand::X;
    gtsam::IncrementalFixedLagSmoother smoother(3.0);

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    gtsam::FixedLagSmoother::KeyTimestampMap stamps;
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.01);
    const gtsam::Pose3 step(gtsam::Rot3::Yaw(0.1), gtsam::Point3(1, 0, 0));

    graph.addPrior(X(0), gtsam::Pose3(), noise);
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(0), X(1), step, noise);
    initial.insert(X(0), gtsam::Pose3());
    initial.insert(X(1), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.8, 0.1, 0)));
    stamps[X(0)] = 0.0;
    stamps[X(1)] = 0.25;

    smoother.update(graph, initial, stamps);
    const auto X1 = smoother.calculateEstimate<gtsam::Pose3>(X(1));
    EXPECT_TRUE(X1.equals(step, 1e-3));
}

struct DistanceToTen {
    template <typename T>
    bool operator()(const T* x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};

TEST(Dependencies, CeresAutoDiff) {
    double x = 0.5;
    ceres::Problem problem;
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<DistanceToTen, 1, 1>(new DistanceToTen), nullptr, &x);
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    EXPECT_TRUE(summary.IsSolutionUsable());
    EXPECT_NEAR(x, 10.0, 1e-6);
}

TEST(Dependencies, RerunWritesRecording) {
    const auto path = std::filesystem::temp_directory_path() / "mini_vio_dependencies_test.rrd";
    {
        const rerun::RecordingStream rec("mini_vio_dependencies_test");
        ASSERT_TRUE(rec.save(path.string()).is_ok());
        rec.log("world/points", rerun::Points3D({{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}}));
    }
    EXPECT_TRUE(std::filesystem::exists(path));
    EXPECT_GT(std::filesystem::file_size(path), 0u);
    std::filesystem::remove(path);
}
