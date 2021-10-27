#include <mutex>
#include <thread>
#include <vector>
#include <iostream>
#include <boost/format.hpp>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_ext/types/frame_cpu.hpp>
#include <gtsam_ext/factors/integrated_gicp_factor.hpp>
#include <gtsam_ext/optimizers/levenberg_marquardt_ext.hpp>

#include <glim/frontend/estimation_frame.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Scancontext/Scancontext.h>

bool validate_loop(const gtsam_ext::Frame::ConstPtr& frame1, const gtsam_ext::Frame::ConstPtr& frame2, Eigen::Isometry3d& T_frame1_frame2) {
  gtsam::Values values;
  values.insert(0, gtsam::Pose3::identity());
  values.insert(1, gtsam::Pose3(T_frame1_frame2.matrix()));

  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(0, gtsam::Pose3::identity(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

  auto factor = gtsam::make_shared<gtsam_ext::IntegratedGICPFactor>(0, 1, frame1, frame2);
  graph.add(factor);

  values.print();

  gtsam_ext::LevenbergMarquardtExtParams lm_params;
  lm_params.setlambdaInitial(1e-12);
  lm_params.setMaxIterations(5);
  lm_params.callback = [](const gtsam_ext::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values) { std::cout << status.to_string() << std::endl; };
  gtsam_ext::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
  values = optimizer.optimize();

  T_frame1_frame2 = Eigen::Isometry3d(values.at<gtsam::Pose3>(1).matrix());

  std::cout << "overlap:" << factor->inlier_fraction() << std::endl;
  values.print();

  return factor->inlier_fraction() > 0.8;
}

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();

  std::unique_ptr<SCManager> sc(new SCManager);

  for (int i = 0; i < 14223; i++) {
    const auto frame = gtsam_ext::FrameCPU::load((boost::format("/tmp/frames/%06d") % i).str());
    if (!frame) {
      return 1;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.resize(frame->size());

    for (int j = 0; j < frame->size(); j++) {
      cloud.at(j).getVector3fMap() = frame->points[j].head<3>().cast<float>();
    }

    sc->makeAndSaveScancontextAndKeys(cloud);
    auto loop = sc->detectLoopClosureID();

    if (loop.first < 0) {
      viewer->append_text("no loop");
      continue;
    }

    Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
    init_guess.linear() = Eigen::AngleAxisd(loop.second, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    const auto frame2 = gtsam_ext::FrameCPU::load((boost::format("/tmp/frames/%06d") % loop.first).str());

    validate_loop(frame, frame2, init_guess);

    auto cloud_buffer1 = std::make_shared<glk::PointCloudBuffer>(frame->points, frame->size());
    auto cloud_buffer2 = std::make_shared<glk::PointCloudBuffer>(frame2->points, frame2->size());
    viewer->append_text((boost::format("loop: %d %d %.3f") % i % loop.first % loop.second).str());

    viewer->update_drawable("frame1", cloud_buffer1, guik::FlatRed());
    viewer->update_drawable("frame2", cloud_buffer2, guik::FlatGreen(init_guess.cast<float>()));
    viewer->spin_until_click();
  }
  return 0;
}