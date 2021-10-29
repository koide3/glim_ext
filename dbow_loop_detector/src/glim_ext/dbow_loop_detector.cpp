#include <glim_ext/dbow_loop_detector.hpp>

#include <atomic>
#include <thread>
#include <boost/format.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <DBoW2.h>

#include <glim/backend/callbacks.hpp>
#include <glim/util/concurrent_vector.hpp>

#include <glk/texture.hpp>
#include <glk/texture_opencv.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

class DBoWLoopDetector::Impl {
public:
  Impl() {
    // Create DBoW2 vocabulary
    const int k = 9;
    const int L = 3;
    const DBoW2::WeightingType weight = DBoW2::TF_IDF;
    const DBoW2::ScoringType scoring = DBoW2::L1_NORM;
    voc.reset(new OrbVocabulary(k, L, weight, scoring));

    std::cout << *voc << std::endl;

    feature_detector = cv::ORB::create();

    last_image_stamp = std::chrono::high_resolution_clock::now();

    // Setting up callbacks
    using std::placeholders::_1;
    using std::placeholders::_2;
    GlobalMappingCallbacks::on_insert_image.add(std::bind(&Impl::on_insert_image, this, _1, _2));

    kill_switch = false;
    thread = std::thread([this] { loop_detection_task(); });
  }

  ~Impl() {
    kill_switch = true;
    if (thread.joinable()) {
      thread.join();
    }
  }

  void on_insert_image(const double stamp, const cv::Mat& image) {
    auto since_last_image = std::chrono::high_resolution_clock::now() - last_image_stamp;
    if (since_last_image < std::chrono::milliseconds(500)) {
      return;
    }

    last_image_stamp = std::chrono::high_resolution_clock::now();
    input_image_queue.push_back(std::make_pair(stamp, image));
  }

  void loop_detection_task() {
    while (!kill_switch) {
      auto images = input_image_queue.get_all_and_clear();
      if (images.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      for (auto& stamp_image : images) {
        const double stamp = stamp_image.first;
        auto& image = stamp_image.second;

        if (image.cols > 640) {
          cv::Mat resized;
          const double scale = 640.0 / image.rows;
          cv::resize(image, resized, cv::Size(0, 0), scale, scale);
          image = resized;
        }

        cv::Mat descriptors;
        std::vector<cv::KeyPoint> keypoints;
        feature_detector->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
      }
    }
  }

private:
  std::chrono::high_resolution_clock::time_point last_image_stamp;
  ConcurrentVector<std::pair<double, cv::Mat>> input_image_queue;

  std::unique_ptr<OrbVocabulary> voc;
  cv::Ptr<cv::Feature2D> feature_detector;

  std::atomic_bool kill_switch;
  std::thread thread;
};

DBoWLoopDetector::DBoWLoopDetector() {
  impl.reset(new Impl);
}

DBoWLoopDetector::~DBoWLoopDetector() {}

}  // namespace glim
