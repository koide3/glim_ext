#include <thread>
#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

#include <DBoW3.h>

#include <glim/util/concurrent_vector.hpp>

#include <opencv2/features2d.hpp>

class Trainer {
public:
  Trainer() {
    orb = cv::ORB::create(2000);

    const int k = 10;
    const int L = 6;
    const DBoW3::WeightingType weight = DBoW3::TF_IDF;
    const DBoW3::ScoringType scoring = DBoW3::BHATTACHARYYA;
    voc.reset(new DBoW3::Vocabulary(k, L, weight, scoring));

    kill_switch = false;
  }

  ~Trainer() {}

  void add_images(const std::string& path, int steps) {
    boost::filesystem::directory_iterator itr(path);
    boost::filesystem::directory_iterator end;

    std::vector<std::string> filenames;
    for (; itr != end; itr++) {
      const std::string extension = itr->path().extension().string();
      if (extension != ".jpg" && extension != ".png") {
        continue;
      }

      filenames.push_back(itr->path().string());
    }

    std::sort(filenames.begin(), filenames.end());

    for (int i = 0; i < filenames.size(); i += steps) {
      all_filenames.push_back(filenames[i]);
      std::cout << all_filenames.back() << std::endl;
    }
  }

  void save(const std::string& path) { voc->save(path); }

  void process() {
    decoded_images.reserve(all_filenames.size());
    features.reserve(all_filenames.size());

    read_image_thread = std::thread([this] { read_data_task(); });

    decode_image_threads.resize(16);
    for (int i = 0; i < decode_image_threads.size(); i++) {
      decode_image_threads[i] = std::thread([this] { decode_image_task(); });
    }

    int count = 0;
    cv::Mat canvas(256 * 4, 256 * 4, CV_8UC1, cv::Scalar::all(0));
    while (!kill_switch || !image_buffers.empty()) {
      auto images = decoded_images.get_and_clear(16);
      for (int i = 0; i < images.size(); i++) {
        int row = (count / 4) % 4;
        int col = count % 4;

        cv::Mat roi(canvas, cv::Rect(row * 256, col * 256, 256, 256));

        cv::Mat resized;
        cv::resize(images[i], resized, cv::Size(256, 256));

        resized.copyTo(roi);
        count++;
      }

      cv::imshow("images", canvas);
      cv::waitKey(10);
      std::cout << image_buffers.size() << std::endl;
    }

    read_image_thread.join();
    for (auto& thread : decode_image_threads) {
      thread.join();
    }

    train_voc();
  }

  void read_data_task() {
    for (const auto& filename : all_filenames) {
      if (kill_switch) {
        break;
      }

      std::cout << "reading " << filename << std::endl;
      std::ifstream ifs(filename, std::ios::binary | std::ios::ate);
      if (!ifs) {
        std::cerr << "error: failed to open " << filename << std::endl;
        continue;
      }

      std::streamsize bytes = ifs.tellg();
      ifs.seekg(0, std::ios::beg);

      cv::Mat buffer(bytes, 1, CV_8UC1);
      ifs.read(reinterpret_cast<char*>(buffer.data), bytes);
      image_buffers.push_back(buffer);
    }

    kill_switch = true;
  }

  void decode_image_task() {
    while (true) {
      auto buffers = image_buffers.get_and_clear(10);

      if (buffers.empty()) {
        if (kill_switch) {
          break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      std::vector<cv::Mat> decoded;
      std::vector<cv::Mat> descs;

      for (const auto& buffer : buffers) {
        cv::Mat image = cv::imdecode(buffer, 0);

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        orb->detectAndCompute(image, cv::Mat(), keypoints, descriptors);

        decoded.push_back(image);
        descs.push_back(descriptors);
      }

      decoded_images.insert(decoded);
      features.insert(descs);
    }
  }

  void train_voc() {
    std::cout << "training..." << std::endl;
    auto all_features = features.get_all_and_clear();
    std::cout << "num_images:" << all_features.size() << std::endl;
    voc->create(all_features);
    std::cout << "done" << std::endl;

    std::cout << *voc << std::endl;
  }

private:
  std::atomic_bool kill_switch;

  cv::Ptr<cv::ORB> orb;
  std::unique_ptr<DBoW3::Vocabulary> voc;

  std::vector<std::string> all_filenames;

  glim::ConcurrentVector<cv::Mat> image_buffers;
  glim::ConcurrentVector<cv::Mat> decoded_images;
  glim::ConcurrentVector<cv::Mat> features;

  std::thread read_image_thread;
  std::vector<std::thread> decode_image_threads;
};

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: dbow2_training voc_dst_path training_images_dir1 step1 training_images_dir2 steps2 ..." << std::endl;
    return 0;
  }

  const std::string& dst_path = argv[1];

  Trainer trainer;

  for (int i = 2; i < argc; i += 2) {
    const std::string images_dir = argv[i];
    const int steps = std::stoi(argv[i + 1]);
    trainer.add_images(images_dir, steps);
  }

  trainer.process();
  trainer.save(dst_path);

  return 0;
}