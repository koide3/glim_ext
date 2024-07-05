#pragma once

#include <memory>
#include <Eigen/Core>
#include <nanoflann.hpp>

namespace glim {

struct PositionKdTree {
public:
  explicit PositionKdTree(const std::pair<int, Eigen::Vector3d>* positions, size_t num_data) : positions(positions), num_data(num_data) {
    index.reset(new Index(2, *this, nanoflann::KDTreeSingleIndexAdaptorParams(10)));
    index->buildIndex();
  }

  inline size_t kdtree_get_point_count() const { return num_data; }

  inline double kdtree_get_pt(const size_t idx, const size_t dim) const { return positions[idx].second[dim]; }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }

  size_t knn_search(const double* query, int k, size_t* k_indices, double* k_sq_dists) const { return index->knnSearch(query, k, k_indices, k_sq_dists); }

private:
  const std::pair<int, Eigen::Vector3d>* positions;
  const size_t num_data;

  using Index = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PositionKdTree, double>, PositionKdTree, 2, size_t>;
  std::unique_ptr<Index> index;
};

}