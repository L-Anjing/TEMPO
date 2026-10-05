#include <gtest/gtest.h>
#include <random>
#include "ivox/ivox3d_node.hpp"

TEST(IvoxNearestPoints, MatchesBruteForceAndPreservesPreviousNodes)
{
  using Node = faster_lio::IVoxNode<pcl::PointXYZ, 3>;
  Node node;
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> uniform(-2.f, 2.f);
  for (int i = 0; i < 3000; ++i) { node.InsertPoint(pcl::PointXYZ(uniform(rng), uniform(rng), uniform(rng))); }
  for (int query = 0; query < 100; ++query) {
    pcl::PointXYZ point(uniform(rng), uniform(rng), uniform(rng));
    for (int k : {0, 1, 5, 20}) {
      for (double radius : {.05, .5, 5.}) {
        std::vector<Node::DistPoint> result{{123., nullptr, 99}};
        node.KNNPointByCondition(result, point, k, radius);
        EXPECT_EQ(result[0].idx, 99);
        EXPECT_EQ(result[0].dist, 123.);
        std::vector<double> expected, actual;
        for (size_t i = 0; i < node.Size(); ++i) {
          const auto d = faster_lio::distance2(node.GetPoint(i), point);
          if (d < radius * radius) { expected.push_back(d); }
        }
        std::sort(expected.begin(), expected.end());
        expected.resize(std::min(expected.size(), static_cast<size_t>(k)));
        for (size_t i = 1; i < result.size(); ++i) { actual.push_back(result[i].dist); }
        std::sort(actual.begin(), actual.end());
        EXPECT_EQ(expected, actual);
      }
    }
  }
}
