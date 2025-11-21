// tests/nn/ann_index_test.cpp
#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>

#include "utils/nn/ann_index.h"

using namespace factorlib::nn;

class ANNIndexTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 2D 测试数据
        points_2d_ = {
            {0.0, 0.0},
            {1.0, 0.0},
            {0.0, 1.0},
            {1.0, 1.0},
            {0.5, 0.5}
        };

        // 3D 测试数据
        points_3d_ = {
            {0.0, 0.0, 0.0},
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0},
            {1.0, 1.0, 1.0}
        };

        // 随机测试数据
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 10.0);

        random_points_.resize(100);
        for (auto& point : random_points_) {
            point = {dis(gen), dis(gen), dis(gen)};
        }
    }

    std::vector<std::vector<double>> points_2d_;
    std::vector<std::vector<double>> points_3d_;
    std::vector<std::vector<double>> random_points_;
};

// ============================================================================
// 基本功能测试
// ============================================================================

TEST_F(ANNIndexTest, BuildAndQuery2D) {
    KDTreeANN index;
    index.build(points_2d_);

    // 查询点 (0.2, 0.2)，应该最接近 (0.0, 0.0) 或 (0.5, 0.5)
    std::vector<double> query = {0.2, 0.2};
    std::vector<KDTreeANN::IndexType> indices(2);
    std::vector<double> distances(2);

    index.knn(query, 2, indices, distances);

    // 应该返回2个最近邻
    EXPECT_EQ(indices.size(), 2);
    EXPECT_EQ(distances.size(), 2);

    // 第一个最近邻应该是自己（如果查询点在数据集中）或真正的最近邻
    // 由于查询点不在数据集中，第一个就是真正的最近邻
    EXPECT_GE(indices[0], 0);
    EXPECT_LT(indices[0], points_2d_.size());
    EXPECT_GE(distances[0], 0.0);
}

TEST_F(ANNIndexTest, BuildAndQuery3D) {
    KDTreeANN index;
    index.build(points_3d_);

    std::vector<double> query = {0.1, 0.1, 0.1};
    std::vector<KDTreeANN::IndexType> indices(3);
    std::vector<double> distances(3);

    index.knn(query, 3, indices, distances);

    EXPECT_EQ(indices.size(), 3);
    EXPECT_EQ(distances.size(), 3);

    // 所有索引应该在有效范围内
    for (auto idx : indices) {
        EXPECT_GE(idx, 0);
        EXPECT_LT(idx, points_3d_.size());
    }

    // 距离应该非负且按升序排列
    for (auto dist : distances) {
        EXPECT_GE(dist, 0.0);
    }
    EXPECT_LE(distances[0], distances[1]);
    EXPECT_LE(distances[1], distances[2]);
}

// ============================================================================
// 边界情况测试
// ============================================================================

TEST_F(ANNIndexTest, EmptyPoints) {
    KDTreeANN index;
    std::vector<std::vector<double>> empty_points;

    // 构建空点集应该不崩溃
    EXPECT_NO_THROW(index.build(empty_points));

    // 在空索引上查询
    std::vector<double> query = {1.0, 2.0};
    std::vector<KDTreeANN::IndexType> indices(2);
    std::vector<double> distances(2);

    // 应该能够处理，但结果可能未定义
    // 主要测试不崩溃
    EXPECT_NO_THROW(index.knn(query, 2, indices, distances));
}

TEST_F(ANNIndexTest, SinglePoint) {
    std::vector<std::vector<double>> single_point = {{1.0, 2.0, 3.0}};

    KDTreeANN index;
    index.build(single_point);

    std::vector<double> query = {1.0, 2.0, 3.0};  // 完全相同的点
    std::vector<KDTreeANN::IndexType> indices(2);
    std::vector<double> distances(2);

    index.knn(query, 2, indices, distances);

    // 第一个结果应该是点0
    EXPECT_EQ(indices[0], 0);
    EXPECT_NEAR(distances[0], 0.0, 1e-10);
}

TEST_F(ANNIndexTest, DuplicatePoints) {
    std::vector<std::vector<double>> duplicates = {
        {1.0, 1.0},
        {1.0, 1.0},  // 重复点
        {2.0, 2.0},
        {1.0, 1.0}   // 另一个重复点
    };

    KDTreeANN index;
    index.build(duplicates);

    std::vector<double> query = {1.0, 1.0};
    std::vector<KDTreeANN::IndexType> indices(4);
    std::vector<double> distances(4);

    index.knn(query, 4, indices, distances);

    // 应该返回所有4个点
    EXPECT_EQ(indices.size(), 4);

    // 前三个结果的距离应该为0（重复点）
    EXPECT_NEAR(distances[0], 0.0, 1e-10);
    EXPECT_NEAR(distances[1], 0.0, 1e-10);
    EXPECT_NEAR(distances[2], 0.0, 1e-10);

    // 最后一个点距离应该较大
    EXPECT_GT(distances[3], 0.0);
}

// ============================================================================
// 查询参数测试
// ============================================================================

TEST_F(ANNIndexTest, DifferentKValues) {
    KDTreeANN index;
    index.build(points_2d_);

    std::vector<double> query = {0.3, 0.3};

    // 测试不同的k值
    for (int k = 1; k <= 3; ++k) {
        std::vector<KDTreeANN::IndexType> indices(k);
        std::vector<double> distances(k);

        index.knn(query, k, indices, distances);

        EXPECT_EQ(indices.size(), k);
        EXPECT_EQ(distances.size(), k);

        // 所有索引应该唯一（除非有重复点）
        for (size_t i = 0; i < indices.size(); ++i) {
            for (size_t j = i + 1; j < indices.size(); ++j) {
                if (points_2d_[indices[i]] != points_2d_[indices[j]]) {
                    EXPECT_NE(indices[i], indices[j]);
                }
            }
        }
    }
}

TEST_F(ANNIndexTest, KGreaterThanPoints) {
    KDTreeANN index;
    index.build(points_2d_);

    std::vector<double> query = {0.5, 0.5};
    std::vector<KDTreeANN::IndexType> indices(10);  // k > 点数
    std::vector<double> distances(10);

    // 应该能够处理 k > 点数的情况
    EXPECT_NO_THROW(index.knn(query, 10, indices, distances));

    // 实际返回的点数应该不超过总点数
    // nanoflann 可能会返回所有点，但距离可能包含无效值
}

// ============================================================================
// 正确性验证测试
// ============================================================================

TEST_F(ANNIndexTest, CorrectNearestNeighbor) {
    KDTreeANN index;
    index.build(points_2d_);

    // 查询点很接近 (0.5, 0.5)
    std::vector<double> query = {0.6, 0.6};
    std::vector<KDTreeANN::IndexType> indices(1);
    std::vector<double> distances(1);

    index.knn(query, 1, indices, distances);

    // 最近邻应该是点4: (0.5, 0.5)
    EXPECT_EQ(indices[0], 4);

    // 验证距离计算：sqrt((0.6-0.5)^2 + (0.6-0.5)^2) = sqrt(0.02) ≈ 0.1414
    double expected_dist = std::sqrt(0.01 + 0.01);
    EXPECT_NEAR(distances[0], expected_dist * expected_dist, 1e-10);  // 注意：返回的是平方距离
}

TEST_F(ANNIndexTest, ExactMatchQuery) {
    KDTreeANN index;
    index.build(points_2d_);

    // 查询一个实际存在的点
    std::vector<double> query = points_2d_[2];  // (0.0, 1.0)
    std::vector<KDTreeANN::IndexType> indices(2);
    std::vector<double> distances(2);

    index.knn(query, 2, indices, distances);

    // 第一个结果应该是自己，距离为0
    EXPECT_EQ(indices[0], 2);
    EXPECT_NEAR(distances[0], 0.0, 1e-10);

    // 第二个结果应该是另一个点
    EXPECT_NE(indices[1], 2);
    EXPECT_GT(distances[1], 0.0);
}

// ============================================================================
// 性能测试
// ============================================================================

TEST_F(ANNIndexTest, PerformanceLargeDataset) {
    // 创建较大的测试数据集
    const int num_points = 1000;
    const int dim = 3;
    std::vector<std::vector<double>> large_dataset(num_points);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 100.0);

    for (int i = 0; i < num_points; ++i) {
        large_dataset[i].resize(dim);
        for (int j = 0; j < dim; ++j) {
            large_dataset[i][j] = dis(gen);
        }
    }

    KDTreeANN index;

    // 构建时间测试
    EXPECT_NO_THROW(index.build(large_dataset));

    // 查询时间测试
    std::vector<double> query(dim, 50.0);  // 中心点
    std::vector<KDTreeANN::IndexType> indices(10);
    std::vector<double> distances(10);

    EXPECT_NO_THROW(index.knn(query, 10, indices, distances));

    // 验证结果
    for (auto idx : indices) {
        EXPECT_GE(idx, 0);
        EXPECT_LT(idx, num_points);
    }
}

// ============================================================================
// 随机性测试
// ============================================================================

TEST_F(ANNIndexTest, RandomQueries) {
    KDTreeANN index;
    index.build(random_points_);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 10.0);

    // 多次随机查询测试
    for (int i = 0; i < 10; ++i) {
        std::vector<double> query = {dis(gen), dis(gen), dis(gen)};
        std::vector<KDTreeANN::IndexType> indices(5);
        std::vector<double> distances(5);

        EXPECT_NO_THROW(index.knn(query, 5, indices, distances));

        // 基本验证
        for (auto idx : indices) {
            EXPECT_GE(idx, 0);
            EXPECT_LT(idx, random_points_.size());
        }

        for (auto dist : distances) {
            EXPECT_GE(dist, 0.0);
        }
    }
}

// ============================================================================
// 重建测试
// ============================================================================

TEST_F(ANNIndexTest, RebuildWithDifferentData) {
    KDTreeANN index;

    // 第一次构建
    index.build(points_2d_);

    std::vector<double> query = {0.5, 0.5};
    std::vector<KDTreeANN::IndexType> indices1(2);
    std::vector<double> distances1(2);
    index.knn(query, 2, indices1, distances1);

    // 用不同的数据重建
    index.build(points_3d_);

    std::vector<double> query3d = {0.5, 0.5, 0.5};
    std::vector<KDTreeANN::IndexType> indices2(2);
    std::vector<double> distances2(2);
    index.knn(query3d, 2, indices2, distances2);

    // 两次查询应该得到不同的结果
    // 主要测试重建功能不崩溃
    SUCCEED();
}

// ============================================================================
// 与暴力搜索对比测试
// ============================================================================

// 暴力搜索实现，用于验证 KD-Tree 的正确性
std::vector<std::pair<size_t, double>> brute_force_knn(
    const std::vector<std::vector<double>>& points,
    const std::vector<double>& query,
    size_t k) {

    std::vector<std::pair<size_t, double>> results;
    for (size_t i = 0; i < points.size(); ++i) {
        double dist_sq = 0.0;
        for (size_t j = 0; j < query.size(); ++j) {
            double diff = points[i][j] - query[j];
            dist_sq += diff * diff;
        }
        results.emplace_back(i, dist_sq);
    }

    // 按距离排序
    std::sort(results.begin(), results.end(),
              [](const auto& a, const auto& b) {
                  return a.second < b.second;
              });

    if (results.size() > k) {
        results.resize(k);
    }

    return results;
}

TEST_F(ANNIndexTest, CompareWithBruteForce) {
    KDTreeANN index;
    index.build(points_2d_);

    std::vector<double> query = {0.3, 0.7};

    // KD-Tree 查询
    std::vector<KDTreeANN::IndexType> kd_indices(3);
    std::vector<double> kd_distances(3);
    index.knn(query, 3, kd_indices, kd_distances);

    // 暴力搜索
    auto bf_results = brute_force_knn(points_2d_, query, 3);

    // 比较结果
    EXPECT_EQ(kd_indices.size(), bf_results.size());

    for (size_t i = 0; i < kd_indices.size(); ++i) {
        // KD-Tree 应该找到相同的最近邻（顺序可能不同，但距离应该匹配）
        double kd_dist = kd_distances[i];
        double bf_dist = bf_results[i].second;

        // 允许小的数值误差
        EXPECT_NEAR(kd_dist, bf_dist, 1e-10);
    }
}

// ============================================================================
// 维度一致性测试
// ============================================================================

TEST_F(ANNIndexTest, ConsistentDimensions) {
    // 测试所有点维度一致的情况
    std::vector<std::vector<double>> consistent_points = {
        {1.0, 2.0},
        {3.0, 4.0},
        {5.0, 6.0}
    };

    KDTreeANN index;
    EXPECT_NO_THROW(index.build(consistent_points));

    // 查询维度也必须匹配
    std::vector<double> query = {2.0, 3.0};  // 2D查询
    std::vector<KDTreeANN::IndexType> indices(2);
    std::vector<double> distances(2);

    EXPECT_NO_THROW(index.knn(query, 2, indices, distances));
}