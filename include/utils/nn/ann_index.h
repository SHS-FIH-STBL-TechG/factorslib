#pragma once
/**
 * @file ann_index.h
 * @brief 使用 nanoflann 的 KD-Tree 做近邻搜索（无回退实现，强依赖 nanoflann）。
 *
 * 说明：
 *   - 接口为简单封装：build(pts) + knn(q, k, idx, dist2)；
 *   - 典型复杂度：构建 O(n log n)，查询 O(log n)（kd-tree 近似）。
 */

#include <vector>
#include <memory>
#include <cstddef>
#include <nanoflann.hpp>

namespace factorlib { namespace nn {

/**
 * @brief nanoflann 需要的“点云适配器”
 *        这里把 std::vector<std::vector<double>> 适配成点云接口。
 */
struct PointCloudAdaptor {
    const std::vector<std::vector<double>>* pts{nullptr};

    inline std::size_t kdtree_get_point_count() const {
        return pts ? pts->size() : 0;
    }

    inline double kdtree_get_pt(const std::size_t idx, const std::size_t dim) const {
        return (*pts)[idx][dim];
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};

/**
 * @brief 基于 nanoflann::KDTreeSingleIndexAdaptor 的简单封装
 */
class KDTreeANN {
public:
    using IndexType = std::size_t;
    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, PointCloudAdaptor>,
        PointCloudAdaptor,
        -1  // 维度运行时决定
    >;

    KDTreeANN() = default;

    /// 构建索引；pts 的生命周期需要覆盖 KDTreeANN 的使用期
    void build(const std::vector<std::vector<double>>& pts) {
        cloud_.pts = &pts;
        const std::size_t dim = pts.empty() ? 0 : pts[0].size();
        index_.reset(new KDTree(dim, cloud_, nanoflann::KDTreeSingleIndexAdaptorParams(10 /*leaf max*/)));
        index_->buildIndex();
    }

    /// k 近邻查询：返回索引与 L2 距离平方
    void knn(const std::vector<double>& q,
             std::size_t k,
             std::vector<IndexType>& out_idx,
             std::vector<double>& out_dist2) const
    {
        out_idx.assign(k, IndexType());
        out_dist2.assign(k, 0.0);

        nanoflann::KNNResultSet<double> rs(k);
        rs.init(out_idx.data(), out_dist2.data());
        index_->findNeighbors(rs, q.data());
    }

private:
    PointCloudAdaptor       cloud_;
    std::unique_ptr<KDTree> index_;
};

}} // namespace factorlib::nn
