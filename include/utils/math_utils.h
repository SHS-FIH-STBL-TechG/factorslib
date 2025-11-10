// include/utils/math_utils.h
#pragma once
#include <vector>
#include <cmath>
#include <algorithm>

namespace factorlib {

    /**
     * @brief 数学计算工具类
     */
    class MathUtils {
    public:
        /**
         * @brief 计算序列的均值
         */
        static double mean(const std::vector<double>& data);

        /**
         * @brief 计算序列的标准差
         */
        static double stddev(const std::vector<double>& data);

        /**
         * @brief 计算序列的分位数
         * @param data 输入数据
         * @param percentile 分位数 (0-1)
         */
        static double quantile(const std::vector<double>& data, double percentile);

        /**
         * @brief 计算两个序列的相关系数
         */
        static double correlation(const std::vector<double>& x, const std::vector<double>& y);

        /**
         * @brief 正态分布逆CDF（使用Wichura算法近似）
         */
        static double normal_quantile(double p);

        /**
         * @brief 经验逆CDF计算
         */
        static double empirical_inverse_cdf(const std::vector<double>& data, double probability);

        /**
         * @brief 计算中位秩
         */
        static double median_rank(const std::vector<double>& data, double value);

        /**
         * @brief 安全的对数计算（处理非正数）
         */
        static double safe_log(double x);
    };

} // namespace factorlib