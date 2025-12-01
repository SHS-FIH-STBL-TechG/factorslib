#include "factors/stat/volume_price_structure_factor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <Eigen/Dense>

#include "core/databus.h"
#include "utils/log.h"

namespace factorlib {

namespace {
/**
 * @brief topic 名称：
 *   - TOP_RET_LOGVOL_CORR     : 16. 收益对数成交量相关；
 *   - TOP_PV_PCA1_RET_LOADING : 20. 量价第一主成分收益载荷。
 */
constexpr const char* TOP_RET_LOGVOL_CORR     = "kline/ret_logvol_corr";        // 16
constexpr const char* TOP_PV_PCA1_RET_LOADING = "kline/pv_pca1_ret_loading";    // 20

/**
 * @brief 对成交量做“安全取对数”：若 volume <= 0，则返回 NaN。
 */
double safe_log_volume(double volume) {
    if (!(volume > 0.0)) return std::numeric_limits<double>::quiet_NaN();
    return std::log(volume);
}

} // anonymous namespace

//====================== CodeState 实现 ======================//

VolumePriceStructureFactor::CodeState::CodeState(int window_size)
    : window(window_size),
      has_last_close(false),
      last_close(0.0),
      sum_ret(0.0),
      sum_log_volume(0.0),
      sum_amp(0.0),
      sum_ret2(0.0),
      sum_log_volume2(0.0),
      sum_amp2(0.0),
      sum_ret_log_volume(0.0),
      sum_ret_amp(0.0),
      sum_log_volume_amp(0.0) {}

/**
 * @brief 推进一根 K 线，更新对数收益 / log 成交量 / 振幅滑窗。
 */
bool VolumePriceStructureFactor::CodeState::push_bar(const Bar& b) {
    if (!(b.close > 0.0) || !(b.volume > 0.0)) {
        // 收盘价或成交量非法时，忽略该样本
        return false;
    }

    const double close = static_cast<double>(b.close);
    if (!has_last_close) {
        last_close = close;
        has_last_close = true;
        return false;
    }

    const double ret = std::log(close / last_close);
    last_close = close;

    const double vol = static_cast<double>(b.volume);
    const double lv  = safe_log_volume(vol);

    const double high = static_cast<double>(b.high);
    const double low  = static_cast<double>(b.low);
    double amp = 0.0;
    if (high > 0.0 && low > 0.0) {
        amp = (high - low) / std::max(low, 1e-12);
    }

    Sample sample{ret, lv, amp};
    samples.push_back(sample);
    add_sample(sample);

    if (static_cast<int>(samples.size()) > window) {
        remove_sample(samples.front());
        samples.pop_front();
    }

    return ready();
}

void VolumePriceStructureFactor::CodeState::add_sample(const Sample& s) {
    sum_ret += s.ret;
    sum_log_volume += s.log_volume;
    sum_amp += s.amplitude;

    sum_ret2 += s.ret * s.ret;
    sum_log_volume2 += s.log_volume * s.log_volume;
    sum_amp2 += s.amplitude * s.amplitude;

    sum_ret_log_volume += s.ret * s.log_volume;
    sum_ret_amp += s.ret * s.amplitude;
    sum_log_volume_amp += s.log_volume * s.amplitude;
}

void VolumePriceStructureFactor::CodeState::remove_sample(const Sample& s) {
    sum_ret -= s.ret;
    sum_log_volume -= s.log_volume;
    sum_amp -= s.amplitude;

    sum_ret2 -= s.ret * s.ret;
    sum_log_volume2 -= s.log_volume * s.log_volume;
    sum_amp2 -= s.amplitude * s.amplitude;

    sum_ret_log_volume -= s.ret * s.log_volume;
    sum_ret_amp -= s.ret * s.amplitude;
    sum_log_volume_amp -= s.log_volume * s.amplitude;
}

double VolumePriceStructureFactor::CodeState::ret_logvol_corr() const {
    const std::size_t n = samples.size();
    if (n < 2) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double nn = static_cast<double>(n);
    const double cov = sum_ret_log_volume - (sum_ret * sum_log_volume) / nn;
    const double var_ret = sum_ret2 - (sum_ret * sum_ret) / nn;
    const double var_lv  = sum_log_volume2 - (sum_log_volume * sum_log_volume) / nn;
    const double denom = std::sqrt(std::max(var_ret, 0.0) * std::max(var_lv, 0.0));
    if (!(denom > 0.0)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return cov / denom;
}

bool VolumePriceStructureFactor::CodeState::covariance(double cov[3][3]) const {
    const std::size_t n = samples.size();
    if (n < 2) {
        return false;
    }
    const double nn = static_cast<double>(n);
    const double inv_n = 1.0 / nn;
    const double inv_n1 = 1.0 / static_cast<double>(n - 1);

    const double mean_r  = sum_ret * inv_n;
    const double mean_lv = sum_log_volume * inv_n;
    const double mean_a  = sum_amp * inv_n;

    const double cov_rr = (sum_ret2 - sum_ret * sum_ret * inv_n) * inv_n1;
    const double cov_ll = (sum_log_volume2 - sum_log_volume * sum_log_volume * inv_n) * inv_n1;
    const double cov_aa = (sum_amp2 - sum_amp * sum_amp * inv_n) * inv_n1;
    const double cov_rl = (sum_ret_log_volume - sum_ret * sum_log_volume * inv_n) * inv_n1;
    const double cov_ra = (sum_ret_amp - sum_ret * sum_amp * inv_n) * inv_n1;
    const double cov_la = (sum_log_volume_amp - sum_log_volume * sum_amp * inv_n) * inv_n1;

    cov[0][0] = cov_rr;
    cov[0][1] = cov_rl;
    cov[0][2] = cov_ra;
    cov[1][0] = cov_rl;
    cov[1][1] = cov_ll;
    cov[1][2] = cov_la;
    cov[2][0] = cov_ra;
    cov[2][1] = cov_la;
    cov[2][2] = cov_aa;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!std::isfinite(cov[i][j])) {
                return false;
            }
        }
    }
    return true;
}

//====================== 因子主体实现 ======================//

VolumePriceStructureFactor::VolumePriceStructureFactor(
    const std::vector<Code>& codes,
    const VolumePriceStructureConfig& cfg)
    : _cfg(cfg),
      _codes_filter(codes.begin(), codes.end()) {}

/**
 * @brief 判断是否需要为指定代码计算本因子。
 */
bool VolumePriceStructureFactor::accept_code(const Code& code) const {
    if (_codes_filter.empty()) return true;
    return _codes_filter.find(code) != _codes_filter.end();
}

/**
 * @brief 注册因子输出的 DataBus topic。
 */
void VolumePriceStructureFactor::register_topics(std::size_t capacity) {
    auto& bus = DataBus::instance();
    bus.register_topic<double>(TOP_RET_LOGVOL_CORR, capacity);
    bus.register_topic<double>(TOP_PV_PCA1_RET_LOADING, capacity);
}

/**
 * @brief 使用单根 K 线驱动两个量价结构类因子。
 */
void VolumePriceStructureFactor::on_bar(const Bar& b) {
    if (!accept_code(b.instrument_id)) return;

    // 找到 / 创建该代码对应的滑窗状态
    auto it = _states.find(b.instrument_id);
    if (it == _states.end()) {
        it = _states.emplace(b.instrument_id, CodeState(_cfg.window_size)).first;
    }
    CodeState& st = it->second;

    // 更新滑窗
    if (!st.push_bar(b)) return;

    const int64_t ts = b.data_time_ms;

    //------------------ 16. 收益-对数成交量相关 ------------------//
    const double corr = st.ret_logvol_corr();
    if (std::isfinite(corr)) {
        safe_publish<double>(TOP_RET_LOGVOL_CORR,
                             b.instrument_id,
                             ts,
                             corr);
    }

    //------------------ 20. 量价 PCA 第一主成分收益载荷 ------------------//
    if (st.sample_count() < 3) return;

    double cov_arr[3][3];
    if (!st.covariance(cov_arr)) {
        return;
    }
    Eigen::Matrix3d cov;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            cov(i, j) = cov_arr[i][j];
        }
    }

    // 自伴随特征分解（对称矩阵的特征值/向量）
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    if (es.info() != Eigen::Success) return;

    Eigen::VectorXd evals = es.eigenvalues();
    Eigen::Matrix3d evecs = es.eigenvectors();

    int idx_max = 0;
    if (evals[1] > evals[idx_max]) idx_max = 1;
    if (evals[2] > evals[idx_max]) idx_max = 2;

    const Eigen::Vector3d pc1 = evecs.col(idx_max);
    double loading_ret = pc1[0];  // 收益维度在第一主成分上的载荷

    if (!std::isfinite(loading_ret)) return;

    safe_publish<double>(TOP_PV_PCA1_RET_LOADING,
                         b.instrument_id,
                         ts,
                         loading_ret);
}

} // namespace factorlib
