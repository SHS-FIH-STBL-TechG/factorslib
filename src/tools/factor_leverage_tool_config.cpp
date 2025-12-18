#include "factor_leverage_tool_config.h"

#include "factors/kline/low_freq_return_factor.h"
#include "factors/kline/ar1_return_factor.h"
#include "factors/kline/high_volume_remaining_factor.h"
#include "factors/kline/volume_ar_forecast_factor.h"
#include "factors/kline/volume_price_structure_factor.h"
#include "factors/kline/kalman_forecast_z_factor.h"
#include "factors/kline/wls_drift_z_factor.h"
#include "factors/kline/hmm2_logodds_factor.h"
#include "factors/kline/bayes_drift_logodds_factor.h"
#include "factors/stat/wavelet_trend_energy_factor.h"
#include "factors/Kline_new/haar_wavelet_trend_factor.h"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <utility>

namespace factorlib::tools {

namespace {

template <typename FactorT>
std::shared_ptr<factorlib::BaseFactor> MakeBarOnlyFactor(const std::vector<std::string>& codes) {
    struct Wrapper final : public FactorT {
        using FactorT::FactorT;
        void on_quote(const factorlib::QuoteDepth&) override {}
        void on_tick(const factorlib::CombinedTick&) override {}
    };
    return std::make_shared<Wrapper>(codes);
}

} // namespace

const std::vector<FactorBinding>& GetFactorBindings() {
    static const std::vector<FactorBinding> bindings = {
        {
            "low_freq_return",
            "kline/ret_lowfreq_mu10",
            [](std::size_t cap) { factorlib::LowFreqReturnFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return std::make_shared<factorlib::LowFreqReturnFactor>(codes);
            }
        },
        {
            "ar1_return",
            "kline/ar1_return_coeff",
            [](std::size_t cap) { factorlib::Ar1ReturnFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return MakeBarOnlyFactor<factorlib::Ar1ReturnFactor>(codes);
            }
        },
        {
            "high_volume_remaining",
            "kline/vol_high_remaining",
            [](std::size_t cap) { factorlib::HighVolumeRemainingFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return MakeBarOnlyFactor<factorlib::HighVolumeRemainingFactor>(codes);
            }
        },
        {
            "volume_ar_forecast",
            "kline/vol_ar1_pred_ratio",
            [](std::size_t cap) { factorlib::VolumeArForecastFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return MakeBarOnlyFactor<factorlib::VolumeArForecastFactor>(codes);
            }
        },
        {
            "volume_price_structure_corr",
            "kline/ret_logvol_corr",
            [](std::size_t cap) { factorlib::VolumePriceStructureFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return MakeBarOnlyFactor<factorlib::VolumePriceStructureFactor>(codes);
            }
        },
        {
            "volume_price_structure_pca1",
            "kline/pv_pca1_ret_loading",
            [](std::size_t cap) { factorlib::VolumePriceStructureFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return MakeBarOnlyFactor<factorlib::VolumePriceStructureFactor>(codes);
            }
        },
        {
            "kalman_forecast_z",
            "kline/kalman_forecast_z",
            [](std::size_t cap) { factorlib::KalmanForecastZFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return MakeBarOnlyFactor<factorlib::KalmanForecastZFactor>(codes);
            }
        },
        {
            "wls_drift_z",
            "kline/wls_drift_z",
            [](std::size_t cap) { factorlib::WlsDriftZFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return MakeBarOnlyFactor<factorlib::WlsDriftZFactor>(codes);
            }
        },
        {
            "hmm2_logodds",
            "kline/hmm2_logodds",
            [](std::size_t cap) { factorlib::Hmm2LogOddsFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return MakeBarOnlyFactor<factorlib::Hmm2LogOddsFactor>(codes);
            }
        },
        {
            "bayes_drift_logodds",
            "kline/bayes_drift_logodds",
            [](std::size_t cap) { factorlib::BayesDriftLogOddsFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return MakeBarOnlyFactor<factorlib::BayesDriftLogOddsFactor>(codes);
            }
        },
        {
            "wavelet_trend_energy",
            factorlib::TOP_WAVE_TREND,
            [](std::size_t cap) { factorlib::WaveletTrendEnergyFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return MakeBarOnlyFactor<factorlib::WaveletTrendEnergyFactor>(codes);
            }
        },
        {
            "haar_wavelet_trend",
            "kline_new/haar_trend_energy",
            [](std::size_t cap) { factorlib::HaarWaveletTrendFactor::register_topics(cap); },
            [](const std::vector<std::string>& codes) {
                return MakeBarOnlyFactor<factorlib::HaarWaveletTrendFactor>(codes);
            }
        },
    };
    return bindings;
}

const FactorBinding* ResolveFactorBinding(const std::string& key) {
    for (const auto& b : GetFactorBindings()) {
        if (b.key == key) return &b;
    }
    return nullptr;
}

std::string DefaultDataDir() {
    return "tests/data";
}

std::string DefaultOutputDir() {
    namespace fs = std::filesystem;

    const auto now = std::chrono::system_clock::now();
    const std::time_t tt = std::chrono::system_clock::to_time_t(now);

    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm, "%y%m%d%H%M%S");

    fs::path base = fs::path("output") / ("factor_leverage_" + oss.str());
    std::error_code ec;
    if (fs::exists(base, ec) && !ec) {
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        base = fs::path("output") / ("factor_leverage_" + oss.str() + "_" + std::to_string(ms));
    }
    return base.string();
}

} // namespace factorlib::tools
