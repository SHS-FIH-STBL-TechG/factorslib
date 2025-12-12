#include "factor_leverage_tool_config.h"

#include "factors/kline/low_freq_return_factor.h"
#include "factors/kline/ar1_return_factor.h"
#include "factors/kline/high_volume_remaining_factor.h"
#include "factors/kline/volume_ar_forecast_factor.h"
#include "factors/kline/volume_price_structure_factor.h"

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
    return "output/factor_leverage";
}

} // namespace factorlib::tools

