#pragma once

#include "core/databus.h"
#include "core/ifactor.h"
#include "core/types.h"
#include "math/rolling_hmm_forward.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace factorlib {

struct Hmm2LogOddsConfig {
    int window_size = 60;

    double mu_up = 0.006;
    double sigma_up = 0.010;
    double mu_down = -0.006;
    double sigma_down = 0.012;

    double trans_uu = 0.90;
    double trans_dd = 0.90;
    double pi_up = 0.5;

    double r_cap = 0.20;

    math::HMM2GaussianParams to_math_params() const {
        return math::HMM2GaussianParams{
            mu_up, sigma_up,
            mu_down, sigma_down,
            trans_uu, trans_dd,
            pi_up
        };
    }
};

class Hmm2LogOddsFactor : public BaseFactor {
public:
    using Code = std::string;

    explicit Hmm2LogOddsFactor(const std::vector<Code>& codes,
                               const Hmm2LogOddsConfig& cfg = Hmm2LogOddsConfig{});

    static void register_topics(std::size_t capacity);

    void on_bar(const Bar& b) override;

    bool force_flush(const std::string& /*code*/) override { return true; }

private:
    struct CodeState {
        explicit CodeState(const Hmm2LogOddsConfig& cfg);

        bool has_last_close = false;
        double last_close = 0.0;
        std::uint64_t bad_close_count = 0;
        math::RollingHMMForward<double> hmm;

        void push_bar(const Bar& b, const Hmm2LogOddsConfig& cfg);
        bool ready() const { return hmm.ready(); }
        double log_odds() const;
    };

    Hmm2LogOddsConfig _cfg;
    std::unordered_set<Code> _codes_filter;
    std::unordered_map<Code, CodeState> _states;

    bool accept_code(const Code& code) const;
};

} // namespace factorlib
