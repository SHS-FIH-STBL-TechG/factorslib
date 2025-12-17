#pragma once

#include "core/databus.h"
#include "core/ifactor.h"
#include "core/types.h"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace factorlib {

struct WlsDriftZConfig {
    int window_size = 60;         // windowN in yinzi1
    int min_obs = 20;
    double sigma2_floor = 1e-8;
    double r_cap = 0.20;
};

class WlsDriftZFactor : public BaseFactor {
public:
    using Code = std::string;

    explicit WlsDriftZFactor(const std::vector<Code>& codes,
                             const WlsDriftZConfig& cfg = WlsDriftZConfig{});

    static void register_topics(std::size_t capacity);

    void on_bar(const Bar& b) override;

    bool force_flush(const std::string& /*code*/) override { return true; }

private:
    struct CodeState {
        struct Sample {
            double w = 0.0;
            double wr = 0.0;
        };

        explicit CodeState(const WlsDriftZConfig& cfg);

        bool has_last_close = false;
        double last_close = 0.0;

        std::uint64_t bad_close_count = 0;
        std::uint64_t bad_hilo_count = 0;
        std::uint64_t bad_weight_count = 0;

        std::deque<Sample> window;
        double sum_w = 0.0;
        double sum_wr = 0.0;

        bool push_bar(const Bar& b, const WlsDriftZConfig& cfg);
        bool ready(const WlsDriftZConfig& cfg) const {
            return static_cast<int>(window.size()) >= cfg.min_obs;
        }
        double value() const;
    };

    WlsDriftZConfig _cfg;
    std::unordered_set<Code> _codes_filter;
    std::unordered_map<Code, CodeState> _states;

    bool accept_code(const Code& code) const;
};

} // namespace factorlib
