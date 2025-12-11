#include "factor_tool_config.h"

#include <cstdlib>

namespace factorlib::tools {
namespace {

std::size_t ParseSizeEnv(const char* name, std::size_t fallback) {
    if (!name) return fallback;
    if (const char* env = std::getenv(name)) {
        char* end = nullptr;
        unsigned long long value = std::strtoull(env, &end, 10);
        if (end != env) {
            return static_cast<std::size_t>(value);
        }
    }
    return fallback;
}

} // namespace

const FactorToolConfig& GetFactorToolConfig() {
    static const FactorToolConfig kConfig = [] {
        FactorToolConfig cfg{};
        cfg.default_lookback_days =
            ParseSizeEnv("FACTOR_TOOL_LOOKBACK_DAYS", cfg.default_lookback_days);
        return cfg;
    }();
    return kConfig;
}

} // namespace factorlib::tools
