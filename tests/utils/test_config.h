#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include <cstdint>

#include "core/types.h"

namespace testcfg {

    struct Cfg {
        std::unordered_map<std::string, std::string> kv; // key: "section.key"
        std::string get(const std::string& k, const std::string& defv) const;
        int         geti(const std::string& k, int defv) const;
        int64_t     geti64(const std::string& k, int64_t defv) const;
        double      getd(const std::string& k, double defv) const;
        bool        getb(const std::string& k, bool defv) const;
    };

    // 从环境变量 TEST_CFG 读取路径；否则默认 "tests/utis/test_config.ini"
    const Cfg& global();

    // 简单 CSV 读取（若你要在测试中用真实 CSV）
    std::vector<std::vector<std::string>> read_csv(const std::string& path, char sep=',');

    // 新增：基于 ini 配置和 read_csv 的封装
    std::vector<factorlib::Bar> read_bars_from_cfg();
    std::vector<factorlib::Transaction> read_transactions_from_cfg();
    std::vector<factorlib::QuoteDepth> read_quotes_from_cfg();

} // namespace testcfg
