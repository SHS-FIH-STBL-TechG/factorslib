#pragma once
#include <string>
#include <unordered_map>
#include <cstdint>

namespace factorlib::config {

    class RuntimeConfig {
    public:
        // 单例：从环境变量 FACTOR_CFG 读取 INI 路径；若未设置则默认 "config/factor_config.ini"
        static const RuntimeConfig& instance();

        // 读取字符串或带默认值的类型化访问
        std::string get(const std::string& key, const std::string& def) const;
        int     geti (const std::string& key, int def) const;
        int64_t geti64(const std::string& key, int64_t def) const;
        double  getd (const std::string& key, double def) const;
        bool    getb (const std::string& key, bool def) const;

    private:
        RuntimeConfig(); // 私有：通过 instance() 获取
        std::unordered_map<std::string, std::string> kv_; // "section.key" -> value
    };

    // 便捷别名：RC() 即 RuntimeConfig::instance()
    inline const RuntimeConfig& RC() { return RuntimeConfig::instance(); }

} // namespace factorlib::config
