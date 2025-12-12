#pragma once

#include "core/ifactor.h"

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace factorlib::tools {

// 单独维护“因子绑定/默认参数”，以后扩展因子或改默认值只需改这个文件。

struct FactorBinding {
    std::string key;
    std::string input_topic;
    std::function<void(std::size_t)> register_topics;
    std::function<std::shared_ptr<factorlib::BaseFactor>(const std::vector<std::string>&)> create;
};

const std::vector<FactorBinding>& GetFactorBindings();
const FactorBinding* ResolveFactorBinding(const std::string& key);

// 默认目录
std::string DefaultDataDir();
std::string DefaultOutputDir();

} // namespace factorlib::tools

