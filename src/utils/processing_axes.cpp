#include "utils/processing_axes.h"
#include <algorithm>

namespace factorlib {

ProcessingAxes::ProcessingAxes()
    : _time_freqs{1} {} // 默认只保留基础频率 1ms，确保无配置时也可运行

ProcessingAxes& ProcessingAxes::instance() {
    static ProcessingAxes inst;
    return inst;
}

void ProcessingAxes::normalize_locked(std::vector<int64_t>& freqs) const {
    // 1) 删除非正数，用户可能在配置里误写 0/-1
    freqs.erase(std::remove_if(freqs.begin(), freqs.end(),
                               [](int64_t f) { return f <= 0; }),
                freqs.end());
    // 2) 如果全被删空，则兜底为 {1}
    if (freqs.empty()) {
        freqs.push_back(1);
    }
    // 3) 最终保持递增且不重复，方便后续与缓存比对
    std::sort(freqs.begin(), freqs.end());
    freqs.erase(std::unique(freqs.begin(), freqs.end()), freqs.end());
}

void ProcessingAxes::set_time_frequencies(std::vector<int64_t> freqs) {
    std::lock_guard<std::mutex> lk(_mutex);
    normalize_locked(freqs);
    _time_freqs = std::move(freqs);
}

const std::vector<int64_t>& ProcessingAxes::time_frequencies() const {
    return _time_freqs;
}

void set_time_frequencies(const std::vector<int64_t>& freqs) {
    ProcessingAxes::instance().set_time_frequencies(freqs);
}

const std::vector<int64_t>& get_time_frequencies() {
    return ProcessingAxes::instance().time_frequencies();
}

} // namespace factorlib
