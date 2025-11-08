# 日志接入（spdlog 封装）

在程序入口加一行初始化（可选，默认即可工作）：
```cpp
#include "utils/log.h"
int main(){
    factorlib::log::init_logging(); // 可选：自定义日志文件/格式/级别
    LOG_INFO("factorlib started");
    ...
}
```

使用：
```cpp
LOG_TRACE("hello");
LOG_INFO("price=%f ts=%lld", price, (long long)ts_ms);
LOG_WARN("warn: code=%s", code.c_str());
LOG_ERROR("oops: err=%d", err);
```

如需 vendor 第三方：把 `spdlog/include` 目录放到 `third_party/spdlog/include/` 即可。
