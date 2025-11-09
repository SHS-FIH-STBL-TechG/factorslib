
# factorlib (v2)

- 基础/复杂因子分层；
- 模板化 DataBus（层级 topic，环形历史 + timestamp 精确匹配）；
- 0109 因子“翻译”到基础因子层。

## 目录
```
include/utils/
  log.h        # 日志宏
  utils.h      # 数据类型 + Nms 桶
  databus.h    # 模板化 DataBus（register_topic/publish/get*/subscribe）

src/basic_factors/
  tick_trans_orders.h/.cpp

src/advanced_factors/
  (示例留空)

src/utils/
  log.cpp
  utils.cpp

tests/
  test_bus_and_factor.cpp
  utils/data_gen.h

third_party/googletest/   # 放 gtest/gmock
docs/CONFIRMED.md         # 确认记录
```

## 构建（两种方式）
### A 静态库
```bash
g++ -std=c++17 -O2 -Iinclude -Isrc -Ithird_party/googletest/include   src/utils/log.cpp src/utils/utils.cpp   src/basic_factors/tick_trans_orders.cpp   tests/test_bus_and_factor.cpp   -Lthird_party/googletest/lib -lgtest -lgtest_main -lpthread -o tests/run_tests
./tests/run_tests
```

### B 源码方式
```bash
GTEST_SRC=third_party/googletest
g++ -std=c++17 -O2 -Iinclude -Isrc -I$GTEST_SRC/googletest/include   src/utils/log.cpp src/utils/utils.cpp   src/basic_factors/tick_trans_orders.cpp   tests/test_bus_and_factor.cpp   $GTEST_SRC/googletest/src/gtest-all.cc   -lpthread -o tests/run_tests
./tests/run_tests
```

## 使用示例（模板化）
```cpp
auto& bus = DataBus::instance();
bus.register_topic<double>("zyd/amount", 120);
bus.publish<double>("zyd/amount", "000001.SZ", /*ts_ms*/ 1730964600000, 12345.6);

double amt=0; int64_t ts=0;
bool ok = bus.get_latest<double>("zyd/amount", "000001.SZ", amt, &ts); // 精确类型读取
```



## 使用 CMake 构建（推荐）
```bash
mkdir -p build && cd build
cmake ..
cmake --build . -j
ctest --output-on-failure
```
