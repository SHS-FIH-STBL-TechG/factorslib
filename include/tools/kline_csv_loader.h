#pragma once

#include "core/types.h"

#include <optional>
#include <string>
#include <vector>

namespace factorlib::tools {

/**
 * @brief K线数据序列，包含一支股票的完整K线数据
 */
struct KlineCsvSeries {
    std::string table_name;  ///< 表名/数据源名称
    std::string code;        ///< 股票代码（如 000001.SZ）
    std::vector<factorlib::Bar> bars;  ///< K线数据列表（按时间升序）
};

/**
 * @brief 从 CSV 文件加载 K线数据
 * 
 * 功能：
 * - 从指定目录加载 CSV 格式的 K线数据
 * - 支持按股票代码和时间范围过滤
 * - 自动解析不同格式的日期、价格、成交量等字段
 * 
 * CSV 文件格式要求：
 * - 文件名：{code}.csv（代码可包含 .SZ/.SH 后缀）
 * - 必须包含列：日期、开盘、最高、最低、收盘、成交量、成交额
 * - 支持带表头的 CSV 文件
 * 
 * 示例目录结构：
 * @code
 * data_dir/
 *   ├── 000001.SZ.csv
 *   ├── 600000.SH.csv
 *   └── ...
 * @endcode
 */
class KlineCsvLoader {
public:
    /**
     * @brief 构造函数
     * @param data_dir K线 CSV 文件所在目录
     */
    explicit KlineCsvLoader(std::string data_dir);

    /**
     * @brief 加载 K线数据
     * @param codes_filter 股票代码过滤器（空则加载所有）
     * @param start_time_ms 开始时间（毫秒时间戳，nullopt 表示不限制）
     * @param end_time_ms 结束时间（毫秒时间戳，nullopt 表示不限制）
     * @return 加载的 K线序列列表，每个序列对应一支股票
     * 
     * 功能：
     * 1. 扫描目录下所有 .csv 文件
     * 2. 根据 codes_filter 过滤股票
     * 3. 解析 CSV 内容并转换为 Bar 结构
     * 4. 按时间范围过滤
     * 5. 按代码排序返回
     */
    std::vector<KlineCsvSeries> load(const std::vector<std::string>& codes_filter,
                                     std::optional<int64_t> start_time_ms,
                                     std::optional<int64_t> end_time_ms) const;

private:
    std::string _root_dir;  ///< K线数据根目录

    /**
     * @brief 解析 CSV 行
     * @param line CSV 行字符串
     * @return 字段列表
     */
    static std::vector<std::string> parse_csv_line(const std::string& line);
    
    /**
     * @brief 去除字符串首尾空白
     * @param value 输入字符串
     * @return 去除空白后的字符串
     */
    static std::string trim(const std::string& value);
    
    /**
     * @brief 将日期字符串转换为毫秒时间戳
     * @param date_token 日期字符串（如 "2023-01-01" 或 "20230101"）
     * @return 毫秒时间戳，解析失败则返回 nullopt
     */
    static std::optional<int64_t> parse_date_to_ms(const std::string& date_token);
    
    /**
     * @brief 解析数字字符串
     * @param token 数字字符串
     * @return 浮点数，解析失败则返回 nullopt
     */
    static std::optional<double> parse_number(const std::string& token);
    
    /**
     * @brief 从文件名提取股票代码
     * @param file_name 文件名（如 "000001.SZ.csv" 或 "600000.csv"）
     * @return 股票代码（如 "000001.SZ"）
     */
    static std::string extract_code(const std::string& file_name);
    
    /**
     * @brief 检查代码是否在过滤器列表中
     * @param filters 过滤器列表（空表示接受所有）
     * @param code 股票代码
     * @return true 表示接受该代码
     */
    static bool accept_code(const std::vector<std::string>& filters, const std::string& code);
};

} // namespace factorlib::tools

