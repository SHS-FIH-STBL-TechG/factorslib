// tests//gtest_printer_zh.h
#pragma once
#include <gtest/gtest.h>
#include <iostream>

#if defined(_WIN32)
  #include <windows.h>
#endif

namespace gtest_cn {

// ANSI 颜色
inline const char* Reset()  { return "\x1b[0m";  }
inline const char* Bold()   { return "\x1b[1m";  }
inline const char* Green()  { return "\x1b[32m"; }
inline const char* Red()    { return "\x1b[31m";  }
inline const char* Yellow() { return "\x1b[33m"; }

// Windows 控制台：启用 ANSI + UTF-8
inline void EnableAnsiColorOnWindows() {
#if defined(_WIN32)
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    if (h != INVALID_HANDLE_VALUE) {
        DWORD mode = 0;
        if (GetConsoleMode(h, &mode)) {
            mode |= 0x0004; // ENABLE_VIRTUAL_TERMINAL_PROCESSING
            SetConsoleMode(h, mode);
        }
    }
    SetConsoleOutputCP(CP_UTF8);
#endif
}

/**
 * 中文打印器：保持 gtest 标签与对齐，文本中文化 + 颜色。
 * 注意：必须完整覆盖 TestEventListener 的所有虚方法，否则会变成“抽象类”导致编译失败。
 */
class CnDefaultStylePrinter : public ::testing::TestEventListener {
public:
    explicit CnDefaultStylePrinter(::testing::TestEventListener* fallback)
        : fallback_(fallback) {}
    ~CnDefaultStylePrinter() override { delete fallback_; }

    // ========== 顶层 ==========
    void OnTestProgramStart(const ::testing::UnitTest& ut) override {
        EnableAnsiColorOnWindows();
        ::testing::GTEST_FLAG(color) = "yes";
        std::cout << Bold() << "[==========] " << Reset()
                  << "将运行 " << ut.test_to_run_count()
                  << " 个测试，来自 " << ut.test_suite_to_run_count()
                  << " 个测试套件。\n";
    }
    void OnTestProgramEnd(const ::testing::UnitTest& /*ut*/) override {
        // 保持沉默（gtest 默认也基本在 IterationEnd 汇总）
    }

    // ========== 迭代 ==========
    void OnTestIterationStart(const ::testing::UnitTest& /*ut*/, int /*iteration*/) override {
        // 保持默认结构；不额外输出
    }
    void OnTestIterationEnd(const ::testing::UnitTest& ut, int /*iteration*/) override {
        std::cout << Bold() << "[==========] " << Reset()
                  << ut.test_to_run_count() << " 个测试来自 "
                  << ut.test_suite_to_run_count() << " 个测试套件已运行完毕"
                  << "（总耗时 " << ut.elapsed_time() << " ms）。\n";
        if (ut.failed_test_count() == 0) {
            std::cout << Green() << "[  PASSED  ] " << Reset()
                      << ut.successful_test_count() << " 个测试全部通过。\n";
        } else {
            std::cout << Red() << "[  FAILED  ] " << Reset()
                      << ut.failed_test_count() << " 个测试失败；通过 "
                      << ut.successful_test_count() << "，跳过 "
                      << ut.skipped_test_count() << "。\n";
            PrintFailedTests(ut);
        }
    }

    // ========== 全局环境 ==========
    void OnEnvironmentsSetUpStart(const ::testing::UnitTest& /*ut*/) override {
        std::cout << Bold() << "[----------] " << Reset()
                  << "全局测试环境初始化。\n";
    }
    void OnEnvironmentsSetUpEnd(const ::testing::UnitTest& /*ut*/) override {
        // 保持沉默
    }
    void OnEnvironmentsTearDownStart(const ::testing::UnitTest& /*ut*/) override {
        std::cout << Bold() << "[----------] " << Reset()
                  << "全局测试环境清理。\n";
    }
    void OnEnvironmentsTearDownEnd(const ::testing::UnitTest& /*ut*/) override {
        // 保持沉默
    }

    // ========== 套件 ==========
    void OnTestSuiteStart(const ::testing::TestSuite& ts) override {
        std::cout << Bold() << "[----------] " << Reset()
                  << "开始 " << ts.name()
                  << "，包含 " << ts.test_to_run_count()
                  << " 个测试。\n";
    }
    void OnTestSuiteEnd(const ::testing::TestSuite& ts) override {
        std::cout << Bold() << "[----------] " << Reset()
                  << "结束 " << ts.name()
                  << " （耗时 " << ts.elapsed_time() << " ms）\n";
    }

    // ========== 单测 ==========
    void OnTestStart(const ::testing::TestInfo& ti) override {
        std::cout << Bold() << "[ RUN      ] " << Reset()
                  << ti.test_suite_name() << "." << ti.name() << "\n";
    }
    void OnTestPartResult(const ::testing::TestPartResult& r) override {
        if (r.type() == ::testing::TestPartResult::kSuccess) return;
        std::cout << Red() << "  断言失败: " << Reset()
                  << r.file_name() << ":" << r.line_number() << "\n"
                  << "    " << r.summary() << "\n";
    }
    void OnTestEnd(const ::testing::TestInfo& ti) override {
        const auto ms = ti.result()->elapsed_time();
        if (ti.result()->Passed()) {
            std::cout << Green() << "[       OK ] " << Reset()
                      << ti.test_suite_name() << "." << ti.name()
                      << " （耗时 " << ms << " ms）\n";
        } else {
            std::cout << Red() << "[  FAILED  ] " << Reset()
                      << ti.test_suite_name() << "." << ti.name()
                      << " （耗时 " << ms << " ms）\n";
        }
    }

private:
    static const ::testing::TestPartResult* FirstFailurePart(const ::testing::TestResult& result) {
        for (int i = 0; i < result.total_part_count(); ++i) {
            const auto& part = result.GetTestPartResult(i);
            if (part.failed()) {
                return &part;
            }
        }
        return nullptr;
    }

    void PrintFailedTests(const ::testing::UnitTest& ut) const {
        int index = 1;
        for (int i = 0; i < ut.total_test_suite_count(); ++i) {
            const auto* ts = ut.GetTestSuite(i);
            if (!ts || !ts->should_run()) continue;
            for (int j = 0; j < ts->total_test_count(); ++j) {
                const auto* ti = ts->GetTestInfo(j);
                if (!ti || !ti->should_run()) continue;
                if (!ti->result()->Failed()) continue;

                std::cout << "      " << index++ << ". "
                          << ts->name() << "." << ti->name();
                if (const auto* failure = FirstFailurePart(*ti->result())) {
                    if (failure->file_name()) {
                        std::cout << " (" << failure->file_name()
                                  << ":" << failure->line_number() << ")";
                    }
                    std::cout << "\n";
                    if (failure->summary() && *failure->summary()) {
                        std::cout << "         ↳ " << failure->summary() << "\n";
                    }
                } else {
                    std::cout << "\n";
                }
            }
        }
    }

    ::testing::TestEventListener* fallback_; // 不使用默认打印器，仅为占位释放
};

// 安装：替换默认英文打印器
inline void InstallCnDefaultStyle() {
    auto& listeners = ::testing::UnitTest::GetInstance()->listeners();
    auto* def = listeners.Release(listeners.default_result_printer());
    listeners.Append(new CnDefaultStylePrinter(def));
}

} // namespace gtest_cn
