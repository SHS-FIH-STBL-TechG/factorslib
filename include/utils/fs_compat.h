// include/utils/fs_compat.h
#pragma once

// 提供跨编译器的 std::filesystem 兼容层：
// GCC 7 仍然只支持 experimental::filesystem，因此这里检测可用的头文件，
// 并统一暴露 factorlib::fs 命名空间别名供项目使用。

#if defined(__has_include)
#  if __has_include(<filesystem>)
#    include <filesystem>
#    define FACTORLIB_USE_EXPERIMENTAL_FS 0
#  elif __has_include(<experimental/filesystem>)
#    include <experimental/filesystem>
#    define FACTORLIB_USE_EXPERIMENTAL_FS 1
#  else
#    error "Neither <filesystem> nor <experimental/filesystem> is available on this toolchain."
#  endif
#else
#  include <experimental/filesystem>
#  define FACTORLIB_USE_EXPERIMENTAL_FS 1
#endif

namespace factorlib {
#if FACTORLIB_USE_EXPERIMENTAL_FS
namespace fs = std::experimental::filesystem;
#else
namespace fs = std::filesystem;
#endif
}  // namespace factorlib

#undef FACTORLIB_USE_EXPERIMENTAL_FS
