#pragma once
// tunable.hpp —— 参数注册表：把策略里"人肉调出来的数值常量"变成运行时可注入的旋钮。
//
// 生活化比喻：原来是焊死在车上的螺母，现在换成可调旋钮——
//   但**默认拧到原来的位置**，所以平台跑的 DLL 行为与改动前逐位一致。
//
// 设计要点（为什么这么做）：
//  1. 头文件内联实现 ⇒ 不用改 CMakeLists（DLL / offline_test / sim_bench 都能直接用）；
//  2. 默认值 = 源码里写的值，注册表只在启动时被"登记"，**不会自己改值**；
//  3. 只有离线工具（sim_bench / tune_es）才调用 apply_param_file()，
//     平台加载的 DLL 里没有任何文件读取 ⇒ 不违反"裸 DLL、无外部依赖"的红线；
//  4. 用静态变量 + 存指针的方式注入：使用点零开销（不是每次读 map）。
//
// 用法（在 .cpp 文件顶层）：
//     #define TUNABLE_PREFIX "shoot."
//     #include "simuro5/tunable.hpp"
//     TUNABLE(kMaxShotNormal, 70.0);   // 原来是 constexpr double kMaxShotNormal = 70.0;
//
// 注意：**只能用在文件顶层**（namespace 作用域）。函数内部的 static 局部变量
// 要到第一次调用才初始化，会覆盖掉先注入的值，所以那种常量不要转。

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace simuro5 {

struct ParamEntry {
    std::string name;
    double* ptr;
    double def;      // 编译时的默认值（用于 reset 和 dump）
};

inline std::vector<ParamEntry>& param_registry() {
    static std::vector<ParamEntry> v;
    return v;
}

struct ParamReg {
    ParamReg(const char* name, double* p) {
        param_registry().push_back(ParamEntry{std::string(name), p, *p});
    }
};

// 按名字设值；找不到返回 false
inline bool set_param(const char* name, double value) {
    for (auto& e : param_registry()) {
        if (e.name == name) { *e.ptr = value; return true; }
    }
    return false;
}

// 恢复全部默认值
inline void reset_params() {
    for (auto& e : param_registry()) *e.ptr = e.def;
}

// 从文本文件读参数：每行 "name value"，# 或 // 开头为注释，空行忽略。
// 返回成功设置的条数；unknown 非空时收集未识别的名字。
inline int apply_param_file(const char* path, std::vector<std::string>* unknown = nullptr) {
    std::FILE* f = std::fopen(path, "r");
    if (!f) return -1;
    char buf[512];
    int ok = 0;
    while (std::fgets(buf, sizeof(buf), f)) {
        char* p = buf;
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == '\0' || *p == '\n' || *p == '\r' || *p == '#' || (p[0] == '/' && p[1] == '/')) continue;
        char name[256] = {0};
        double val = 0.0;
        if (std::sscanf(p, "%255s %lf", name, &val) != 2) continue;
        if (set_param(name, val)) ++ok;
        else if (unknown) unknown->push_back(name);
    }
    std::fclose(f);
    return ok;
}

// 导出全部参数：name value  # default=...（可直接当参数文件再喂回去）
inline int dump_params(const char* path) {
    std::FILE* f = std::fopen(path, "w");
    if (!f) return -1;
    for (auto& e : param_registry())
        std::fprintf(f, "%-28s %-12.6g  # default=%.6g\n", e.name.c_str(), *e.ptr, e.def);
    std::fclose(f);
    return (int)param_registry().size();
}

}  // namespace simuro5

#ifndef TUNABLE_PREFIX
#define TUNABLE_PREFIX ""
#endif

// 注意：static ⇒ 内部链接，不会与其他 .cpp 里的同名常量冲突；注册在同一 TU 内完成。
#define TUNABLE(name, def)                                                     \
    static double name = (def);                                                \
    static ::simuro5::ParamReg _tune_reg_##name(TUNABLE_PREFIX #name, &name)
