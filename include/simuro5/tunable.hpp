#pragma once
// tunable.hpp — 数值常量变运行时可注入旋钮（默认值=源码值），只能用在文件顶层；TUNABLE(name,def) 即登记

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace simuro5 {

struct ParamEntry {
    std::string name;
    double* ptr;
    double def;
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

inline bool set_param(const char* name, double value) {
    for (auto& e : param_registry()) {
        if (e.name == name) { *e.ptr = value; return true; }
    }
    return false;
}

inline void reset_params() {
    for (auto& e : param_registry()) *e.ptr = e.def;
}

inline double get_param(const char* name, double fallback = 0.0) {
    for (auto& e : param_registry()) {
        if (e.name == name) return *e.ptr;
    }
    return fallback;
}

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

#define TUNABLE(name, def)                                                     \
    static double name = (def);                                                \
    static ::simuro5::ParamReg _tune_reg_##name(TUNABLE_PREFIX #name, &name)
