# ============================================================
# tools/sparring_yellow — 一键重建「陪练黄队」（官方 demo 源码 + 我们的禁区纪律补丁）
#
# ⚠️ 红线：官方 demo 源码不进仓库。本工程在**构建时**从平台自带目录拷贝源码到 build/，
#    再应用 tools/py/patch_demo_yellow.py 打补丁，然后编译。仓库里没有官方代码。
#
# 用法（VS2022 生成器 + Win32，必须 32 位）：
#   cmake -S tools/sparring_yellow -B build/sparring_yellow -G "Visual Studio 17 2022" -A Win32
#   cmake --build build/sparring_yellow --config Release
#   build/sparring_yellow/Release/test_discipline.exe
#   copy /Y build/sparring_yellow/Release/Strategy4Yellow.dll C:\Strategy\   (平台需先关闭)
# ============================================================
cmake_minimum_required(VERSION 3.15)
project(sparring_yellow CXX)

# 官方 demo 源码目录（平台自带；不在本仓库里）
set(DEMO_SRC_DIR "C:/Strategy/src/Strategy4Yellow" CACHE PATH "官方 demo 源码目录（平台自带）")
set(PATCH_PY "${CMAKE_CURRENT_SOURCE_DIR}/../py/patch_demo_yellow.py")
set(WORK "${CMAKE_BINARY_DIR}/demo_src")

if(NOT EXISTS "${DEMO_SRC_DIR}/Strategy4Yellow.cpp")
  message(FATAL_ERROR "找不到官方 demo 源码：${DEMO_SRC_DIR}/Strategy4Yellow.cpp（用 -DDEMO_SRC_DIR=... 指定）")
endif()
if(NOT EXISTS "${PATCH_PY}")
  message(FATAL_ERROR "找不到补丁脚本：${PATCH_PY}")
endif()

# 每次构建都：清工作目录 → 拷官方源码 → 打我们的补丁（脚本自带锚点校验，已是补丁版会报错退出）
# 注意：这些源文件是"构建时才生成"的 → 必须声明 OUTPUT + GENERATED，否则 CMake 配置阶段会报找不到源文件
set(GEN_SRCS "${WORK}/Strategy4Yellow.cpp" "${WORK}/dllmain.cpp" "${WORK}/stdafx.cpp")
add_custom_command(
  OUTPUT ${GEN_SRCS}
  COMMAND "${CMAKE_COMMAND}" -E remove_directory "${WORK}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${WORK}"
  COMMAND "${CMAKE_COMMAND}" -E copy_directory "${DEMO_SRC_DIR}" "${WORK}"
  COMMAND python "${PATCH_PY}" --src "${WORK}/Strategy4Yellow.cpp"
  COMMENT "拷贝官方 demo 源码并应用 tools/py/patch_demo_yellow.py"
  VERBATIM)
set_source_files_properties(${GEN_SRCS} PROPERTIES GENERATED TRUE)

add_library(Strategy4Yellow SHARED ${GEN_SRCS})
target_include_directories(Strategy4Yellow PRIVATE "${WORK}")
target_compile_definitions(Strategy4Yellow PRIVATE STRATEGY4YELLOW_EXPORTS)
set_target_properties(Strategy4Yellow PROPERTIES
  PREFIX ""
  # 必须 /MT：平台是裸 32 位进程，官方 demo 也是 /MT（导入表只有 KERNEL32）
  MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

add_executable(test_discipline
  "${WORK}/Strategy4Yellow.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/test_discipline.cpp")
target_include_directories(test_discipline PRIVATE "${WORK}")
target_compile_definitions(test_discipline PRIVATE STRATEGY4YELLOW_EXPORTS)
set_property(TARGET test_discipline PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
# 测试文件本身是 UTF-8 无 BOM，只对它开 /utf-8（官方源码是 GBK，不能一起开）
set_source_files_properties(test_discipline.cpp PROPERTIES COMPILE_OPTIONS "/utf-8")
