# ============================================================
# FIRA 仿真 5v5 策略工程
# 产出: Strategy4Blue.dll / Strategy4Yellow.dll (32 位, 必须!)
# ============================================================
cmake_minimum_required(VERSION 3.16)
project(simuro5_strategy CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- 硬约束: 官方平台 WorldModel.exe 是 32 位(i386), DLL 必须 32 位 ----
if(NOT CMAKE_SIZEOF_VOID_P EQUAL 4)
  message(FATAL_ERROR "官方平台是 32 位(i386)。请用 VS 生成器 + -A Win32 或 32 位工具链构建！")
endif()

# ---- 共享核心源码(编译进两个 DLL) ----
set(CORE_SOURCES
  src/geometry.cpp
  src/world_model.cpp
  src/role_assignment.cpp
  src/situation.cpp
  src/motion.cpp
  src/shoot.cpp
  src/pass.cpp
  src/defense.cpp
  src/roles.cpp
  src/formation.cpp
  src/strategy.cpp
)

# ---- 蓝队 DLL ----
add_library(Strategy4Blue SHARED src/dll_blue.cpp ${CORE_SOURCES})
target_compile_definitions(Strategy4Blue PRIVATE STRATEGY4BLUE_EXPORTS)
set_target_properties(Strategy4Blue PROPERTIES
  OUTPUT_NAME "Strategy4Blue"
  PREFIX ""
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# ---- 黄队 DLL ----
add_library(Strategy4Yellow SHARED src/dll_yellow.cpp ${CORE_SOURCES})
target_compile_definitions(Strategy4Yellow PRIVATE STRATEGY4YELLOW_EXPORTS)
set_target_properties(Strategy4Yellow PROPERTIES
  OUTPUT_NAME "Strategy4Yellow"
  PREFIX ""
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

foreach(tgt Strategy4Blue Strategy4Yellow)
  target_include_directories(${tgt} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
  if(MSVC)
    # 关键: 导出符号必须是 MSVC C++ mangled 名(与官方模板一致),
    # 平台按这些名字加载, 因此禁止 extern "C"。
    target_compile_options(${tgt} PRIVATE /W3 /utf-8)
    # 红线(docs/02): 静态链接运行时(/MT), DLL 不依赖 vcruntime/ucrt,
    # 否则比赛现场低配机(无 MSVC 运行时)加载失败 → 机器人不动。
    set_property(TARGET ${tgt} PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
  endif()
endforeach()

# ---- 测试(可选, 需要 -DBUILD_TEST=ON) ----
if(BUILD_TEST)
  add_executable(offline_test src/offline_test.cpp ${CORE_SOURCES})
  target_include_directories(offline_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
  target_compile_definitions(offline_test PRIVATE STRATEGY4BLUE_EXPORTS)
  if(MSVC)
    target_compile_options(offline_test PRIVATE /W3 /utf-8)
    set_property(TARGET offline_test PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
  endif()
endif()
