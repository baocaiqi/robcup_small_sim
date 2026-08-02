// ============================================================
// simuro_interface.hpp — 官方平台接口（与官方模板逐字一致的签名）
//
// ⚠️ 兼容性红线（不要改）：
//   1. 以下 struct / enum / 函数签名与官方 Strategy4Blue.h 完全一致，
//      平台(WorldModel.exe)按 MSVC C++ mangled 导出名加载，禁止 extern "C"。
//   2. 必须用 MSVC 编译 32 位 DLL（MinGW 的 mangled 名不兼容）。
//   3. 文件名必须保持 Strategy4Blue.dll / Strategy4Yellow.dll。
//
// 坐标系：原点 (0,0) 场地左下角，x 向右，y 向上，单位厘米，角度制。
// 蓝队守右门(x≈220)，黄队守左门(x≈0)；1 号机器人是守门员。
// ============================================================
#ifndef SIMURO_INTERFACE_HPP
#define SIMURO_INTERFACE_HPP

#ifdef STRATEGY4BLUE_EXPORTS
#define STRATEGY4BLUE_API __declspec(dllexport)
#else
#define STRATEGY4BLUE_API __declspec(dllimport)
#endif

#ifdef STRATEGY4YELLOW_EXPORTS
#define STRATEGY4YELLOW_API __declspec(dllexport)
#else
#define STRATEGY4YELLOW_API __declspec(dllimport)
#endif

#include <math.h>

const long PLAYERS_PER_SIDE = 5;
const double PI = 3.1415926535;

typedef struct
{
    double x, y, z;
} Vector3D;

typedef struct
{
    long left, right, top, bottom;
} Bounds;

typedef struct
{
    Vector3D pos;
    double rotation;
    double velocityLeft, velocityRight;
} Robot;

typedef struct
{
    Vector3D pos;
    double rotation;
} OpponentRobot;

typedef struct
{
    Vector3D pos;
} Ball;

typedef struct
{
    Robot home[PLAYERS_PER_SIDE];
    OpponentRobot opponent[PLAYERS_PER_SIDE];
    Ball currentBall, lastBall, predictedBall;
    Bounds fieldBounds, goalBounds;
    long gameState;
    long whosBall;
    void *userData;
} Environment;

enum PlayMode {
    PM_PlayOn = 0,
    PM_FreeBall_LeftTop = 1,
    PM_FreeBall_LeftBot = 2,
    PM_FreeBall_RightTop = 3,
    PM_FreeBall_RightBot = 4,
    PM_PlaceKick_Yellow = 5,
    PM_PlaceKick_Blue = 6,
    PM_PenaltyKick_Yellow = 7,
    PM_PenaltyKick_Blue = 8,
    PM_FreeKick_Yellow = 9,
    PM_FreeKick_Blue = 10,
    PM_GoalKick_Yellow = 11,
    PM_GoalKick_Blue = 12
};

// 官方 5 个导出接口。蓝/黄按各自 EXPORTS 宏条件编译，
// 避免同一函数名在单个 DLL 内出现 dllexport/dllimport 冲突(C4273)。
#ifdef STRATEGY4BLUE_EXPORTS
STRATEGY4BLUE_API void SetFormerRobots(PlayMode gameState, Robot robots[]);
STRATEGY4BLUE_API void SetLaterRobots(PlayMode gameState, Robot formerRobots[], Vector3D ball, Robot laterRobots[]);
STRATEGY4BLUE_API void SetBall(PlayMode gameState, Vector3D * pBall);
STRATEGY4BLUE_API void RunStrategy(Environment *pEnv);
STRATEGY4BLUE_API void SetBlueTeamName(char* teamName);
#endif

#ifdef STRATEGY4YELLOW_EXPORTS
STRATEGY4YELLOW_API void SetFormerRobots(PlayMode gameState, Robot robots[]);
STRATEGY4YELLOW_API void SetLaterRobots(PlayMode gameState, Robot formerRobots[], Vector3D ball, Robot laterRobots[]);
STRATEGY4YELLOW_API void SetBall(PlayMode gameState, Vector3D * pBall);
STRATEGY4YELLOW_API void RunStrategy(Environment *pEnv);
STRATEGY4YELLOW_API void SetYellowTeamName(char* teamName);
#endif

#endif // SIMURO_INTERFACE_HPP
