// simuro_interface.hpp — 官方平台接口：签名与文件名不可改，须 32 位 MSVC 编译，平台按 mangled 名加载；坐标原点左下角，cm/角度，蓝队守右门(x≈220)，1 号为守门员
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

// 官方 5 个导出接口：蓝/黄按各自 EXPORTS 宏条件编译，避免同名 dllexport/import 冲突(C4273)
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
