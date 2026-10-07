// route.hpp — 避障路径规划（网格 A*）；cm，原点左下角，蓝队守 x=220 右门
#ifndef SIMURO5_ROUTE_HPP
#define SIMURO5_ROUTE_HPP

#include "simuro5/geometry.hpp"

namespace simuro5 {

// 规划结果：waypoint 折线（wp[0]=起点，末点=终点），栈上定长
struct RoutePlan {
    bool found = false;
    double wp_x[64] = {0}, wp_y[64] = {0};
    int n_wp = 0;
    double length = 0.0;                      // 路径总长 cm
};

constexpr int ROUTE_MAX_OBSTACLES = 5;

// 求 (sx,sy)→(tx,ty) 最优避障折线；obs 半径须已含净空 inflate
// 契约：判定半径 = r−margin ⇒ 必须保证 inflate ≥ 本体6 + margin + 净空，否则会真撞
// found=false（起/终点在障碍内或无通路）时调用方回退直线
RoutePlan plan_route(double sx, double sy, double tx, double ty,
                     const CircleObstacle *obs, int n,
                     double margin = 0.5);

}  // namespace simuro5
#endif