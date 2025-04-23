//
// Created by ubuntu on 25-3-2.
//

#ifndef AVP_WS_PLANNER_H
#define AVP_WS_PLANNER_H

#endif //AVP_WS_PLANNER_H

#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>
#include <algorithm>  // reverse
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

struct Vector2iHash {
    size_t operator()(const Vector2i &v) const {
        size_t h1 = hash<int>{}(v.x());
        size_t h2 = hash<int>{}(v.y());
        return h1 ^ (h2 << 1);
    }
};

constexpr double min_angle_diff = M_PI/12;

// 定义车辆状态
struct State {
    double x, y, theta; // 位置和朝向角
    double g;           // 累计代价
    double f;           // 总估计代价
    State* parent;      // 父节点指针

    State(double x, double y, double theta, double g, double f, State* parent)
            : x(x), y(y), theta(theta), g(g), f(f), parent(parent) {}

    // 比较器，用于优先队列
    bool operator>(const State& other) const {
        return f > other.f;
    }
        
    // 哈希函数，用于判断两个状态是否相同
    // 因为使用了 std::unordered_set<State, StateHash> 和 std::unordered_map<State, double, StateHash>
    // State 类型 需要支持 相等性 比较操作符 operator==
    // 如果未定义，编译器会报错
    bool operator==(const State& other) const {
        return std::fabs(x - other.x) < 0.5 && //1e-1
            std::fabs(y - other.y) < 0.5 &&
            std::fabs(theta - other.theta) < min_angle_diff; // 1e-1
    }
};




class Planner {
public:
    Planner();
    vector<Vector2i> Bfs(const Vector2i &start, const Vector2i &goal,
                                  const unordered_set<Vector2i, Vector2iHash>& obstacles);
    vector<Vector2i> AStar(const Vector2i &start, const Vector2i &goal,
                                    const unordered_set<Vector2i, Vector2iHash>& obstacles);
    vector<Vector2i> AStar_old(const Vector2i &start, const Vector2i &goal,
                                    const unordered_set<Vector2i, Vector2iHash>& obstacles);
    
    // HybridAStar + take 欧式距离/曼哈顿距离 as heuristic function
    std::vector<State*> HybridAStar(const State& start, const State& goal,
                                    const unordered_set<Vector2i, Vector2iHash>& obstacles_index,
                                    double wheelbase, double step_size, double max_steer);
    // std::vector<State*> HybridAStar(const State& start, const State& goal,
    //                                     const unordered_set<Vector2i, Vector2iHash>& obstacles_index,
    //                                     double wheelbase, double step_size, double max_steer);
                                    
    // // HybridAStar + take BFS as heuristic function
    // std::vector<State*> HybridAStar_BFS_as_Heuristic(const State& start, const State& goal,
    //                                 const unordered_set<Vector2i, Vector2iHash>& obstacles_index,
    //                                 double wheelbase, double step_size, double max_steer);
    // // HybridAStar + take Astar as heuristic function
    // std::vector<State*> HybridAStar_Astar_as_Heuristic(const State& start, const State& goal,
    //                                 const unordered_set<Vector2i, Vector2iHash>& obstacles_index,
    //                                 double wheelbase, double step_size, double max_steer);
    // // Mixed Method: HybridAStar + RS 曲线
    // std::vector<State*> HybridAStar_Mixed_RS(const State& start, const State& goal,
    //                                 const unordered_set<Vector2i, Vector2iHash>& obstacles_index,
    //                                 double wheelbase, double step_size, double max_steer);
    // // Mixed Method: HybridAStar + Dubins 曲线
    // std::vector<State*> HybridAStar_Mixed_Dubins(const State& start, const State& goal,
    //                                 const unordered_set<Vector2i, Vector2iHash>& obstacles_index,
    //                                 double wheelbase, double step_size, double max_steer);
};