//
// Created by ubuntu on 25-3-2.
//
#include "planner.h"

Planner::Planner() {

}


vector<Vector2i> Planner::Bfs(const Vector2i &start, const Vector2i &goal,
                     const unordered_set<Vector2i, Vector2iHash>& obstacles) {
    // 定义四个方向的移动：上、右、下、左（注释掉原来的版本）
    vector<pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    
    // 定义八个方向的移动：上、右上、右、右下、下、左下、左、左上
    // vector<pair<int, int>> directions = {
    //     {0, 1}, {1, 1}, {1, 0}, {1, -1}, 
    //     {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}
    // };
    
    // 使用队列进行BFS搜索
    queue<Vector2i> q;
    q.push(start);
    
    // 使用哈希表记录已访问的节点
    unordered_set<Vector2i, Vector2iHash> visited;
    visited.insert(start);
    
    // 使用哈希表记录每个节点的父节点，用于重建路径
    unordered_map<Vector2i, Vector2i, Vector2iHash> parent;
    
    bool found = false;
    
    // BFS主循环
    while (!q.empty() && !found) {
        Vector2i current = q.front();
        q.pop();
        
        // 如果到达目标
        if (current == goal) {
            found = true;
            break;
        }
        
        // 探索八个方向
        for (const auto& dir : directions) {
            Vector2i next(current.x() + dir.first, current.y() + dir.second);
            
            // 检查是否已访问或是障碍物
            if (visited.count(next) == 0 && obstacles.count(next) == 0) {
                visited.insert(next);
                q.push(next);
                parent[next] = current;
            }
        }
    }
    
    // 如果找到路径，重建路径
    vector<Vector2i> path;
    if (found) {
        Vector2i current = goal;
        while (!(current == start)) {
            path.push_back(current);
            current = parent[current];
        }
        path.push_back(start);
        reverse(path.begin(), path.end());
    }
    
    return path;
}


vector<Vector2i> Planner::AStar(const Vector2i &start, const Vector2i &goal,
                                const unordered_set<Vector2i, Vector2iHash>& obstacles) {
    // 定义四个方向的移动：上、右、下、左（注释掉原来的版本）
    vector<pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    
    // 定义八个方向的移动：上、右上、右、右下、下、左下、左、左上
    // vector<pair<int, int>> directions = {
    //     {0, 1}, {1, 1}, {1, 0}, {1, -1}, 
    //     {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}
    // };
    
    // 使用优先队列进行A*搜索
    auto cmp = [](const pair<Vector2i, double>& a, const pair<Vector2i, double>& b) {
        return a.second > b.second; // 最小堆
    };
    priority_queue<pair<Vector2i, double>, vector<pair<Vector2i, double>>, decltype(cmp)> pq(cmp);
    
    // 启发式函数：曼哈顿距离
    auto h = [&goal](const Vector2i& pos) {
        return abs(pos.x() - goal.x()) + abs(pos.y() - goal.y());
    };
    
    // g值：从起点到当前节点的实际代价
    unordered_map<Vector2i, double, Vector2iHash> g_score;
    g_score[start] = 0;
    
    // f值：g值 + 启发式函数值
    pq.push({start, h(start)});
    
    // 记录每个节点的父节点，用于重建路径
    unordered_map<Vector2i, Vector2i, Vector2iHash> parent;
    
    // 记录已经处理过的节点
    unordered_set<Vector2i, Vector2iHash> closed_set;
    
    bool found = false;
    
    // A*主循环
    while (!pq.empty() && !found) {
        Vector2i current = pq.top().first;
        pq.pop();
        
        // 如果已经处理过该节点，跳过
        if (closed_set.count(current) > 0)
            continue;
        
        // 如果到达目标
        if (current == goal) {
            found = true;
            break;
        }
        
        // 标记为已处理
        closed_set.insert(current);
        
        // 探索八个方向
        for (const auto& dir : directions) {
            Vector2i next(current.x() + dir.first, current.y() + dir.second);
            
            // 检查是否是障碍物或已处理
            if (obstacles.count(next) > 0 || closed_set.count(next) > 0)
                continue;
            
            // 计算新的g值 (对角线移动距离为√2)
            double movement_cost = (abs(dir.first) + abs(dir.second) == 2) ? 1.414 : 1.0;
            double tentative_g = g_score[current] + movement_cost;
            
            // 如果是新节点或找到了更好的路径
            if (g_score.count(next) == 0 || tentative_g < g_score[next]) {
                parent[next] = current;
                g_score[next] = tentative_g;
                double f = tentative_g + h(next);
                pq.push({next, f});
            }
        }
    }
    
    // 如果找到路径，重建路径
    vector<Vector2i> path;
    if (found) {
        Vector2i current = goal;
        while (!(current == start)) {
            path.push_back(current);
            current = parent[current];
        }
        path.push_back(start);
        reverse(path.begin(), path.end());
    }
    
    return path;
}


std::vector<State*> Planner::HybridAStar(const State& start, const State& goal,
                                const unordered_set<Vector2i, Vector2iHash>& obstacles_index,
                                double wheelbase, double step_size, double max_steer) {
    // 先使用A*算法获取一条网格路径，用于指导Hybrid A*搜索
    Vector2i start_index(round(start.x / step_size), round(start.y / step_size));
    Vector2i goal_index(round(goal.x / step_size), round(goal.y / step_size));
    vector<Vector2i> astar_path = AStar(start_index, goal_index, obstacles_index);
    
    // 将A*路径点转换成查找表，用于快速计算启发式函数
    unordered_map<Vector2i, int, Vector2iHash> astar_path_lookup;
    for (int i = 0; i < astar_path.size(); ++i) {
        astar_path_lookup[astar_path[i]] = i;
    }
    
    // 用于计算启发式函数的辅助函数
    auto heuristic = [&goal, &astar_path, &astar_path_lookup, &step_size](const State* s) -> double {
        // 原始欧几里得距离启发式函数（注释掉）
        /*
        double dx = s->x - goal.x;
        double dy = s->y - goal.y;
        return sqrt(dx*dx + dy*dy);
        */
        
        // 使用A*路径作为启发式函数
        Vector2i current_index(round(s->x / step_size), round(s->y / step_size));
        
        // 如果当前位置在A*路径上，使用到目标的路径长度作为启发式
        if (astar_path_lookup.count(current_index) > 0) {
            int path_idx = astar_path_lookup[current_index];
            return (astar_path.size() - 1 - path_idx) * step_size;
        }
        
        // 如果不在路径上，找到最近的路径点
        double min_dist = std::numeric_limits<double>::max();
        int closest_idx = 0;
        
        for (int i = 0; i < astar_path.size(); ++i) {
            double dx = current_index.x() - astar_path[i].x();
            double dy = current_index.y() - astar_path[i].y();
            double dist = sqrt(dx*dx + dy*dy);
            
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        
        // 返回到最近路径点的距离加上从该点到目标的路径长度
        return min_dist * step_size + (astar_path.size() - 1 - closest_idx) * step_size;
    };
    
    // 离散化状态以便检查是否访问过
    auto discretizeState = [&step_size](const State* s) -> Vector2i {
        int x = round(s->x / (step_size * 0.5));
        int y = round(s->y / (step_size * 0.5));
        int theta = round(s->theta / (M_PI / 18.0)); // 每10度离散化
        return Vector2i(x * 1000 + theta, y);
    };
    
    // 判断是否达到目标的函数
    auto isGoalReached = [&goal, step_size](const State* s) -> bool {
        double dx = s->x - goal.x;
        double dy = s->y - goal.y;
        double distance = sqrt(dx*dx + dy*dy);
        return distance < step_size;
    };
    
    // 检查状态是否碰撞障碍物
    auto checkCollision = [&obstacles_index, step_size](const State* s) -> bool {
        Vector2i pos(round(s->x / step_size), round(s->y / step_size));
        return obstacles_index.count(pos) > 0;
    };
    
    // 定义优先队列，按f值排序
    auto cmp = [](const State* a, const State* b) {
        return a->f > b->f;
    };
    priority_queue<State*, vector<State*>, decltype(cmp)> open_list(cmp);
    
    // 创建起始状态的副本
    State* startState = new State(start.x, start.y, start.theta, 0, heuristic(&start), nullptr);
    open_list.push(startState);
    
    // 已访问状态集合
    unordered_set<Vector2i, Vector2iHash> closed_set;
    
    // 转向角度列表
    vector<double> steer_angles = {-max_steer, -max_steer/2, 0, max_steer/2, max_steer};
    
    // Hybrid A*主循环
    while (!open_list.empty()) {
        // 取出f值最小的状态
        State* current = open_list.top();
        open_list.pop();
        
        // 检查是否到达目标
        if (isGoalReached(current)) {
            // 构建路径并返回
            std::vector<State*> path;
            State* s = current;
            while (s != nullptr) {
                path.push_back(s);
                s = s->parent;
            }
            reverse(path.begin(), path.end());
            return path;
        }
        
        // 将当前状态标记为已访问
        Vector2i discretized = discretizeState(current);
        if (closed_set.count(discretized) > 0) {
            delete current; // 释放内存
            continue;
        }
        closed_set.insert(discretized);
        
        // 对每个可能的转向角度，生成下一个状态
        for (double steer : steer_angles) {
            // 计算前轮位置
            double beta = atan(tan(steer) / 2.0);
            
            // 应用自行车模型
            double new_x = current->x + step_size * cos(current->theta + beta);
            double new_y = current->y + step_size * sin(current->theta + beta);
            double new_theta = current->theta + step_size * tan(steer) / wheelbase;
            
            // 规范化角度到[-pi, pi]
            while (new_theta > M_PI) new_theta -= 2 * M_PI;
            while (new_theta < -M_PI) new_theta += 2 * M_PI;
            
            // 创建新状态
            State* next = new State(new_x, new_y, new_theta, current->g + step_size, 0, current);
            
            // 检查碰撞
            if (checkCollision(next)) {
                delete next;
                continue;
            }
            
            // 计算f值
            next->f = next->g + heuristic(next);
            
            // 检查是否已访问
            Vector2i next_discretized = discretizeState(next);
            if (closed_set.count(next_discretized) > 0) {
                delete next;
                continue;
            }
            
            // 加入开放列表
            open_list.push(next);
        }
    }
    
    // 如果没有找到路径，返回空路径
    return {};
}