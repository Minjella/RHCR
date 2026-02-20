C++
#include "SIPPSection.h"
#include <limits>
#include <iostream>

using namespace std;

// -----------------------------------------------------------------------------
// Helper: Heuristic Calculation
// -----------------------------------------------------------------------------
double SIPPSection::compute_h_value(const BasicGraph& G, int curr_sec, int goal_idx, const vector<pair<int, int>>& goals) {
    if (goal_idx >= goals.size()) return 0.0;

    // 1. 현재 위치에서 현재 목표까지의 거리
    double h = G.get_heuristic(curr_sec, goals[goal_idx].first);

    // 2. 남은 목표들 간의 거리 누적 (Goal A -> Goal B -> Goal C ...)
    for (size_t i = goal_idx; i < goals.size() - 1; ++i) {
        h += G.get_heuristic(goals[i].first, goals[i+1].first);
    }
    return h;
}

// -----------------------------------------------------------------------------
// Helper: Reconstruct Path
// -----------------------------------------------------------------------------
Path SIPPSection::reconstruct_path(shared_ptr<SIPPNode> goal_node) {
    vector<State> path;
    auto curr = goal_node;

    while (curr != nullptr) {
        // 현재 노드의 도착 상태 추가
        path.emplace_back(curr->section_id, curr->arrival_time, 0);

        if (curr->parent) {
            auto prev = curr->parent;
            int travel_time = curr->arrival_time - prev->arrival_time;
            
            // SIPP는 시간을 점프하므로, 빈 시간(Wait or Move)을 채워줌
            // 여기서는 단순하게 이전 위치에서 대기하다가 이동했다고 가정하거나
            // 섹션 이동 시간을 고려하여 역순으로 채움
            // (구체적인 채우기 로직은 프로젝트의 Move/Wait 정의에 따라 수정 필요)
            
            // 예: 도착 직전까지는 '이동 중' 상태 or '대기' 상태
            for (int t = curr->arrival_time - 1; t > prev->arrival_time; --t) {
                // 단순화: 이동 시간이 길다면 중간 상태는 prev_loc 유지
                path.emplace_back(prev->section_id, t, 0); 
            }
        }
        curr = curr->parent;
    }

    // 역순 정렬 (Start -> Goal)
    reverse(path.begin(), path.end());
    return path;
}

// -----------------------------------------------------------------------------
// Main: RUN
// -----------------------------------------------------------------------------
Path SIPPSection::run(const BasicGraph& G, 
                      const State& start, 
                      const vector<pair<int, int>>& goals, 
                      ReservationSection& rs, 
                      const PriorityGraph* pg,
                      int agent_id,
                      int capacity) 
{
    // 초기화
    num_expanded = 0;
    num_generated = 0;
    
    // Open List
    priority_queue<SIPPNode, vector<SIPPNode>, greater<SIPPNode>> open_list;
    
    // Closed List: <SectionID, GoalIdx> -> Min Arrival Time으로 관리하거나
    // SIPP 특성상 <SectionID, IntervalID, GoalIdx> 방문 여부 체크
    // 여기서는 간단히 visited set 사용 (Key: Section * 1000 + GoalIdx 등으로 해싱 추천)
    // 정확성을 위해 map으로 g_score 관리
    struct NodeKey {
        int s_id, i_id, g_idx;
        bool operator==(const NodeKey& o) const { 
            return s_id == o.s_id && i_id == o.i_id && g_idx == o.g_idx; 
        }
    };
    struct KeyHash {
        size_t operator()(const NodeKey& k) const {
            return hash<int>()(k.s_id) ^ hash<int>()(k.i_id) ^ hash<int>()(k.g_idx);
        }
    };
    unordered_map<NodeKey, double, KeyHash> closed_list;

    // 1. Start Node 생성
    // 시작 섹션의 Safe Interval들 가져오기
    auto start_intervals = rs.get_safe_intervals(start.location, capacity);
    bool start_found = false;

    for (int i = 0; i < start_intervals.size(); ++i) {
        auto [i_start, i_end, i_count] = *next(start_intervals.begin(), i);
        
        // 시작 시간이 구간 안에 포함되는지 확인
        if (start.timestep >= i_start && start.timestep < i_end) {
            double h = compute_h_value(G, start.location, 0, goals);
            open_list.emplace(start.location, i, start.timestep, 0, 0.0, 0.0 + h, nullptr);
            start_found = true;
            num_generated++;
            break; // 시작 시간은 하나의 구간에만 속함
        }
    }

    if (!start_found) {
        // 시작부터 막힌 경우 (Capacity Full or Collision)
        return Path(); 
    }

    // 2. Search Loop
    while (!open_list.empty()) {
        // Pop Best Node
        SIPPNode current = open_list.top();
        open_list.pop();
        num_expanded++;

        // Check Closed List (더 나쁜 경로로 왔으면 스킵)
        NodeKey key = {current.section_id, current.interval_id, current.goal_idx};
        if (closed_list.find(key) != closed_list.end() && closed_list[key] <= current.g_score) {
            continue;
        }
        closed_list[key] = current.g_score;

        // -------------------------------------------------------
        // Goal Logic (Multi-Goal Sequence)
        // -------------------------------------------------------
        int curr_goal_sec = goals[current.goal_idx].first;
        int curr_goal_min_time = goals[current.goal_idx].second;

        // 현재 목표 섹션에 도착했고, 시간 조건도 만족했다면?
        if (current.section_id == curr_goal_sec && current.arrival_time >= curr_goal_min_time) {
            
            // 마지막 목표였다면 종료
            if (current.goal_idx == goals.size() - 1) {
                return reconstruct_path(make_shared<SIPPNode>(current));
            }

            // 다음 목표로 전환 (위치는 그대로, Goal Index만 증가)
            int next_goal_idx = current.goal_idx + 1;
            double new_h = compute_h_value(G, current.section_id, next_goal_idx, goals);
            
            // 현재 상태에서 Goal Index만 바꿔서 다시 Push
            // (같은 섹션, 같은 구간, 같은 시간인데 목표만 바뀜)
            open_list.emplace(
                current.section_id, 
                current.interval_id, 
                current.arrival_time, 
                next_goal_idx, 
                current.g_score, 
                current.g_score + new_h, 
                make_shared<SIPPNode>(current) // 부모 연결
            );
            continue; 
        }

        // -------------------------------------------------------
        // Expand Neighbors
        // -------------------------------------------------------
        // 현재 구간 정보 가져오기 (메모리 최적화를 위해 인덱스로 접근하거나 다시 조회)
        // 여기서는 편의상 다시 조회 (캐싱되어 있어 빠름)
        auto intervals = rs.get_safe_intervals(current.section_id, capacity);
        auto [curr_start, curr_end, curr_count] = *next(intervals.begin(), current.interval_id);

        // 1. 같은 섹션의 다음 안전 구간으로 대기 (Wait)
        // 현재 구간 끝난 직후 다음 구간이 있다면 연결
        if (current.interval_id + 1 < intervals.size()) {
             auto [next_start, next_end, next_count] = *next(intervals.begin(), current.interval_id + 1);
             
             // 다음 구간 시작 시간이 연결되어 있는지 확인 (혹은 갭이 있어도 대기 가능하면 이동)
             // 단순히 시간만 흐르는 것이므로 위치는 동일
             int wait_arrival = next_start;
             double wait_cost = wait_arrival - current.arrival_time; // 대기 시간 비용
             
             open_list.emplace(
                 current.section_id,
                 current.interval_id + 1,
                 wait_arrival,
                 current.goal_idx,
                 current.g_score + wait_cost,
                 current.g_score + wait_cost + compute_h_value(G, current.section_id, current.goal_idx, goals),
                 make_shared<SIPPNode>(current)
             );
        }

        // 2. 이웃 섹션으로 이동 (Move)
        for (const auto& edge : G.get_neighbors(current.section_id)) {
            int next_sec = edge.to;
            int travel_time = edge.cost; // 섹션 간 이동 시간
            int min_arrival = current.arrival_time + travel_time;

            // 다음 섹션의 Safe Intervals 조회
            auto next_intervals = rs.get_safe_intervals(next_sec, capacity);

            for (int i = 0; i < next_intervals.size(); ++i) {
                auto [ni_start, ni_end, ni_count] = *next(next_intervals.begin(), i);

                // [핵심] 도착 가능 시간 계산
                // 가장 빨리 도착하는 시간 vs 구간이 열리는 시간
                int arrival = max(min_arrival, ni_start);

                // 유효성 검사:
                // 1. 구간이 닫히기 전에 도착해야 함
                // 2. 현재 구간(curr_end) 내에 출발할 수 있어야 함 (Departure constraint)
                //    출발 시간 = arrival - travel_time
                if (arrival < ni_end && (arrival - travel_time) < curr_end) {

                    // 3. [PBS Check] 정밀 충돌 검사 (is_safe)
                    // SIPP는 '용량'만 보장하므로, 특정 Agent와의 충돌은 여기서 체크
                    // 이동하는 'Edge'와 도착하는 'Node'가 안전한지 확인
                    // (사용자님의 is_safe 구현에 따라 인자 조절)
                    if (rs.is_safe(arrival, next_sec, current.section_id, -1, agent_id, capacity, pg)) {
                        
                        double move_cost = travel_time; 
                        double wait_cost = (arrival - min_arrival); // 추가 대기 시간
                        // 혼잡도 패널티 추가 가능 (ni_count * weight)

                        double new_g = current.g_score + move_cost + wait_cost;
                        double new_h = compute_h_value(G, next_sec, current.goal_idx, goals);

                        open_list.emplace(
                            next_sec,
                            i,
                            arrival,
                            current.goal_idx,
                            new_g,
                            new_g + new_h,
                            make_shared<SIPPNode>(current)
                        );
                    }
                }
            }
        }
    }

    // 경로 없음
    return Path();
}