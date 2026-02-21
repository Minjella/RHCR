#include "SIPPSection.h"
#include "MapSystem.h"
#include <climits>
#include <ctime>

// ------------------------------------------------------------------------------------------------
// [1] 더미(Dummy) run 함수: 기존 시스템이 ReservationTable로 호출할 경우 에러를 뿜게 만듦
// ------------------------------------------------------------------------------------------------
Path SIPPSection::run(const BasicGraph& G, const State& start, const vector<pair<int, int>>& goal_location, ReservationTable& RT)
{
    std::cerr << "[Error] SIPPSection은 ReservationTable을 사용하지 않습니다! 새로운 run을 호출하세요." << std::endl;
    exit(1);
    return Path();
}

// ------------------------------------------------------------------------------------------------
// [2] Path 역추적 (기존 SIPP.cpp와 거의 동일 - SIPPNode 타입만 매칭)
// ------------------------------------------------------------------------------------------------
SectionPath SIPPSection::updatePath(const SIPPNode* goal)
{
    SectionPath path;
    path_cost = goal->getFVal();
    num_of_conf = goal->conflicts;

    const SIPPNode* curr = goal;

    int child_exit_index = -1;
    std::vector<int> child_wait_list = {};
    bool has_child = false;

    while (curr != nullptr)
    {
        SectionState state_copy = curr->s_state;

        // if has child, update exit index & wait list.
        if (has_child){
            state_copy.exit_index = child_exit_index;
            state_copy.wait_list = child_wait_list;
        } else{
            // goal section -> exit index = goal_index
            state_copy.exit_index = state_copy.goal_index;
        }

        path.push_back(state_copy);

        if (curr->parent != nullptr){
            child_exit_index = curr -> parent_exit_index;
            child_wait_list = curr -> parent_wait_list;
            has_child = true;
        }

        if (curr->parent == nullptr){
            int t = curr->s_state.timestep - 1;
            for (; t>=0; t--){
                path.push_back(SectionState(-1, -1, -1, t));
            }
        }

        curr = curr->parent;
    }

    std::reverse(path.begin(), path.end());

    return path;
}

// ------------------------------------------------------------------------------------------------
// [3] 진짜 SIPP 알고리즘 본체 (ReservationSection 연동)
// ------------------------------------------------------------------------------------------------
SectionPath SIPPSection::run_section(const State& start, 
                                     const vector<pair<int, int>>& goal_location,
                                     ReservationSection& rs, 
                                     int agent_id, int capacity, void* map_system_ptr)
{
    num_expanded = 0; num_generated = 0; runtime = 0;
    clock_t start_clock = std::clock();

    // MapSystem 포인터 캐스팅 (클래스명에 맞춰 수정)
    MapSystem* MapSys = static_cast<MapSystem*>(map_system_ptr);

    // ✨ 1. 시작 그리드(location)를 통해 시작 섹션 ID 알아내기
    int start_section_id = MapSys->get_section_id(start.location);
    
    // 시작 상태 빚어내기 (start_index는 현재 위치 그대로 사용)
    SectionState start_state(start_section_id, start.location, -1, start.timestep);

    // 초기 휴리스틱 계산
    double h_val = compute_h_value(G, start.location, 0, goal_location);

    // 2. 출발지의 Safe Interval 찾기
    SecInterval start_interval = std::make_tuple(0, 0, 0);
    bool start_found = false;
    for (auto const& interval : rs.get_safe_intervals(start_section_id, capacity)) {
        if (std::get<0>(interval) <= start.timestep && std::get<1>(interval) > start.timestep) {
            start_interval = interval;
            start_found = true;
            break;
        }
    }

    if (!start_found) return SectionPath();

    // 3. 시작 노드 생성
    auto start_node = new SIPPNode(start_state, 0, h_val, start_interval, nullptr, 0, -1);
    num_generated++;
    start_node->open_handle = open_list.push(start_node);
    start_node->in_openlist = true;
    allNodes_table.insert(start_node);
    min_f_val = start_node->getFVal();
    focal_bound = min_f_val * suboptimal_bound;
    start_node->focal_handle = focal_list.push(start_node);

    // 4. 메인 루프
    while (!focal_list.empty())
    {
        SIPPNode* curr = focal_list.top(); 
        focal_list.pop();
        open_list.erase(curr->open_handle);
        curr->in_openlist = false;
        num_expanded++;

        // ✨ 5. 정확한 Goal 도달 체크 로직 (매우 중요)
        int current_goal_grid = goal_location[curr->goal_id].first;
        int current_goal_time = goal_location[curr->goal_id].second;
        int goal_section_id = MapSys->get_section_id(current_goal_grid);

        // 만약 현재 노드가 '목표 지점이 있는 섹션'에 진입했다면?
        if (curr->s_state.section_id == goal_section_id) 
        {
            // 진입점(curr->s_state.start_index)에서 실제 목표 그리드까지의 거리 계산
            double dist_to_goal = MapSys->get_distance(curr->s_state.start_index, current_goal_grid);
            int arrive_time_at_goal = curr->s_state.timestep + dist_to_goal;

            // 목표 릴리즈 시간 이후에 도달할 수 있는지 확인
            if (arrive_time_at_goal >= current_goal_time) 
            {
                curr->goal_id++;
                // 모든 목적지를 다 찍었다면 종료!
                if (curr->goal_id == (int)goal_location.size())
                {
                    SectionPath path = updatePath(curr);
                    releaseClosedListNodes();
                    open_list.clear(); focal_list.clear();
                    runtime = (std::clock() - start_clock) * 1.0 / CLOCKS_PER_SEC;
                    return path;
                }
            }
        }

        // 6. 이웃 섹션 확장 (MapSystem 연동)
        auto neighbors = MapSys->get_neighbors(curr->s_state.section_id);

        for (const auto& neighbor : neighbors) 
        {
            int next_section_id = neighbor.section_id;
            int curr_exit_index = neighbor.curr_exit_index;
            int next_start_index = neighbor.next_start_index;
            
            double total_travel_cost = neighbor.internal_cost + neighbor.edge_cost;
            int arrival_time = curr->s_state.timestep + total_travel_cost;

            // 목적지까지의 휴리스틱 (현재 목적지의 그리드 좌표 기준)
            double next_h_val = compute_h_value(G, next_start_index, curr->goal_id, goal_location);

            // ReservationSection 체크
            auto safe_intervals = rs.get_safe_intervals(next_section_id, capacity);
            for (const auto& interval : safe_intervals)
            {
                if (std::get<1>(interval) <= arrival_time) continue;

                int section_congestion = std::get<2>(interval);

                generate_node(interval, curr, next_section_id, next_start_index, curr_exit_index, 
                              total_travel_cost, arrival_time, next_h_val, section_congestion);
            }
        }

        // ... (FOCAL / OPEN 리스트 업데이트 - 이전과 동일) ...
    }

    releaseClosedListNodes();
    open_list.clear(); focal_list.clear();
    return SectionPath();
}

// ------------------------------------------------------------------------------------------------
// [4] 노드 생성 (기존 로직과 동일, Interval -> SecInterval 매핑)
// ------------------------------------------------------------------------------------------------
void SIPPSection::generate_node(const SecInterval& interval, SIPPNode* curr, 
                                int next_section_id, int next_start_index, int curr_exit_index,
                                const std::vector<int>& wait_list,
                                double travel_cost, int arrival_time, double h_val, int section_congestion)
{
    // 1. 실제 도착(진입) 시간
    int timestep = std::max(std::get<0>(interval), arrival_time);
    
    // 2. 대기 시간 계산 (이전 섹션에서 얼마나 머무르다 넘어와야 하는가)
    int extra_wait_time = timestep - arrival_time;
    std::vector<int> final_wait_list = wait_list;
    if (extra_wait_time > 0){
        for(int i=0; i<extra_wait_time; i++){
            final_wait_list.push_back(curr->parent_exit_index);
        }
    }
    int wait_time = sizeof(final_wait_list);

    // 3. g_val 계산 (이동 시간 + 대기 시간) -> 시간적 비용만 철저히 관리
    double g_val = curr->g_val + wait_time; 

    // 4. 혼잡도 누적 계산 (FOCAL 정렬용 Tie-breaker)
    int next_conflicts = curr->conflicts + section_congestion;

    // 5. SectionState 생성 (방금 도착했으므로 내 출구 exit_index는 아직 모름 -> -1)
    SectionState next_state(next_section_id, next_start_index, -1, timestep);

    // 6. 노드 생성 (부모의 출구 curr_exit_index를 내 노드에 기록해둠)
    auto next = new SIPPNode(next_state, g_val, h_val, interval, curr, next_conflicts, curr_exit_index, final_wait_list);

    auto it = allNodes_table.find(next);
    if (it == allNodes_table.end())
    {
        next->open_handle = open_list.push(next);
        next->in_openlist = true;
        num_generated++;
        if (next->getFVal() <= focal_bound)
            next->focal_handle = focal_list.push(next);
        allNodes_table.insert(next);
        return;
    }

    // 기존 노드 업데이트 로직 (g_val + h_val 비교 후 conflicts 비교)
    SIPPNode* existing_next = *it;
    double existing_f_val = existing_next->getFVal();
    double new_f_val = g_val + h_val;

    if (existing_next->in_openlist)
    {  
        if (existing_f_val > new_f_val || (existing_f_val == new_f_val && existing_next->conflicts > next_conflicts))
        {
            bool add_to_focal = false;  
            bool update_in_focal = false;  
            bool update_open = false;
            
            if (new_f_val <= focal_bound) {  
                if (existing_f_val > focal_bound) add_to_focal = true;  
                else update_in_focal = true;  
            }
            if (existing_f_val > new_f_val) update_open = true;
            
            existing_next->s_state = next->s_state;
            existing_next->g_val = g_val;
            existing_next->h_val = h_val;
            existing_next->parent = curr;
            existing_next->depth = next->depth;
            existing_next->conflicts = next_conflicts;
            existing_next->parent_exit_index = curr_exit_index; // 출구 정보도 업데이트

            if (update_open) open_list.increase(existing_next->open_handle);  
            if (add_to_focal) existing_next->focal_handle = focal_list.push(existing_next);
            if (update_in_focal) focal_list.update(existing_next->focal_handle);  
        }
    }
    else
    {  
        // Reopen 로직
        if (existing_f_val > new_f_val || (existing_f_val == new_f_val && existing_next->conflicts > next_conflicts))
        {
            existing_next->s_state = next->s_state;
            existing_next->g_val = g_val;
            existing_next->h_val = h_val;
            existing_next->parent = curr;
            existing_next->depth = next->depth;
            existing_next->conflicts = next_conflicts;
            existing_next->parent_exit_index = curr_exit_index;
            
            existing_next->open_handle = open_list.push(existing_next);
            existing_next->in_openlist = true;
            if (existing_f_val <= focal_bound)
                existing_next->focal_handle = focal_list.push(existing_next);
        }
    }  

    delete(next); 
}

// ------------------------------------------------------------------------------------------------
// [5] 메모리 정리
// ------------------------------------------------------------------------------------------------
inline void SIPPSection::releaseClosedListNodes()
{
    for (auto it = allNodes_table.begin(); it != allNodes_table.end(); it++)
        delete (*it);
    allNodes_table.clear();
}