#include "SIPPSection.h"
#include "MapSystem.h"
#include <climits>
#include <ctime>

// [1] 더미 run 함수 유지
Path SIPPSection::run(const BasicGraph& G, const State& start, const vector<pair<int, int>>& goal_location, ReservationTable& RT) {
    std::cerr << "[Error] SIPPSection은 ReservationTable을 사용하지 않습니다! 새로운 run을 호출하세요." << std::endl;
    exit(1);
    return Path();
}

// ------------------------------------------------------------------------------------------------
// [2] Path 역추적 (단 한 번의 재계산으로 완벽한 경로 복원!)
// ------------------------------------------------------------------------------------------------
SectionPath SIPPSection::updatePath(const SIPPSectionNode* goal, ReservationSection& rs, MapSystem* MapSys)
{
    SectionPath path;
    path_cost = goal->getFVal();
    num_of_conf = goal->conflicts;

    // Start 노드부터 Goal 노드까지 순서대로 정렬하기 위해 리스트 수집
    std::vector<const SIPPSectionNode*> nodes;
    const SIPPSectionNode* curr = goal;
    while (curr != nullptr) {
        nodes.push_back(curr);
        curr = curr->parent;
    }
    std::reverse(nodes.begin(), nodes.end());

    // 앞에서부터 순차적으로 시뮬레이션하며 경로 복원
    for (size_t i = 0; i < nodes.size(); ++i) {
        SectionState state_copy = nodes[i]->s_state;

        if (i < nodes.size() - 1) {
            const SIPPSectionNode* next_node = nodes[i+1];
            state_copy.exit_index = next_node->parent_exit_index;

            std::vector<int> temp_wait;
            std::vector<pair<int,int>> temp_path;
            int dummy_next_sec = next_node->s_state.section_id;
            int dummy_next_start = next_node->s_state.start_index;
            int circle_flag = -1;

            // 1. 내부 이동 경로 재계산 (동일한 환경이므로 탐색 때와 정확히 같은 결과 반환)
            find_wait_list(state_copy.section_id, state_copy.start_index, state_copy.exit_index, 
                           state_copy.timestep, rs, MapSys, dummy_next_sec, dummy_next_start, 
                           temp_wait, temp_path, circle_flag, true);

            // 2. 출구(Exit) 또는 목적지(Goal)에서 추가로 대기했던 시간 복원
            int total_extra_wait = next_node->wait_at_exit + next_node->wait_at_goal;
            for (int w = 0; w < total_extra_wait; ++w) {
                temp_wait.push_back(state_copy.exit_index);
                int last_time = temp_path.empty() ? state_copy.timestep : temp_path.back().first;
                temp_path.push_back({last_time + 1, state_copy.exit_index});
            }

            state_copy.wait_list = temp_wait;
            state_copy.full_path = temp_path;
        } else {
            // 마지막 Goal 노드
            state_copy.exit_index = state_copy.goal_index;
        }

        path.push_back(state_copy);
    }

    return path;
}

// ------------------------------------------------------------------------------------------------
// [3] 진짜 SIPP 알고리즘 본체
// ------------------------------------------------------------------------------------------------
SectionPath SIPPSection::run_section(const SectionState& start_state, 
                                     const vector<pair<SectionState, int>>& goal_sections, 
                                     ReservationSection& rs, 
                                     int agent_id, int capacity, MapSystem* MapSys)
{
    num_expanded = 0; num_generated = 0; runtime = 0;

    std::vector<double> goal_to_goal = MapSys->goal_to_goal_dist(goal_sections);
    int start_section_id = start_state.section_id;
    double h_val = MapSys->compute_h_value(start_section_id, start_state.start_index, 0, goal_sections, goal_to_goal); 

    // 1. 출발지의 Safe Interval 찾기
    SecInterval start_interval = std::make_tuple(0, 0, 0);
    bool start_found = false;
    const auto& safe_intervals = rs.get_safe_intervals(start_section_id);
    for (auto const& interval : safe_intervals) {
        int section_congestion = std::get<2>(interval);
        if (section_congestion >= MapSys->sections_by_id[start_section_id]->info->capacity) continue; 

        if (std::get<0>(interval) <= start_state.timestep && std::get<1>(interval) > start_state.timestep) {
            start_interval = interval;
            start_found = true;
            break;
        }
    }

    if (!start_found) return SectionPath();

    // 2. 시작 노드 생성 (wait 변수 0으로 초기화)
    auto start_node = new SIPPSectionNode(start_state, 0, h_val, start_interval, nullptr, 0, -1, 0, 0);
    num_generated++;
    start_node->open_handle = open_list.push(start_node);
    start_node->in_openlist = true;
    allNodes_table.insert(start_node);
    min_f_val = start_node->getFVal();
    focal_bound = min_f_val * suboptimal_bound;
    start_node->focal_handle = focal_list.push(start_node);

    int max_goal_reached = 0;
    int max_timestep_reached = 0;

    // Local 계산을 위한 임시 캐시 (노드에 저장되지 않음!)
    std::vector<int> cached_wait_list;
    cached_wait_list.reserve(100);
    std::vector<pair<int, int>> cached_full_path;
    cached_full_path.reserve(100);

    // 3. 메인 탐색 루프
    while (!focal_list.empty())
    {
        SIPPSectionNode* curr = focal_list.top(); 
        focal_list.pop();
        open_list.erase(curr->open_handle);
        curr->in_openlist = false;
        num_expanded++;

        // 4. 다중 목적지(Multi-goal) 로직
        const SectionState& current_goal_state = goal_sections[curr->goal_id].first;
        int current_goal_time = goal_sections[curr->goal_id].second;
        int goal_section_id = current_goal_state.section_id;
        int goal_index = current_goal_state.goal_index; 

        if (curr->s_state.section_id == goal_section_id) 
        {
            int circle_flag = -1;
            int dist_to_goal = find_wait_list(curr->s_state.section_id, curr->s_state.start_index, goal_index, curr->s_state.timestep, rs, MapSys, goal_section_id, goal_index, cached_wait_list, cached_full_path, circle_flag);
            
            bool can_expand = true;

            if (dist_to_goal != -1) 
            {
                int arrive_time_at_goal = curr->s_state.timestep + dist_to_goal;
                int wait_time_at_goal = 0;

                // 목표 시간보다 일찍 도착 시 대기 처리
                if (arrive_time_at_goal < current_goal_time) {
                    wait_time_at_goal = current_goal_time - arrive_time_at_goal;
                    for(int i = 0; i < wait_time_at_goal; ++i) {
                        if(!rs.is_cell_safe(arrive_time_at_goal + i + 1, goal_section_id, goal_index)){
                            can_expand = false;
                            break;
                        } 
                    }
                    arrive_time_at_goal = current_goal_time; 
                }

                if (can_expand){
                    int next_goal_id = curr->goal_id + 1;
                    dist_to_goal += wait_time_at_goal;

                    // 내부 경로의 prev_index 추출을 위해 캐시 참조
                    int prev_index = -1;
                    if (!cached_wait_list.empty() && cached_wait_list.back() == goal_index){
                        prev_index = goal_index;
                    } else {
                        const vector<int>& sp = MapSys->sections_by_id[curr->s_state.section_id]->info->path_table[curr->s_state.start_index][goal_index];
                        prev_index = (sp.size() >= 2) ? sp[sp.size() - 2] : sp[0];
                    }

                    // 🏁 4-A. 최종 목적지 도착 완료
                    if (next_goal_id == (int)goal_sections.size())
                    {
                        SectionState final_state(goal_section_id, goal_index, -1, curr->s_state.timestep + dist_to_goal);

                        // 생성자에 무거운 vector 대신 wait_time_at_goal만 전달
                        SIPPSectionNode final_node(final_state, curr->g_val + dist_to_goal, 0, 
                                            curr->interval, curr, curr->conflicts, prev_index, 0, wait_time_at_goal);
                        
                        // rs와 MapSys를 넘겨주어 경로 재조립
                        SectionPath path = updatePath(&final_node, rs, MapSys);
                        releaseClosedListNodes();
                        open_list.clear(); focal_list.clear();
                        return path;
                    }
                    // 🔄 4-B. 경유지(Waypoint) 도착
                    else 
                    {
                        SectionState waypoint_state(goal_section_id, goal_index, -1, curr->s_state.timestep + dist_to_goal);
                        double next_h_val = MapSys->compute_h_value(goal_section_id, goal_index, next_goal_id, goal_sections, goal_to_goal);
                        
                        // 생성자에 무거운 vector 대신 wait_time_at_goal만 전달
                        auto waypoint_node = new SIPPSectionNode(waypoint_state, curr->g_val + dist_to_goal, 
                                                        next_h_val, curr->interval, curr, curr->conflicts, prev_index, 0, wait_time_at_goal); 
                        waypoint_node->goal_id = next_goal_id;

                        waypoint_node->open_handle = open_list.push(waypoint_node);
                        waypoint_node->in_openlist = true;
                        num_generated++;
                        allNodes_table.insert(waypoint_node);
                    
                        if (waypoint_node->getFVal() <= focal_bound)
                            waypoint_node->focal_handle = focal_list.push(waypoint_node);

                        continue; 
                    }
                }
            }
        }

        // 5. 이웃 섹션 확장 로직
        const auto& neighbors_map = MapSys->sections_by_id[curr->s_state.section_id]->neighbors;
        for (const auto& [curr_exit_index, connections] : neighbors_map) 
        {
            for (const auto& port : connections) 
            {
                int next_section_id = port.target_sec->id;           
                int next_start_index = port.target_entry_idx;   
                int edge_cost = 1; 
                int circle_flag = -1; 

                int internal_cost = find_wait_list(curr->s_state.section_id, curr->s_state.start_index, curr_exit_index, curr->s_state.timestep, rs, MapSys, next_section_id, next_start_index, cached_wait_list, cached_full_path, circle_flag);

                if (internal_cost == -1) continue;
                
                double total_travel_cost = internal_cost + edge_cost;
                int arrival_time = curr->s_state.timestep + total_travel_cost;
                double next_h_val = MapSys->compute_h_value(next_section_id, next_start_index, curr->goal_id, goal_sections, goal_to_goal);

                const auto& safe_intervals = rs.get_safe_intervals(next_section_id);
                for (const auto& interval : safe_intervals)
                {
                    int section_congestion = std::get<2>(interval);
                    if (section_congestion >= MapSys->sections_by_id[start_section_id]->info->capacity) continue; 
                    if (std::get<1>(interval) <= arrival_time) continue;

                    // 파라미터에서 vector들 싹 다 뺌
                    generate_node(interval, curr, next_section_id, next_start_index, curr_exit_index, 
                                  rs, total_travel_cost, arrival_time, next_h_val, section_congestion);
                }
            }
        }

        // FOCAL 업데이트
        if (open_list.empty()) break; 
        
        SIPPSectionNode* open_head = open_list.top();
        if (open_head->getFVal() > min_f_val)
        {
            double new_min_f_val = open_head->getFVal();
            double new_focal_bound = new_min_f_val * suboptimal_bound;
            for (SIPPSectionNode* n : open_list) {
                if (n->getFVal() > focal_bound && n->getFVal() <= new_focal_bound)
                    n->focal_handle = focal_list.push(n);
            }
            min_f_val = new_min_f_val;
            focal_bound = new_focal_bound;
        }

        if (curr->goal_id > max_goal_reached) max_goal_reached = curr->goal_id;
        if (curr->s_state.timestep > max_timestep_reached) max_timestep_reached = curr->s_state.timestep;
    }

    releaseClosedListNodes();
    open_list.clear(); focal_list.clear();
    return SectionPath();
}

// ------------------------------------------------------------------------------------------------
// [4] 노드 생성 (vector 복사 로직 전면 제거!)
// ------------------------------------------------------------------------------------------------
void SIPPSection::generate_node(const SecInterval& interval, SIPPSectionNode* curr, 
                                int next_section_id, int next_start_index, int curr_exit_index,
                                ReservationSection& rs, double travel_cost, int arrival_time, double h_val, int section_congestion)
{
    int timestep = std::max(std::get<0>(interval), arrival_time);
    int extra_wait_time = timestep - arrival_time;

    // 대기 가능 여부만 가볍게 확인하고 끝! (배열 복사 없음)
    if (extra_wait_time > 0) {
        for(int i=0; i<extra_wait_time; i++){
            if (!rs.is_cell_safe(arrival_time+i, curr->s_state.section_id, curr_exit_index)){ 
                return; // 대기 불가시 조용히 폐기
            }
        }
    }
    
    double g_val = curr->g_val + travel_cost + extra_wait_time; 
    int next_conflicts = curr->conflicts + section_congestion;
    SectionState next_state(next_section_id, next_start_index, -1, timestep);

    SIPPSectionNode dummy_node(next_state, interval, curr->goal_id);
    auto it = allNodes_table.find(&dummy_node);

    if (it == allNodes_table.end())
    { 
        // 새 노드 생성시 extra_wait_time 값만 딱 전달
        auto next = new SIPPSectionNode(next_state, g_val, h_val, interval, curr, next_conflicts, curr_exit_index, extra_wait_time, 0);
        next->open_handle = open_list.push(next);
        next->in_openlist = true;
        num_generated++;
        if (next->getFVal() <= focal_bound)
            next->focal_handle = focal_list.push(next);
        allNodes_table.insert(next);
        return;
    }

    SIPPSectionNode* existing_next = *it;
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

            existing_next->s_state = next_state;
            existing_next->g_val = g_val;
            existing_next->h_val = h_val;
            existing_next->parent = curr;
            existing_next->depth = curr->depth+1;
            existing_next->conflicts = next_conflicts;
            existing_next->parent_exit_index = curr_exit_index; 
            existing_next->wait_at_exit = extra_wait_time; // 기록 갱신
            existing_next->wait_at_goal = 0;

            if (update_open) open_list.increase(existing_next->open_handle);  
            if (add_to_focal) existing_next->focal_handle = focal_list.push(existing_next);
            if (update_in_focal) focal_list.update(existing_next->focal_handle);  
        }
    }
    else
    {  
        if (existing_f_val > new_f_val || (existing_f_val == new_f_val && existing_next->conflicts > next_conflicts))
        {
            existing_next->s_state = next_state;
            existing_next->g_val = g_val;
            existing_next->h_val = h_val;
            existing_next->parent = curr;
            existing_next->depth = curr->depth+1;
            existing_next->conflicts = next_conflicts;
            existing_next->parent_exit_index = curr_exit_index;
            existing_next->wait_at_exit = extra_wait_time; // 기록 갱신
            existing_next->wait_at_goal = 0;
            
            existing_next->open_handle = open_list.push(existing_next);
            existing_next->in_openlist = true;
            if (existing_f_val <= focal_bound)
                existing_next->focal_handle = focal_list.push(existing_next);
        }
    }  
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

int SIPPSection::find_wait_list(int section_id, int start_index, int exit_index, int timestep, 
                                const ReservationSection& rs, MapSystem* MapSys, 
                                int& next_section_id, int& next_start_index, 
                                std::vector<int>& wait_list, std::vector<pair<int, int>>& full_path, 
                                int circle_flag, bool build_path) // ✨ 파라미터 추가
{
    if (build_path) {
        wait_list.clear();
        full_path.clear();
    }

    const vector<int>& static_path = MapSys->sections_by_id[section_id]->info->path_table[start_index][exit_index];
    if (static_path.empty() && start_index != exit_index) {
        if (build_path) wait_list.push_back(-1);
        return -1;
    } 

    int curr_time = timestep;
    int wait_count_total = 0; // ✨ 가볍게 대기 시간만 세는 변수

    for (size_t i = 0; i < static_path.size() - 1; ++i) {
        int curr_index = static_path[i];
        int next_index = static_path[i + 1];

        while (!rs.is_cell_safe(curr_time + 1, section_id, next_index)) {
            if (!rs.is_cell_safe(curr_time + 1, section_id, curr_index)) {
                if (build_path) { wait_list.clear(); wait_list.push_back(-1); }
                return -1;
            }
            if (build_path) wait_list.push_back(curr_index);
            wait_count_total++; // 카운트 증가
            curr_time++;
        }
        curr_time++;
    }

    if (section_id != next_section_id) {
        int exit_idx = static_path.back(); // 이름 충돌 방지
        auto sec_type = MapSys->sections_by_id[section_id]->info->type;
        bool is_special_section = (sec_type == SectionType::Eject_CCW || sec_type == SectionType::Eject_CW ||
                                   sec_type == SectionType::Induct_C_BOTTOM || sec_type == SectionType::Induct_C_TOP);
                                   
        while (!rs.is_cell_safe(curr_time + 1, next_section_id, next_start_index)) {
            if (!rs.is_cell_safe(curr_time + 1, section_id, exit_idx)) {
                if (is_special_section){
                    int loop_start_index = MapSys->sections_by_id[section_id]->info->adj[exit_idx].front();
                    if (rs.is_cell_safe(curr_time + 1, section_id, loop_start_index)) {
                        next_section_id = section_id;
                        next_start_index = loop_start_index;
                        break;
                    } else {
                        if (build_path) { wait_list.clear(); wait_list.push_back(-1); }
                        return -1;
                    }
                } else {
                    if (build_path) { wait_list.clear(); wait_list.push_back(-1); }
                    return -1;
                }
            }
            if (build_path) wait_list.push_back(exit_idx);
            wait_count_total++;
            curr_time++;
        }
    }

    // ✨ 진짜 경로를 만들어야 할 때(updatePath)만 이 무거운 로직 실행!
    if (build_path) {
        std::unordered_map<int, int> wait_counts;
        for(int node : wait_list) wait_counts[node]++;

        int current_time = timestep;
        for (int cell_idx : static_path) {
            full_path.push_back({current_time, cell_idx});
            if (wait_counts.find(cell_idx) != wait_counts.end()) {
                int duration = wait_counts[cell_idx];
                for(int k=0; k<duration; ++k) {
                    current_time++;
                    full_path.push_back({current_time, cell_idx});
                }
            }
            current_time++;
        }
    }

    return (int)(static_path.size() - 1) + wait_count_total;
}


    