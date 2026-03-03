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
// [2] Path 역추적 (기존 SIPP.cpp와 거의 동일 - SIPPSectionNode 타입만 매칭)
// ------------------------------------------------------------------------------------------------
SectionPath SIPPSection::updatePath(const SIPPSectionNode* goal)
{
    SectionPath path;
    path_cost = goal->getFVal();
    num_of_conf = goal->conflicts;

    const SIPPSectionNode* curr = goal;

    int child_exit_index = -1;
    std::vector<int> child_wait_list = {};
    std::vector<pair<int, int>> child_full_path = {};
    bool has_child = false;

    while (curr != nullptr)
    {
        SectionState state_copy = curr->s_state;
        // if has child, update exit index & wait list.
        if (has_child){
            state_copy.exit_index = child_exit_index;
            state_copy.wait_list = child_wait_list;
            state_copy.full_path = child_full_path;
        } else{
            // goal section -> exit index = goal_index
            state_copy.exit_index = state_copy.goal_index;
        }

        path.push_back(state_copy);

        if (curr->parent != nullptr){
            child_exit_index = curr -> parent_exit_index;
            child_wait_list = curr -> parent_wait_list;
            child_full_path = curr -> parent_full_path;
            has_child = true;
        }

        if (curr->parent == nullptr){
            int t = curr->s_state.timestep - 1;
            for (; t>=0; t--){
                std::cout << "start time error"  << std::endl;
                path.push_back(SectionState(-1, -1, -1, t));
            }
        }

        curr = curr->parent;
    }

    std::reverse(path.begin(), path.end());
    //std::cout << "first_path_section_index" << path.front().section_id << std::endl;

    return path;
}

// ------------------------------------------------------------------------------------------------
// [3] 진짜 SIPP 알고리즘 본체 (ReservationSection 연동)
// ------------------------------------------------------------------------------------------------
SectionPath SIPPSection::run_section(const SectionState& start_state, 
                                     const vector<pair<SectionState, int>>& goal_sections, 
                                     ReservationSection& rs, 
                                     int agent_id, int capacity, MapSystem* MapSys)
{
    //std::cout << "-- Run Section Start --" << std::endl;
    num_expanded = 0; num_generated = 0; runtime = 0;
    //clock_t start_clock = std::clock();

    std::vector<double> goal_to_goal = MapSys->goal_to_goal_dist(goal_sections);
    // 시작 노드의 섹션 ID
    int start_section_id = start_state.section_id;

    // ✨ 수정 1: 초기 시작 지점의 h 계산 (MapSys 사용)

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

    // 2. 시작 노드 생성
    // std::cout << "-- Create Start Node --" << std::endl;
    auto start_node = new SIPPSectionNode(start_state, 0, h_val, start_interval, nullptr, 0, -1, {});
    num_generated++;
    start_node->open_handle = open_list.push(start_node);
    start_node->in_openlist = true;
    allNodes_table.insert(start_node);
    min_f_val = start_node->getFVal();
    focal_bound = min_f_val * suboptimal_bound;
    start_node->focal_handle = focal_list.push(start_node);

    int max_goal_reached = 0;
    int max_timestep_reached = 0;

    // 에이전트가 방문한 구역들을 저장할 Set
    //std::unordered_set<int> visited_sections;
    // 첫 번째 목적지의 Section ID
    int first_goal_section = goal_sections[0].first.section_id;

    std::vector<int> cached_wait_list;
    cached_wait_list.reserve(100);
    std::vector<pair<int, int>> cached_full_path;
    cached_full_path.reserve(100);

    // 3. 메인 탐색 루프
    while (!focal_list.empty())
    {
        // std::cout << "-- While --" << std::endl;
        SIPPSectionNode* curr = focal_list.top(); 
        //visited_sections.insert(curr->s_state.section_id); // 방문한 구역 기록
        focal_list.pop();
        open_list.erase(curr->open_handle);
        curr->in_openlist = false;
        num_expanded++;

        // 4. 다중 목적지(Multi-goal) 로직
        const SectionState& current_goal_state = goal_sections[curr->goal_id].first;
        int current_goal_time = goal_sections[curr->goal_id].second;

        int goal_section_id = current_goal_state.section_id;
        int goal_index = current_goal_state.goal_index; // (이전 대화에서 맞춘 이름 반영)

        // 현재 노드가 목표 지점이 포함된 섹션에 들어왔다면?
        if (curr->s_state.section_id == goal_section_id) 
        {
            // O(1) 거리 계산 함수 호출
            // std::cout << "-- section_id == goal_section_id --" << std::endl;

            int dist_to_goal = find_wait_list(curr->s_state.section_id, curr->s_state.start_index, goal_index, curr->s_state.timestep, rs, MapSys, goal_section_id, goal_index, cached_wait_list, cached_full_path);
            
            // int dist_to_goal = -1;
            bool can_expand = true;

            // // wait_list가 존재한다면, wait_list update -> 존재하지 않는다면은 수정해야함 // wati_list = {-1}인 경우 node생성 실패를 내보내야함.
            // if (!cached_wait_list.empty() && cached_wait_list.front() == -1) {
            //     dist_to_goal = -1;
            // } else {
            //     // wait_list가 비어있거나({}), 정상적인 대기 목록이 있는 경우 모두 정상 계산!
            //     dist_to_goal = MapSys->get_distance(goal_section_id, curr->s_state.start_index, goal_index, cached_wait_list);
            // }

            //std::cout << "dist_to_goal" << dist_to_goal << std::endl;

            if (dist_to_goal != -1) 
            {
                //std::cout << "-- 153줄?? --" << std::endl;
                int arrive_time_at_goal = curr->s_state.timestep + dist_to_goal;
                int wait_time_at_goal = 0;

                // 목표 시간보다 일찍 도착 시 대기 처리
                if (arrive_time_at_goal < current_goal_time) {
                    wait_time_at_goal = current_goal_time - arrive_time_at_goal;
                    
                    for(int i = 0; i < wait_time_at_goal; ++i) {
                        if(!rs.is_cell_safe(arrive_time_at_goal + i + 1, goal_section_id, goal_index)){
                            std::cout << "Node can not expand (goal stay)" << std::endl;
                            can_expand = false;
                            break;
                        } else{
                            cached_wait_list.push_back(goal_index);
                            cached_full_path.push_back({arrive_time_at_goal + i, goal_index});
                        }
                    }
                    arrive_time_at_goal = current_goal_time; 
                }

                if (can_expand){
                    int next_goal_id = curr->goal_id + 1;
                    dist_to_goal += wait_time_at_goal;

                    // 🏁 4-A. 최종 목적지 도착 완료
                    if (next_goal_id == (int)goal_sections.size())
                    {
                        // std::cout << "-- Reach Final Node --" << std::endl;
                        SectionState final_state(goal_section_id, goal_index, -1, curr->s_state.timestep + dist_to_goal);

                        // auto internal_path = MapSys->sections_by_id[curr->s_state.section_id]->get_internal_path(curr->s_state.timestep, curr->s_state.start_index, goal_index, cached_wait_list);
                        int prev_index = -1;

                        if (!cached_wait_list.empty() && cached_wait_list.back() == goal_index){
                            prev_index = goal_index;
                            cached_full_path.pop_back();
                            cached_wait_list.pop_back();
                        } else{
                            const vector<int>& sp = MapSys->sections_by_id[curr->s_state.section_id]->info->path_table[curr->s_state.start_index][goal_index];
                            prev_index = (sp.size() >= 2) ? sp[sp.size() - 2] : sp[0];
                            cached_full_path.pop_back();
                        }


                        //int prev_index = (internal_path.size() - 2) ? internal_path[internal_path.size() - 2].second: internal_path[0].second;

                        SIPPSectionNode final_node(final_state, curr->g_val + dist_to_goal, 0, 
                                            curr->interval, curr, curr->conflicts, prev_index, cached_wait_list, cached_full_path);
                        // std::cout << "-- Create Final Node --" << goal_section_id << std::endl;
                        // curr->s_state.wait_list = wait_list;
                        // wati_list = {-1}인 경우 node생성 실패를 내보내야함.
                        SectionPath path = updatePath(&final_node);
                        releaseClosedListNodes();
                        open_list.clear(); focal_list.clear();
                        //runtime = (std::clock() - start_clock) * 1.0 / CLOCKS_PER_SEC;
                        //std::cout << "cost: " << path.back().timestep << ", ExpandNode: "<< num_expanded <<", GenerateNode: " << num_generated << ", RunTime: " << runtime << std::endl;

                        
                        return path;
                    }
                    // 🔄 4-B. 아직 경유지가 남은 경우
                    else 
                    {
                        // std::cout << "-- Reach Goal Node --" << std::endl;
                        SectionState waypoint_state(goal_section_id, goal_index, -1, curr->s_state.timestep + dist_to_goal);
                        
                        // ✨ 수정 2: 다음 목적지를 향한 경유지 노드의 h 계산
                        double next_h_val = MapSys->compute_h_value(goal_section_id, goal_index, next_goal_id, goal_sections, goal_to_goal);
                        
                        // auto internal_path = MapSys->sections_by_id[curr->s_state.section_id]->get_internal_path(curr->s_state.timestep, curr->s_state.start_index, goal_index, cached_wait_list);
                        int prev_index = -1;
                        if (!cached_wait_list.empty() && cached_wait_list.back() == goal_index) {
                            prev_index = goal_index;
                            cached_full_path.pop_back();
                            cached_wait_list.pop_back();
                        } else {
                            const vector<int>& sp = MapSys->sections_by_id[curr->s_state.section_id]->info->path_table[curr->s_state.start_index][goal_index];
                            prev_index = (sp.size() >= 2) ? sp[sp.size() - 2] : sp[0];
                            cached_full_path.pop_back();
                        }

                        //int prev_index = (internal_path.size() - 2) ? internal_path[internal_path.size() - 2].second: internal_path[0].second;

                        auto waypoint_node = new SIPPSectionNode(waypoint_state, curr->g_val + dist_to_goal, 
                                                        next_h_val, curr->interval, curr, curr->conflicts, prev_index, cached_wait_list, cached_full_path); // goal이 연결될때는 Internal_path의 -1 의 ㅑndex를 내뱉음 -> timestep이 겹치ㅣ 않도록
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

        // 5-1. map 순회 (Key: 현재 섹션의 출구(exit_idx), Value: 연결된 타겟 정보들)
        for (const auto& [curr_exit_index, connections] : neighbors_map) 
        {
            
            // 5-2. 출구까지 무사히 갈 수 있다면, 이제 이 출구 밖으로 연결된 섹션들을 확인
            for (const auto& port : connections) 
            {
                int next_section_id = port.target_sec->id;           // 타겟 섹션 ID
                int next_start_index = port.target_entry_idx;   // 타겟 섹션의 진입 인덱스
                
                // 엣지 비용 (출구 밖으로 나가는 비용, 보통 1이거나 구조체 내부 값을 사용)
                int edge_cost = 1; // port.edge_cost 가 있다면 그것으로 교체

                
                int internal_cost = find_wait_list(curr->s_state.section_id, curr->s_state.start_index, curr_exit_index, curr->s_state.timestep, rs, MapSys, next_section_id, next_start_index, cached_wait_list, cached_full_path);

                // 만약 현재 위치에서 이 출구로 가는 길이 막혀있다면, 노드 확장 skip
                if (internal_cost == -1) continue;
                
                double total_travel_cost = internal_cost + edge_cost;
                int arrival_time = curr->s_state.timestep + total_travel_cost;

                // 다음 섹션으로 넘어갔을 때의 다음 h 계산
                double next_h_val = MapSys->compute_h_value(next_section_id, next_start_index, curr->goal_id, goal_sections, goal_to_goal);

                // 안전 구간(Safe Interval) 확인
                const auto& safe_intervals = rs.get_safe_intervals(next_section_id);
                //std::cout << "safe_intervals size "<< safe_intervals.size() << std::endl;
                for (const auto& interval : safe_intervals)
                {
                    int section_congestion = std::get<2>(interval);

                    if (section_congestion >= MapSys->sections_by_id[start_section_id]->info->capacity) continue; 

                    // 도착 시간보다 구간 종료 시간이 빠르거나 같으면 진입 불가 (이미 닫힌 문)
                    if (std::get<1>(interval) <= arrival_time) continue;

                    // generate_node 호출 
                    // (주의: wait_list가 필요하다면 빈 벡터 {} 대신 port.wait_list 등을 넘겨주세요)
                    generate_node(interval, curr, next_section_id, next_start_index, curr_exit_index, 
                                  cached_wait_list, cached_full_path, rs, total_travel_cost, arrival_time, next_h_val, section_congestion);
                }
            }
        }

        // FOCAL 리스트 및 최소 F값 업데이트 로직
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
    // std::cout << "🎯 가야 할 첫 번째 목적지(Section ID): " << first_goal_section << std::endl;
    // std::cout << "👣 탐색 중 방문해 본 구역들(Section IDs): ";
    // for (int sec : visited_sections) {
    //     std::cout << sec << " ";
    // }
    // std::cout << std::endl;
    // std::cout << "🔥 [탐색 실패] 한계점 확인 -> 최대 도달 목표 ID: " << max_goal_reached 
    //       << " / 최고 도달 시간(Timestep): " << max_timestep_reached << std::endl;
    // std::cout << "[탐색 실패!] 확장한 노드 수: " << num_expanded 
    //       << ", 생성한 노드 수: " << num_generated 
    //       << ", 마지막 목표 ID: " << goal_sections.size() << std::endl;
    return SectionPath();
}

// ------------------------------------------------------------------------------------------------
// [4] 노드 생성 (기존 로직과 동일, Interval -> SecInterval 매핑)
// ------------------------------------------------------------------------------------------------
void SIPPSection::generate_node(const SecInterval& interval, SIPPSectionNode* curr, 
                                int next_section_id, int next_start_index, int curr_exit_index,
                                const std::vector<int>& wait_list, const std::vector<pair<int, int>>& full_path, ReservationSection& rs,
                                double travel_cost, int arrival_time, double h_val, int section_congestion)
{
    // 1. 실제 도착(진입) 시간
    int timestep = std::max(std::get<0>(interval), arrival_time);
    
    // 2. 대기 시간 계산 (이전 섹션에서 얼마나 머무르다 넘어와야 하는가)
    int extra_wait_time = timestep - arrival_time;
    int plus_wait_time = 0;
    std::vector<int> final_wait_list; 
    std::vector<pair<int, int>> final_full_path; 
    const std::vector<int>* current_wait_list = &wait_list; // 기본은 원본을 가리킴
    const std::vector<pair<int, int>>* current_full_path = &full_path;

    if (extra_wait_time > 0) {
        final_wait_list = wait_list; // 이때만 어쩔 수 없이 복사
        final_full_path = full_path;
        current_wait_list = &final_wait_list; // 이제부터는 복사본을 가리킴
        current_full_path = &final_full_path;

        for(int i=0; i<extra_wait_time; i++){
            if (rs.is_cell_safe(arrival_time+i, curr->s_state.section_id, curr_exit_index)){ 
                final_wait_list.push_back(curr_exit_index);
                final_full_path.push_back({arrival_time + i, curr_exit_index});
                plus_wait_time += 1; 
            } else {
                return;
            }
        }
    }
    
    //int wait_time = current_wait_list->size();

    // 3. g_val 계산 (이동 시간 + 대기 시간) -> 시간적 비용만 철저히 관리
    double g_val = curr->g_val + travel_cost + plus_wait_time; 

    // 4. 혼잡도 누적 계산 (FOCAL 정렬용 Tie-breaker)
    int next_conflicts = curr->conflicts + section_congestion;

    // 5. SectionState 생성 (방금 도착했으므로 내 출구 exit_index는 아직 모름 -> -1)
    SectionState next_state(next_section_id, next_start_index, -1, timestep);

    // 6. 노드 생성 (부모의 출구 curr_exit_index를 내 노드에 기록해둠)
    SIPPSectionNode dummy_node(next_state, interval, curr->goal_id);
    //auto next = new SIPPSectionNode(next_state, g_val, h_val, interval, curr, next_conflicts, curr_exit_index, final_wait_list);

    auto it = allNodes_table.find(&dummy_node);
    // 완전히 새로운 노드 발견

    if (it == allNodes_table.end())
    {
        auto next = new SIPPSectionNode(next_state, g_val, h_val, interval, curr, next_conflicts, curr_exit_index, final_wait_list, final_full_path);
        next->open_handle = open_list.push(next);
        next->in_openlist = true;
        num_generated++;
        if (next->getFVal() <= focal_bound)
            next->focal_handle = focal_list.push(next);
        allNodes_table.insert(next);
        return;
    }

    // 기존 노드 업데이트 로직 (g_val + h_val 비교 후 conflicts 비교)
    SIPPSectionNode* existing_next = *it;
    double existing_f_val = existing_next->getFVal();
    double new_f_val = g_val + h_val;

    if (existing_next->in_openlist)
    {  
        // f_val이 줄어들었거나, f_val은 같은데 혼잡도가 줄어든 경우 update
        if (existing_f_val > new_f_val || (existing_f_val == new_f_val && existing_next->conflicts > next_conflicts))
        {
            bool add_to_focal = false;  
            bool update_in_focal = false;  
            bool update_open = false;
            
            // focal list update 조건
            if (new_f_val <= focal_bound) {  
                if (existing_f_val > focal_bound) add_to_focal = true;  // 이전에 focal이 비싸서 안들어간 경우 새로 추가
                else update_in_focal = true;  // 원래도 focal list에 존재 -> 정보 바뀜
            }

            // open list 갱신 조건 (f_val이 줄었을 때만 open tree 재정렬)
            if (existing_f_val > new_f_val) update_open = true;

            // 이전 노드의 정보를 더 좋은 정보로 갱신
            existing_next->s_state = next_state;
            existing_next->g_val = g_val;
            existing_next->h_val = h_val;
            existing_next->parent = curr;
            existing_next->depth = curr->depth+1;
            existing_next->conflicts = next_conflicts;
            existing_next->parent_exit_index = curr_exit_index; // 출구 정보도 업데이트
            existing_next->parent_wait_list = final_wait_list;
            existing_next->parent_full_path = final_full_path;

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
            existing_next->s_state = next_state;
            existing_next->g_val = g_val;
            existing_next->h_val = h_val;
            existing_next->parent = curr;
            existing_next->depth = curr->depth+1;
            existing_next->conflicts = next_conflicts;
            existing_next->parent_exit_index = curr_exit_index;
            existing_next->parent_wait_list = final_wait_list;
            existing_next->parent_full_path = final_full_path;
            
            existing_next->open_handle = open_list.push(existing_next);
            existing_next->in_openlist = true;
            if (existing_f_val <= focal_bound)
                existing_next->focal_handle = focal_list.push(existing_next);
        }
    }  

    // delete(next); 
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

int SIPPSection::find_wait_list(int section_id, int start_index, int exit_index, int timestep, const ReservationSection& rs, MapSystem* MapSys, int next_section_id, int next_start_index, std::vector<int>& wait_list, std::vector<pair<int, int>>& full_path){
    // initialization -> 그냥 open_list를 만드는건가? 근데 왜 SIPP의 list로 만들지?

    wait_list.clear();
    full_path.clear();

    // std::cout << "initial_paths empty"  << start_index << exit_index << std::endl;
    const vector<int>& static_path = MapSys->sections_by_id[section_id]->info->path_table[start_index][exit_index];
    if (static_path.empty() && start_index != exit_index) {
        wait_list.push_back(-1);
        return -1;
    } 

    int curr_time = timestep;

    for (size_t i = 0; i < static_path.size() - 1; ++i) {
        int curr_index = static_path[i];
        int next_index = static_path[i + 1];

        while (!rs.is_cell_safe(curr_time + 1, section_id, next_index)) {
            if (!rs.is_cell_safe(curr_time + 1, section_id, curr_index)) {
                wait_list.clear();
                wait_list.push_back(-1);
                return -1;
            }
            wait_list.push_back(curr_index);
            curr_time++;
        }
        curr_time++;
    }

    if (section_id != next_section_id) {
        int exit_index = static_path.back();
        while (!rs.is_cell_safe(curr_time + 1, next_section_id, next_start_index)) {
            if (!rs.is_cell_safe(curr_time + 1, section_id, exit_index)) {
                wait_list.clear();
                wait_list.push_back(-1);
                return -1;
            }
            wait_list.push_back(exit_index);
            curr_time++;
        }
    }

    std::unordered_map<int, int> wait_counts;
    for(int node : wait_list) {
        wait_counts[node]++;
    }

    int current_time = timestep;

    for (int cell_idx : static_path) {
        
        // A. 이동: 해당 셀에 도착
        //std::cout << "full_path_update"  << cell_idx << std::endl;
        full_path.push_back({current_time, cell_idx});
        
        // B. 대기: 여기서 기다려야 하는지 확인
        if (wait_counts.find(cell_idx) != wait_counts.end()) {
            int duration = wait_counts[cell_idx];
            
            // 대기 횟수만큼 제자리 걸음 (시간만 증가, 위치는 그대로)
            for(int k=0; k<duration; ++k) {
                current_time++;
                //std::cout << "full_path_update"  << cell_idx << std::endl;
                full_path.push_back({current_time, cell_idx});
            }
        }
        
        // C. 다음 칸으로 가기 위해 시간 증가
        current_time++;
    }
    

    return (int)(static_path.size() - 1) + (int)wait_list.size();
    // std::cout << "wait_list empty? " << wait_list.empty()  << std::endl;

}


    