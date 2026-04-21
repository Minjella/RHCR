#include "SIPPSection.h"
#include "MapSystem.h"
#include <climits>
#include <ctime>

// [1] 더미 run 함수
Path SIPPSection::run(const BasicGraph& G, const State& start, const vector<pair<int, int>>& goal_location, ReservationTable& RT) {
    std::cerr << "[Error] SIPPSection은 ReservationTable을 사용하지 않습니다! 새로운 run을 호출하세요." << std::endl;
    exit(1);
    return Path();
}

// ------------------------------------------------------------------------------------------------
// [2] Path 역추적
// ------------------------------------------------------------------------------------------------
SectionPath SIPPSection::updatePath(const SIPPSectionNode* goal, ReservationSection& rs, MapSystem* MapSys)
{
    SectionPath path;
    path_cost = goal->getFVal();
    num_of_conf = goal->conflicts;

    std::vector<const SIPPSectionNode*> nodes;
    const SIPPSectionNode* curr = goal;
    while (curr != nullptr) {
        nodes.push_back(curr);
        curr = curr->parent;
    }
    std::reverse(nodes.begin(), nodes.end());

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

            find_wait_list(state_copy.section_id, state_copy.start_index, state_copy.exit_index,
                           state_copy.timestep, rs, MapSys, dummy_next_sec, dummy_next_start,
                           temp_wait, temp_path, circle_flag, true); // true: 경로 조립

            int total_extra_wait = next_node->wait_at_exit + next_node->wait_at_goal;
            for (int w = 0; w < total_extra_wait; ++w) {
                temp_wait.push_back(state_copy.exit_index);
                int last_time = temp_path.empty() ? state_copy.timestep : temp_path.back().first;
                temp_path.push_back({last_time + 1, state_copy.exit_index});
            }

            // Same-section transition (pivot) bridge: A*의 find_wait_list는 다음 섹션
            // 입구가 막혀 있을 때 exit 셀에서 기다리다 pivot을 시도하며, 그 대기만큼
            // cost에 포함시킨다. updatePath가 같은 섹션으로 재호출되면 end-while이
            // 스킵되어 이 대기가 temp_path에서 누락되고 paths[k]에 시간 구멍이 생긴다.
            // next_node.timestep - 1까지 exit 셀에서 대기한 것으로 보충한다.
            if (next_node->s_state.section_id == state_copy.section_id &&
                next_node->s_state.start_index != state_copy.start_index) {
                int target_last_time = next_node->s_state.timestep - 1;
                int current_last_time = temp_path.empty()
                                            ? state_copy.timestep - 1
                                            : temp_path.back().first;
                while (current_last_time < target_last_time) {
                    current_last_time++;
                    temp_path.push_back({current_last_time, state_copy.exit_index});
                    temp_wait.push_back(state_copy.exit_index);
                }
            }

            state_copy.wait_list = temp_wait;
            state_copy.full_path = temp_path;
        } else {
            state_copy.exit_index = state_copy.goal_index;
            // 최종 goal 셀에 대한 엔트리를 채워야 update_paths_section이
            // paths[k][timestep + final.timestep]을 실제로 쓴다. 비워두면
            // 그 인덱스가 default State(-1,-1,-1)로 남아 jump 검증에서 실패.
            if (state_copy.full_path.empty()) {
                state_copy.full_path.push_back({state_copy.timestep, state_copy.start_index});
            }
        }

        path.push_back(state_copy);
    }

    return path;
}

// ------------------------------------------------------------------------------------------------
// [3] SIPP 본체
// ------------------------------------------------------------------------------------------------
SectionPath SIPPSection::run_section(const SectionState& start_state, 
                                     const vector<pair<SectionState, int>>& goal_sections, 
                                     ReservationSection& rs, 
                                     int agent_id, int capacity, MapSystem* MapSys)
{
    num_expanded = 0; num_generated = 0; runtime = 0;
    // std::cout << "🔑 [SIPP 시작] allNodes_table.size()=" << allNodes_table.size() 
    //       << " pool_index=" << pool_index << "\n";
    assert(allNodes_table.empty()); // 이게 터지면 이전 탐색 정리가 안 된 것
    assert(pool_index == 0);
    std::vector<double> goal_to_goal = MapSys->goal_to_goal_dist(goal_sections);
    int start_section_id = start_state.section_id;

    // std::cout << "📞 [find_path] agent=" << agent_id 
    //       << " | start_section=" << start_section_id
    //       << " | start_index=" << start_state.start_index << "\n";
    double h_val = MapSys->compute_h_value(start_section_id, start_state.start_index, 0, goal_sections, goal_to_goal); 

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

    if (!start_found) {
        // std::cout << "[SIPP Fail] Agent " << agent_id << " - 시작 위치(" << start_state.section_id << ")에서 유효한 안전 구간(Safe Interval)을 찾지 못함!" << std::endl;
        return SectionPath();
    }
    // ✨ [변경] new 대신 Object Pool 사용
    auto start_node = allocate_node();
    start_node->s_state = start_state;
    start_node->g_val = 0;
    start_node->h_val = h_val;
    start_node->interval = start_interval;
    start_node->parent = nullptr;
    start_node->conflicts = 0;
    start_node->parent_exit_index = -1;
    start_node->wait_at_exit = 0;
    start_node->wait_at_goal = 0;
    start_node->depth = 0;
    start_node->goal_id = 0;

    num_generated++;
    start_node->open_handle = open_list.push(start_node);
    start_node->in_openlist = true;
    allNodes_table.insert(start_node);
    min_f_val = start_node->getFVal();
    focal_bound = min_f_val * suboptimal_bound;
    start_node->focal_handle = focal_list.push(start_node);

    int max_goal_reached = 0;
    int max_timestep_reached = 0;

    std::vector<int> cached_wait_list;
    cached_wait_list.reserve(100);
    std::vector<pair<int, int>> cached_full_path;
    cached_full_path.reserve(100);

    int iteration = 0;
    while (!focal_list.empty())
    {
        // if (++iteration > 10000) {
        //     std::cout << "🚨 [무한루프 의심] 10000회 초과, 강제 종료\n";
        //     break;
        // }
        SIPPSectionNode* curr = focal_list.top(); 
        focal_list.pop();
        // std::cout << "🔄 [확장] Section=" << curr->s_state.section_id 
        //   << " g=" << curr->g_val 
        //   << " h=" << curr->h_val
        //   << " f=" << curr->getFVal()
        //   << " focal_size=" << focal_list.size()
        //   << " open_size=" << open_list.size() << "\n";
        open_list.erase(curr->open_handle);
        curr->in_openlist = false;
        num_expanded++;

        const SectionState& current_goal_state = goal_sections[curr->goal_id].first;
        int current_goal_time = goal_sections[curr->goal_id].second;
        int goal_section_id = current_goal_state.section_id;
        int goal_index = current_goal_state.goal_index; 

        // if (curr->s_state.section_id == 3019) {
        //     std::cout << "🎯 [3019 goal 판정] start_index=" << curr->s_state.start_index 
        //             << " 3019 section type: " << (int)MapSys->sections_by_id[curr->s_state.section_id]->info->type
        //             << " goal_index=" << goal_index
        //             << " goal_section_id=" << goal_section_id
        //             << " curr_section=" << curr->s_state.section_id
        //             << " same_section=" << (curr->s_state.section_id == goal_section_id ? "YES" : "NO") << "\n";
        //     int cf = -1;
        //     std::vector<int> tmp_wait;
        //     std::vector<pair<int,int>> tmp_path;
        //     int dummy_sec = 3019, dummy_idx = goal_index;
        //     int dist = find_wait_list(3019, curr->s_state.start_index, goal_index,
        //                             curr->s_state.timestep, rs, MapSys,
        //                             dummy_sec, dummy_idx,
        //                             tmp_wait, tmp_path, cf);
        //     std::cout << "   dist_to_goal=" << dist << "\n";
        // }
        

        if (curr->s_state.section_id == goal_section_id) 
        {
            int circle_flag = -1;
            int dist_to_goal = find_wait_list(curr->s_state.section_id, curr->s_state.start_index, goal_index, curr->s_state.timestep, rs, MapSys, goal_section_id, goal_index, cached_wait_list, cached_full_path, circle_flag);
            
            bool can_expand = true;

            // if (curr->s_state.section_id == goal_section_id && goal_section_id == 3019) {
            //     std::cout << "🏁 [goal 판정 진입] dist=" << dist_to_goal 
            //             << " can_expand=" << can_expand
            //             << " arrive=" << (curr->s_state.timestep + dist_to_goal)
            //             << " current_goal_time=" << current_goal_time
            //             << " next_goal_id=" << (curr->goal_id + 1)
            //             << " total_goals=" << goal_sections.size() << "\n";
            // }

            if (dist_to_goal != -1) 
            {
                int arrive_time_at_goal = curr->s_state.timestep + dist_to_goal;
                int wait_time_at_goal = 0;

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

                    int prev_index = -1;
                    if (!cached_wait_list.empty() && cached_wait_list.back() == goal_index){
                        prev_index = goal_index;
                    } else {
                        const vector<int>& sp = MapSys->sections_by_id[curr->s_state.section_id]->info->path_table[curr->s_state.start_index][goal_index];
                        prev_index = (sp.size() >= 2) ? sp[sp.size() - 2] : sp[0];
                    }

                    if (next_goal_id == (int)goal_sections.size())
                    {
                        SectionState final_state(goal_section_id, goal_index, -1, curr->s_state.timestep + dist_to_goal);
                        
                        // 이건 Stack에 생성해서 넘기는 거라 그대로 둬도 안전합니다
                        SIPPSectionNode final_node(final_state, curr->g_val + dist_to_goal, 0, 
                                            curr->interval, curr, curr->conflicts, prev_index, 0, wait_time_at_goal);
                        
                        SectionPath path = updatePath(&final_node, rs, MapSys);

                        // std::cout << "🏁 [updatePath 결과] path.size()=" << path.size() << "\n";
                        // if (!path.empty())
                        //     std::cout << "   첫 섹션=" << path[0].section_id << " 마지막 섹션=" << path.back().section_id << "\n";
                        


                        releaseClosedListNodes();
                        open_list.clear(); focal_list.clear();
                        return path;
                    }
                    else 
                    {
                        SectionState waypoint_state(goal_section_id, goal_index, -1, curr->s_state.timestep + dist_to_goal);
                        double next_h_val = MapSys->compute_h_value(goal_section_id, goal_index, next_goal_id, goal_sections, goal_to_goal);
                        
                        // ✨ [변경] new 대신 Object Pool 사용
                        auto waypoint_node = allocate_node();
                        waypoint_node->s_state = waypoint_state;
                        waypoint_node->g_val = curr->g_val + dist_to_goal;
                        waypoint_node->h_val = next_h_val;
                        waypoint_node->interval = curr->interval;
                        waypoint_node->parent = curr;
                        waypoint_node->conflicts = curr->conflicts;
                        waypoint_node->parent_exit_index = prev_index;
                        waypoint_node->wait_at_exit = 0;
                        waypoint_node->wait_at_goal = wait_time_at_goal;
                        waypoint_node->depth = curr->depth + 1;
                        waypoint_node->goal_id = next_goal_id;

                        waypoint_node->open_handle = open_list.push(waypoint_node);
                        waypoint_node->in_openlist = true;
                        num_generated++;
                        allNodes_table.insert(waypoint_node);
                    
                        if (waypoint_node->getFVal() <= focal_bound)
                            waypoint_node->focal_handle = focal_list.push(waypoint_node);

                        if (focal_list.empty() && !open_list.empty()) {
                            SIPPSectionNode* head = open_list.top();
                            double new_min_f_val = head->getFVal();
                            double new_focal_bound = new_min_f_val * suboptimal_bound;
                            for (SIPPSectionNode* n : open_list) {
                                if (n->getFVal() <= new_focal_bound)
                                    n->focal_handle = focal_list.push(n);
                            }
                            min_f_val = new_min_f_val;
                            focal_bound = new_focal_bound;
                        }

                        continue; 
                    }
                }
            }
        }

        const auto& neighbors_map = MapSys->sections_by_id[curr->s_state.section_id]->neighbors;

        for (const auto& [curr_exit_index, connections] : neighbors_map) 
        {
            // ✨ [수사 로그] 3012번 섹션이 도대체 어느 출구들을 쳐다보고 있는지 모조리 출력합니다.
            // if (curr->s_state.section_id == 3012) {
            //     std::cout << "👀 [출구 스캔] Section 3012 (현재 Index: " << curr->s_state.start_index << ") -> [Exit " << curr_exit_index << "] 방향 검토 중...\n";
            // }


            for (const auto& port : connections) 
            {
                int next_section_id = port.target_sec->id;           
                int next_start_index = port.target_entry_idx;   
                int edge_cost = 1; 
                int circle_flag = -1; 

                int internal_cost = find_wait_list(curr->s_state.section_id, curr->s_state.start_index, curr_exit_index, curr->s_state.timestep, rs, MapSys, next_section_id, next_start_index, cached_wait_list, cached_full_path, circle_flag);

                if (internal_cost == -1) {
                    continue;
                }
                double total_travel_cost = internal_cost + edge_cost;

                int arrival_time = curr->s_state.timestep + total_travel_cost;
                double next_h_val = MapSys->compute_h_value(next_section_id, next_start_index, curr->goal_id, goal_sections, goal_to_goal);
                if (next_h_val >= 999999) { // 도달 불가 휴리스틱 가지치기
                    continue;
                }

                const auto& safe_intervals = rs.get_safe_intervals(next_section_id);
                if (safe_intervals.empty()) {
                    continue;
                }
                bool found_interval = false;
                for (const auto& interval : safe_intervals)
                {
                    
                    int section_congestion = std::get<2>(interval);
                    if (section_congestion >= MapSys->sections_by_id[next_section_id]->info->capacity){
                        continue;
                    }
                    if (std::get<1>(interval) <= arrival_time) {
                        found_interval = true;
                        continue;
                    }
                    // [수사 1] 내부 비용이 음수인지 확인
                    // if (curr->s_state.section_id == 3012) {
                    //     std::cout << "   🧪 [계산 검증] 현재시간: " << curr->s_state.timestep 
                    //             << ", 내부비용(internal_cost): " << internal_cost << "\n";
                    // }

                    int arrival_time = curr->s_state.timestep + internal_cost + 1;

                    // [수사 2] 시간 역행 여부
                    // if (arrival_time < curr->s_state.timestep) {
                    //     if (curr->s_state.section_id == 3012) {
                    //         std::cout << "   🚨 [시간 역행 발견!] 도착시간(" << arrival_time 
                    //                 << ")이 현재시간(" << curr->s_state.timestep << ")보다 과거입니다. find_wait_list를 점검하세요!\n";
                    //     }
                    //     continue; 
                    // }


                    

                    // if (!found_interval) {
                    //     if (curr->s_state.section_id == 3012) {
                    //         std::cout << "   ❌ [진입 불가] 섹션 " << next_section_id << " 정밀 분석 (도착예정: " << arrival_time << "):\n";
                    //         for (const auto& interval : safe_intervals) {
                    //             int s_start = std::get<0>(interval);
                    //             int s_end = std::get<1>(interval);
                    //             int s_occ = std::get<2>(interval);
                    //             int s_cap = MapSys->sections_by_id[next_section_id]->info->capacity;

                    //             bool cond1 = (s_end > arrival_time);
                    //             bool cond2 = (s_occ < s_cap);
                    //             bool cond3 = (arrival_time >= s_start); // SIPP의 기본 조건

                    //             std::cout << "      👉 구간 [" << s_start << "~" << s_end << " | 점유:" << s_occ << "/" << s_cap << "]\n";
                    //             std::cout << "         ㄴ 조건1(시간 여유 " << s_end << " > " << arrival_time << "): " << (cond1 ? "OK" : "FAIL") << "\n";
                    //             std::cout << "         ㄴ 조건2(공간 여유 " << s_occ << " < " << s_cap << "): " << (cond2 ? "OK" : "FAIL") << "\n";
                    //             std::cout << "         ㄴ 조건3(시간 역행 방지 " << arrival_time << " >= " << s_start << "): " << (cond3 ? "OK" : "FAIL") << "\n";
                                
                    //             if (arrival_time < curr->s_state.timestep) {
                    //                 std::cout << "         ⚠️ 경고: 현재 에이전트 시간(" << curr->s_state.timestep << ")보다 도착 예정 시간(" << arrival_time << ")이 과거입니다!\n";
                    //             }
                    //         }
                    //     }
                    // } else {
                    //     if (curr->s_state.section_id == 3012) std::cout << "   ✅ [성공 후보] " << next_section_id << "번 섹션으로 가는 노드 생성 시도 가능!\n";
                    // }

                    // if (next_section_id == 3013 || next_section_id == 2012) {
                    //     std::cout << "📤 [generate_node 호출 직전] Section " << curr->s_state.section_id 
                    //             << " -> " << next_section_id 
                    //             << " | arrival=" << arrival_time 
                    //             << " | h=" << next_h_val << "\n";
                    // }

                    generate_node(interval, curr, next_section_id, next_start_index, curr_exit_index, 
                                  rs, total_travel_cost, arrival_time, next_h_val, section_congestion);
                }
            }
        }

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
    // ✨ [새로 교체할 코드] 노드 메모리를 날리기 전에, 가장 멀리 갔던 노드를 찾아서 범행 현장을 낱낱이 출력합니다.
    int max_time = -1;
    int stuck_section = -1;
    int stuck_index = -1;

    for (SIPPSectionNode* n : allNodes_table) {
        if (n->s_state.timestep > max_time) {
            max_time = n->s_state.timestep;
            stuck_section = n->s_state.section_id;
            stuck_index = n->s_state.start_index;
        }
    }

    // std::cout << "\n======================================================\n";
    // std::cout << "🚨 [데드락 추적기] 에이전트 " << agent_id << " 길찾기 실패!\n";
    // std::cout << "📍 처음 출발한 곳: 섹션 " << start_state.section_id << " (Index: " << start_state.start_index << ", Time: " << start_state.timestep << ")\n";
    // if (stuck_section != -1) {
    //     std::cout << "💀 도저히 뚫지 못하고 막힌 곳: 섹션 " << stuck_section << " (Index: " << stuck_index << ", Time: " << max_time << ")\n";
    //     std::cout << "   -> 여기까지는 어떻게든 왔는데, 여기서 다음 칸으로 가는 모든 경우의 수가 막혔습니다!\n";
    // }
    // std::cout << "======================================================\n";



    releaseClosedListNodes();
    open_list.clear(); focal_list.clear();

    return SectionPath();
}

// ------------------------------------------------------------------------------------------------
// [4] 노드 생성 
// ------------------------------------------------------------------------------------------------
void SIPPSection::generate_node(const SecInterval& interval, SIPPSectionNode* curr, 
                                int next_section_id, int next_start_index, int curr_exit_index,
                                ReservationSection& rs, double travel_cost, int arrival_time, double h_val, int section_congestion)
{

    int timestep = std::max(std::get<0>(interval), arrival_time);



    int extra_wait_time = timestep - arrival_time;

    if (extra_wait_time > 0) {
        for(int i=0; i<extra_wait_time; i++){
            if (!rs.is_cell_safe(arrival_time+i, curr->s_state.section_id, curr_exit_index)){ 
                return;
            }
        }
    }
    
    double g_val = curr->g_val + travel_cost + extra_wait_time; 
    int next_conflicts = curr->conflicts + section_congestion;
    SectionState next_state(next_section_id, next_start_index, -1, timestep);
    SIPPSectionNode dummy_node(next_state, interval, curr->goal_id);

    auto it = allNodes_table.find(&dummy_node);
    // if (next_section_id == 3013 || next_section_id == 2012) {
    //     auto it = allNodes_table.find(&dummy_node);
    //     std::cout << "🔬 [중복 체크] Section " << next_section_id 
    //             << " | 이미 존재=" << (it != allNodes_table.end() ? "YES" : "NO");
    //     if (it != allNodes_table.end()) {
    //         std::cout << " | existing g=" << (*it)->g_val 
    //                 << " new g=" << g_val
    //                 << " | in_open=" << (*it)->in_openlist;
    //     }
    //     std::cout << "\n";
    // }

    if (it == allNodes_table.end())
    { 
        // ✨ [변경] new 대신 Object Pool 사용
        auto next = allocate_node();
        next->s_state = next_state;
        next->g_val = g_val;
        next->h_val = h_val;
        next->interval = interval;
        next->parent = curr;
        next->conflicts = next_conflicts;
        next->parent_exit_index = curr_exit_index;
        next->wait_at_exit = extra_wait_time;
        next->wait_at_goal = 0;
        next->depth = curr->depth + 1;
        next->goal_id = curr->goal_id;

        next->open_handle = open_list.push(next);
        next->in_openlist = true;
        num_generated++;
        if (next->getFVal() <= focal_bound)
            next->focal_handle = focal_list.push(next);

        // if (next_section_id == 3013 || next_section_id == 2012) {
        //     std::cout << "✅ [노드 생성] Section " << next_section_id 
        //             << " | f=" << next->getFVal() 
        //             << " | focal_bound=" << focal_bound
        //             << " | in_focal=" << (next->getFVal() <= focal_bound ? "YES" : "NO") << "\n";
        // }


        
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
            existing_next->wait_at_exit = extra_wait_time; 
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
            existing_next->wait_at_exit = extra_wait_time; 
            existing_next->wait_at_goal = 0;
            
            existing_next->open_handle = open_list.push(existing_next);
            existing_next->in_openlist = true;
            if (new_f_val <= focal_bound)
                existing_next->focal_handle = focal_list.push(existing_next);
        }
    }  
}

// ------------------------------------------------------------------------------------------------
// [5] 메모리 정리
// ------------------------------------------------------------------------------------------------
inline void SIPPSection::releaseClosedListNodes()
{
    // ✨ [변경] delete문을 싹 다 날려버리고 마법의 한 줄로 대체!
    allNodes_table.clear();
    pool_index = 0; // 이 한 줄로 O(N)이었던 노드 삭제 시간이 O(1)로 바뀝니다!
}



int SIPPSection::find_wait_list(int section_id, int start_index, int exit_index, int timestep,
                                const ReservationSection& rs, MapSystem* MapSys,
                                int& next_section_id, int& next_start_index,
                                std::vector<int>& wait_list, std::vector<pair<int, int>>& full_path,
                                int circle_flag, bool build_path)
{
    if (build_path) {
        wait_list.clear();
        full_path.clear();
    }

    // 한 번만 dereference — hot path에서 섹션 포인터 해시 조회 줄이기.
    auto* sec_info = MapSys->sections_by_id[section_id]->info;
    const vector<int>& static_path = sec_info->path_table[start_index][exit_index];
    if (static_path.empty()) {
        if (build_path) wait_list.push_back(-1);
        return -1;
    }

    const int path_len     = (int)static_path.size();
    const int exit_idx_cell = static_path.back();
    const int came_from    = (path_len >= 2) ? static_path[path_len - 2] : -1;
    const int original_next_section = next_section_id;
    const int original_next_start   = next_start_index;

    // max_delay: 시작 셀의 safe-interval horizon에서 유도 (cap 500).
    // get_safe_intervals는 lazy cache라 non-const → 읽기 전용 const_cast.
    int max_delay = 500;
    {
        auto& rs_mut = const_cast<ReservationSection&>(rs);
        const auto& sivs = rs_mut.get_safe_intervals(section_id);
        for (const auto& iv : sivs) {
            if (std::get<0>(iv) <= timestep && std::get<1>(iv) > timestep) {
                int horizon = std::get<1>(iv) - timestep;
                if (horizon < max_delay) max_delay = horizon;
                break;
            }
        }
        if (max_delay < 1) max_delay = 1;
    }

    // 빌드 모드일 때만 출력 벡터를 미리 reserve — 매 iteration 재할당 방지.
    if (build_path) {
        wait_list.reserve(path_len + 16);
        full_path.reserve(path_len + 16);
    }

    int best_delay       = -1;
    int wait_count_total = 0;
    int final_next_sec   = original_next_section;
    int final_next_start = original_next_start;

    for (int delay = 0; delay <= max_delay; ++delay) {
        // 시작점에서 delay만큼 대기할 수 있는지 검증.
        bool safe_to_delay = true;
        for (int w = 1; w <= delay; ++w) {
            if (!rs.is_cell_safe(timestep + w, section_id, start_index)) {
                safe_to_delay = false; break;
            }
        }
        if (!safe_to_delay) break;

        int temp_wait_total = delay;
        int temp_curr_time  = timestep + delay;
        bool local_fail     = false;
        int iter_next_sec   = original_next_section;
        int iter_next_start = original_next_start;

        // 빌드 모드: 직접 출력 벡터에 쓰기. 실패 시 아래에서 clear().
        if (build_path) {
            wait_list.clear();
            full_path.clear();
            full_path.push_back({timestep, start_index});
            for (int k = 1; k <= delay; ++k) {
                wait_list.push_back(start_index);
                full_path.push_back({timestep + k, start_index});
            }
        }

        // 섹션 내부 경로 순회.
        for (int i = 0; i < path_len - 1; ++i) {
            int curr_idx = static_path[i];
            int next_idx = static_path[i + 1];

            while (!rs.is_cell_safe(temp_curr_time + 1, section_id, next_idx)) {
                if (!rs.is_cell_safe(temp_curr_time + 1, section_id, curr_idx)) {
                    // 현재 칸도 막혀 있으면 이 경로는 포기 — A*가 다른 출구를
                    // 시도하게 둔다.
                    local_fail = true;
                    break;
                }
                // curr_idx에서 대기.
                temp_wait_total++;
                temp_curr_time++;
                if (build_path) {
                    wait_list.push_back(curr_idx);
                    full_path.push_back({temp_curr_time, curr_idx});
                }
            }
            if (local_fail) break;
            // curr_idx → next_idx 이동 (1 타임스텝).
            temp_curr_time++;
            if (build_path) {
                full_path.push_back({temp_curr_time, next_idx});
            }
        }

        if (local_fail) continue;

        // 목적지 입구 대기열 검증 — 다음 섹션 입구가 열릴 때까지 exit 셀에서 대기.
        // 막히면 같은 섹션 내 adj 후보로 pivot.
        if (section_id != iter_next_sec) {
            while (!rs.is_cell_safe(temp_curr_time + 1, iter_next_sec, iter_next_start)) {
                if (!rs.is_cell_safe(temp_curr_time + 1, section_id, exit_idx_cell)) {
                    bool loop_found = false;
                    const auto& adj_list = sec_info->adj[exit_idx_cell];
                    for (int cand : adj_list) {
                        if (cand == came_from) continue;
                        if (rs.is_cell_safe(temp_curr_time + 1, section_id, cand)) {
                            iter_next_sec   = section_id;
                            iter_next_start = cand;
                            loop_found = true;
                            break;
                        }
                    }
                    if (!loop_found) { local_fail = true; break; }
                    else             { break; }
                }
                temp_wait_total++;
                temp_curr_time++;
                if (build_path) {
                    wait_list.push_back(exit_idx_cell);
                    full_path.push_back({temp_curr_time, exit_idx_cell});
                }
            }
            if (local_fail) continue;
        }

        // 성공!
        best_delay       = delay;
        wait_count_total = temp_wait_total;
        final_next_sec   = iter_next_sec;
        final_next_start = iter_next_start;
        break;
    }

    if (best_delay == -1) {
        if (build_path) {
            wait_list.clear();
            full_path.clear();
            wait_list.push_back(-1);
        }
        return -1;
    }

    // A*에 넘겨줄 실제 목적지 (pivot되었을 수 있음).
    next_section_id  = final_next_sec;
    next_start_index = final_next_start;

    return (path_len - 1) + wait_count_total;
}

