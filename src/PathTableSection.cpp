#include "PathTableSection.h"
#include <algorithm>
#include <memory>
#include <vector>
#include <list>
#include <numeric>


inline uint32_t PathTableSection::make_cell_key(int time, int section_id, int cell_idx) const {
    // (디버깅용) 범위 체크가 필요하다면 assert를 넣으세요
    // assert(cell_idx <= MAX_CELL && section_id <= MAX_SECTION && time <= MAX_TIME);

    return (static_cast<uint32_t>(time) << SHIFT_TIME) | 
        (static_cast<uint32_t>(section_id) << SHIFT_SECTION) | 
        (static_cast<uint32_t>(cell_idx));
}

PathTableSection::PathTableSection(const vector<SectionPath*>& paths, int window, int k_robust, MapSystem* MapSys):
    window(window), k_robust(k_robust)
{
    vector<pair<int, int>> internal_paths = {};
    internal_paths.reserve(100);

    num_of_agents = (int)paths.size();
    for (int i = 0; i < num_of_agents; i++)
    {
        for (auto state : (*paths[i]))
        {
            
            if (state.timestep > window){
                break;
            }


            // full_path로 교체 가능할듯
            // vector<pair<int, int>> internal_paths = MapSys->sections_by_id[state.section_id]->get_internal_path(state.timestep, state.start_index, state.exit_index, state.wait_list);
            internal_paths.clear();
            internal_paths = state.full_path;

            if (internal_paths.empty()){
                std::cout << "something wrong:: internal path is empty" << std::endl;
                continue;
            }

            section_timeline[state.section_id][state.timestep] += 1;

            if (section_timeline[state.section_id][state.timestep] == 0) {
                section_timeline[state.section_id].erase(state.timestep);
            }

            for (auto step: internal_paths){
                if (step.first <= window){
                    cell_table[make_cell_key(step.first, state.section_id, step.second)].push_back(i);
                }
            }
            
            if (internal_paths.back().first + 1 > window){
                break;
            }

            section_timeline[state.section_id][internal_paths.back().first + 1] -= 1;
            if (section_timeline[state.section_id][internal_paths.back().first + 1] == 0) {
                section_timeline[state.section_id].erase(internal_paths.back().first + 1);
            }
            
        }
    }
}


void PathTableSection::remove(const SectionPath* old_path, int agent, MapSystem* MapSys)
{
    if (old_path == nullptr || old_path->empty())
        return;

    vector<pair<int, int>> internal_paths = {};
    internal_paths.reserve(100);
    

    for (auto state : (*old_path))
    {
        if (state.timestep > window)
            break;
        
        // full_path로 대체 가능
        // vector<pair<int, int>> internal_paths = MapSys->sections_by_id[state.section_id]->get_internal_path(state.timestep, state.start_index, state.exit_index, state.wait_list);
        internal_paths.clear();
        internal_paths = state.full_path;

        if (internal_paths.empty()) continue;

        section_timeline[state.section_id][state.timestep] -= 1;

        if (section_timeline[state.section_id][state.timestep] == 0) {
            section_timeline[state.section_id].erase(state.timestep);
        }

        for (auto step : internal_paths)
        {
            if (step.first <= window)
            {
                uint32_t key = make_cell_key(step.first, state.section_id, step.second);
                
                // cell_table에 키가 존재하는지 확인
                auto it = cell_table.find(key);
                if (it != cell_table.end()) 
                {
                    auto& agent_list = it->second;
                    
                    // 벡터에서 해당 agent를 찾아 삭제합니다.
                    auto remove_it = std::find(agent_list.begin(), agent_list.end(), agent);
                    if (remove_it != agent_list.end()) {
                        agent_list.erase(remove_it);
                    }
                    
                    // 만약 이 타일에 더 이상 남은 로봇이 없다면 키를 통째로 삭제 (메모리 최적화)
                    if (agent_list.empty()) {
                        cell_table.erase(it);
                    }
                }
            }
        }

        int leave_time = internal_paths.back().first + 1;

        if (leave_time <= window + 1) { 
            section_timeline[state.section_id][leave_time] += 1;
        }

        if (section_timeline[state.section_id][leave_time] == 0) {
            section_timeline[state.section_id].erase(leave_time);
        }

        if (leave_time > window){
            break;
        }
        
    }
}


list<std::shared_ptr<SectionConflict> > PathTableSection::add(const SectionPath* new_path, int agent, MapSystem* MapSys)
{
    list<std::shared_ptr<SectionConflict> > conflicts;

    if (new_path == nullptr || new_path->empty()){
        std::cout << "something wrong:: new path is empty" << std::endl;
        return conflicts;
    }
    // 중복 충돌 방지
    vector<bool> conflicting_agents(num_of_agents, false);

    vector<pair<int, int>> internal_paths = {};
    internal_paths.reserve(100);

    for (auto state : (*new_path))
    {
        if (state.timestep > window) break;

        // full_path로 대체 가능
        //vector<pair<int, int>> internal_paths = MapSys->sections_by_id[state.section_id]->get_internal_path(state.timestep, state.start_index, state.exit_index, state.wait_list);
        internal_paths.clear();
        internal_paths = state.full_path;
        
        if (internal_paths.empty()) continue;

        // capcaity conflict
        int capacity = MapSys->sections_by_id[state.section_id]->info->capacity;
        int leave_time = internal_paths.back().first + 1;
        bool capacity_overflow = false;

        for (int t = state.timestep; t < leave_time; t++)
        {
            if (t > window) break;

            int current_occupancy = 0;
            for (auto const& [time_key, delta]: section_timeline[state.section_id]){
                if (time_key > t) break;
                current_occupancy += delta;
            }

            if (current_occupancy >= capacity){
                int agent2 = -1;
                auto curr_section = MapSys->sections_by_id[state.section_id];

                auto find_agent_in_cells = [&](const vector<int>& target_cells) -> int {
                    for (int cell_idx: target_cells){
                        uint32_t key = make_cell_key(t, state.section_id, cell_idx);
                        auto it = cell_table.find(key);

                        if (it != cell_table.end()){
                            for (int a: it -> second){
                                if (a != agent && !conflicting_agents[a]){
                                    return a;
                                }
                            }
                        }
                    }
                    return -1;
                };

                agent2 = find_agent_in_cells(curr_section->info->exits);

                if (agent2 == -1){
                    agent2 = find_agent_in_cells(curr_section->info->entries);
                }

                if (agent2 == -1) {
                    // exits, entries 외의 일반 타일들을 탐색하기 위해 전체 순회
                    int num_cells = 9;
                    vector<int> all_cells(num_cells);
                    iota(all_cells.begin(), all_cells.end(), 0); // 0, 1, 2 ... n-1 로 채움
                    
                    agent2 = find_agent_in_cells(all_cells);
                }

                if (agent2 != -1){
                    conflicts.push_back(std::shared_ptr<SectionConflict>(new SectionConflict(agent, agent2, state.section_id, -1, t, ConflictType::SECTION_CAP)));
                    conflicting_agents[agent2] = true;
                    capacity_overflow = true;
                }
            }
        }

        if (capacity_overflow){
            continue; // capacity가 넘쳤다면 section 내부 충돌 검사 x
        }

        for (auto step: internal_paths){
            if (step.first > window) break;

            uint32_t key = make_cell_key(step.first, state.section_id, step.second);
            auto it = cell_table.find(key);

            if (it != cell_table.end()){
                for (int other_agent: it->second){
                    if (other_agent != agent && !conflicting_agents[other_agent]){
                        conflicts.push_back(std::shared_ptr<SectionConflict>(new SectionConflict(agent, other_agent, state.section_id, step.second, step.first, ConflictType::TILE_VERTEX)));
                        conflicting_agents[other_agent] = true;
                    }
                }
            }
        }
    }

    // path table에 경로 추가
    for (auto state : (*new_path))
    {
        if (state.timestep > window) break;

        // full_path로 대체 가능
        //vector<pair<int, int>> internal_paths = MapSys->sections_by_id[state.section_id]->get_internal_path(state.timestep, state.start_index, state.exit_index, state.wait_list);
        internal_paths.clear();
        internal_paths = state.full_path;
        
        if (internal_paths.empty()) continue;

        // 섹션 진입 기록 (+1)
        section_timeline[state.section_id][state.timestep] += 1;

        // 타일 점유 기록 (agent 번호 추가)
        for (auto step : internal_paths) {
            if (step.first <= window) {
                cell_table[make_cell_key(step.first, state.section_id, step.second)].push_back(agent);
            }
        }
        
        // 섹션 진출 기록 (-1)
        int leave_time = internal_paths.back().first + 1;
        if (leave_time <= window + 1) { 
            section_timeline[state.section_id][leave_time] -= 1;
        }
    }

    return conflicts;
}