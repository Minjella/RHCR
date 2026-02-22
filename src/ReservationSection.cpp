#include "ReservationSection.h"
#include <unordered_map>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <iostream>
#include "PriorityGraph.h"

inline uint32_t ReservationSection::make_cell_key(int time, int section_id, int cell_idx) const {
    // (디버깅용) 범위 체크가 필요하다면 assert를 넣으세요
    // assert(cell_idx <= MAX_CELL && section_id <= MAX_SECTION && time <= MAX_TIME);

    return (static_cast<uint32_t>(time) << SHIFT_TIME) | 
        (static_cast<uint32_t>(section_id) << SHIFT_SECTION) | 
        (static_cast<uint32_t>(cell_idx));
}

// inline uint32_t ReservationSection::make_section_key(int time, int section_id) const {
//     // (디버깅용) 범위 체크가 필요하다면 assert를 넣으세요
//     // assert(cell_idx <= MAX_CELL && section_id <= MAX_SECTION && time <= MAX_TIME);

//     return (static_cast<uint32_t>(time) << SHIFT_TIME) | 
//         (static_cast<uint32_t>(section_id) << SHIFT_SECTION);
// }

// 무조건 덮어씀
// void ReservationSection::add_reservation(int agent_id, int start_time, int end_time, int section_id, int cell_idx) {
//     for (int t = start_time; t < end_time; ++t) {
//         uint32_t c_key = make_cell_key(t, section_id, cell_idx);
//         cell_table[c_key] = agent_id; 
//     }

//     // 2. Section Timeline (SIPP용)
//     // "이 시간(start)에 한 명 늘고, 이 시간(end)에 한 명 줄었다"
//     section_timeline[section_id][start_time]++;
//     section_timeline[section_id][end_time]--;

//     if (section_timeline[section_id][end_time] == 0) section_timeline[section_id].erase(end_time);

//     // 데이터가 변했으니 캐시 삭제
//     sit_cache.erase(section_id);
// }

// void ReservationSection::remove_reservation(int agent_id, int start_time, int end_time, int section_id, int cell_idx) {
//     // 1. Cell 점유 해제
//     for (int t = start_time; t < end_time; ++t) {
//         uint32_t c_key = make_cell_key(t, section_id, cell_idx);
//         auto it = cell_table.find(c_key);
//         if (it != cell_table.end() && it->second == agent_id) {
//             cell_table.erase(it);
//         }
    
//     }
    
//     section_timeline[section_id][start_time]--;
//     section_timeline[section_id][end_time]++;

//     if (section_timeline[section_id][start_time] == 0) section_timeline[section_id].erase(start_time);

//     sit_cache.erase(section_id);
// }

void ReservationSection::build(const std::vector<SectionPath*>& paths, const std::unordered_set<int>& high_priority_agents , MapSystem* MapSys)
{
    clear();

    for (int agent_id: high_priority_agents)
    {
        if (paths[agent_id] == nullptr || paths[agent_id]->empty()) continue;

        const auto& path = *paths[agent_id];

        int prev_section = -1;
        // int enter_time = -1;

        for (size_t i = 0; i < path.size(); ++i)
        {
            const auto&state = path[i];
            
            // list(time, cell index)
            vector<pair<int, int>> internal_paths = MapSys->sections_by_id[state.section_id]->get_internal_path(state.timestep, state.start_index, state.exit_index, state.wait_list);

            // Cell Table Update
            for (auto step: internal_paths){
                cell_table[make_cell_key(step.first, state.section_id, step.second)] = agent_id;
            }

            // Section Timeline update
            if (state.section_id != prev_section){
                // 이전 section에서 빠져나옴
                if (prev_section != -1){
                    section_timeline[prev_section][state.timestep] -= 1;
                    if (section_timeline[prev_section][state.timestep] == 0) {
                        section_timeline[prev_section].erase(state.timestep);
                    }
                }
                // new section 진입
                section_timeline[state.section_id][state.timestep] += 1;

                prev_section = state.section_id;
            }
        }

        // 마지막 위치에서 계속 대기
        if (prev_section != -1){
            section_timeline[prev_section][MAX_TIME_LIMIT] -= 1;
            if (section_timeline[prev_section][MAX_TIME_LIMIT] == 0) {
                section_timeline[prev_section].erase(MAX_TIME_LIMIT);
            }
        }
    }
}




















// void ReservationSection::update_sit(int section_id, int capacity){
//     if (sit_cache.find(section_id) != sit_cache.end()) return;

//     auto& timeline = section_timeline[section_id];
//     std::list<SecInterval> intervals;

//     int current_count = 0;
//     int last_time = 0;

//     // time: 변화 시점, delta: 인원수
//     for (auto const& [time, delta]: timeline){
//         // 새로운 이벤트 발생 전까지의 구간을 저장
//         // count < capacity 인 구간이 우리가 찾는 "Safe" 구간
//         intervals.emplace_back(last_time, time, current_count);

//         current_count += delta;
//         last_time = time;

//         intervals.emplace_back(last_time, MAX_TIME_LIMIT, current_count);

//         merge_intervals(intervals);

//         sit_cache[section_id] = intervals;
//     }
// }

/* 
bool ReservationSection::is_cell_available(int time, int section_id, int cell_idx, int my_agent_id, const PriorityGraph* pg) const {
    uint32_t key = make_cell_key(time, section_id, cell_idx);
    
    auto it = cell_table.find(key);
    if (it == cell_table.end()) {
        return true; // 아무도 없음 -> 통과 가능
    }

    int other_agent_id = it->second;
    
    // 내 자신이면 통과
    if (other_agent_id == my_agent_id) return true;

    // PBS 로직: 상대방이 존재하는데, 상대방 우선순위가 더 높으면 못 지나감
    // (PriorityGraph 구현에 따라 함수 이름은 다를 수 있음)
    if (pg->is_higher_priority(other_agent_id, my_agent_id)) {
        return false; // 형님이 계심 -> 통과 불가
    }

    // 상대방이 나보다 우선순위가 낮거나 관계가 없으면?
    // -> 우선순위가 낮은 애는 무시하고 지나감 (나중에 걔가 다시 길 찾음)
    return true; 
}
 */
/* 
bool ReservationSection::is_section_capacity_okay(int time, int section_id, int capacity) const {
    uint32_t key = make_section_key(time, section_id);
    auto it = section_count_table.find(key);
    
    if (it == section_count_table.end()) return true;
    return it->second < capacity;
} 
*/



// void ReservationSection::add_section_constraint(int time, int section_id, int agent_id) {
//     uint32_t key = make_section_key(time, section_id);
//     section_constraints[key].push_back(agent_id); // 기존 제약에 합치기 (OR)
// }

// // 제약 해제
// void ReservationSection::remove_section_constraint(int time, int section_id, int agent_id) {
//     uint32_t key = make_section_key(time, section_id);

//     auto it = section_constraints.find(key);
//     if (it != section_constraints.end()) {
//         std::vector<int>& agents = it->second;

//         agents.erase(std::remove(agents.begin(), agents.end(), agent_id), agents.end());

//         if (agents.empty()) {
//             section_constraints.erase(it);
//         }
//     }
// }

// bool ReservationSection::is_safe(int time, int section_id, int pre_section_id, int cell_idx, int my_id, 
//             int capacity, const PriorityGraph* pg) const 
// {
        
//     uint32_t s_key = make_section_key(time, section_id);

//     // [Check 1] Section Constraints (환경/CBS 제약)
//     auto s_it = section_constraints.find(s_key);
//     if (s_it != section_constraints.end()) {
//         const std::vector<int>& banned_agents = s_it->second;
//         for (int cons : banned_agents){
//             if(cons == my_id){
//                 return false;
//             }
//         }
//     }

//     /* // [Check 2] Section 진입 가능 여부 (Section에 들어갈때만 사용)
//     if (pre_section_id != section_id) { 
//         auto s_it = section_count_table.find(s_key);
//         if (s_it != section_count_table.end() && s_it->second >= capacity) {
//             // 자리가 없으면 진입 불가 (Entry Gate Block)
//             return false; 
//         }
//     } */
//     // [Check 3] Cell Conflict (PBS 우선순위)
//     uint32_t c_key = make_cell_key(time, section_id, cell_idx);
//     auto c_it = cell_table.find(c_key);
    
//     if (c_it != cell_table.end()) {
//         int other_id = c_it->second;
        
//         // 나 자신이면 OK
//         if (other_id == my_id) return true;

//         // 상대가 나보다 형님(우선순위 높음)이면 비켜야 함
//         /* if (pg != nullptr && pg->is_higher_priority(other_id, my_id)) {
//             return false;
//         } */
        
//         // 상대가 나보다 동생이면? -> 내가 밀고 들어감 (True)
//     }

//     return true; // 모든 관문 통과
// }

/* int ReservationSection::get_congestion_count(int time, int section_id) const {

    uint32_t s_key = make_section_key(time, section_id);
    
    auto it = section_count_table.find(s_key);
    if (it != section_count_table.end()) {
        return it->second;
    }
    return 0;
}
 */

void ReservationSection::update_sit(int section_id){
    auto& intervals = sit_cache[section_id];
    const auto& timeline = section_timeline[section_id];

    // 야무도 예약 안했으면 전체 시간이 Safe Interval
    if (timeline.empty()){
        intervals.emplace_back(0, MAX_TIME_LIMIT, 0);
        return;
    }

    int current_agents = 0;
    int last_time = 0; // start time

    // timeline(std::map) -> 시간순 자동 정렬
    for (const auto& [time, delta]: timeline){

        // event time > time -> [last_time ~ time) -> current_agents 명
        if (time > last_time){
            intervals.emplace_back(last_time, time, current_agents);
        }

        current_agents += delta;
        if (current_agents < 0){
            std::cout << "<<error>> current_agents < 0 !!!" << std::endl;
        }
        
        last_time = time;
    }
    // 마지막 시간 이후로 안전하다면, 안전구간 생성
    if (last_time < MAX_TIME_LIMIT){
        intervals.emplace_back(last_time, MAX_TIME_LIMIT, current_agents);
    }
}







std::list<SecInterval> ReservationSection::get_safe_intervals(int section_id, int capacity){
    // 캐시가 없다면 우선 전체 혼잡도 프로필을 만듦
    if (sit_cache.find(section_id) == sit_cache.end()){
        update_sit(section_id);
    }

    std::list<SecInterval> safe_intervals;

    // 전체 구간 중, 현재 인원 < capacity 인 구간만 뽑아서 반환
    for (const auto& interval: sit_cache[section_id]){
        int agents_in_section = std::get<2>(interval);
        
        if (agents_in_section < capacity){
            safe_intervals.push_back(interval);
        }
    }
    return safe_intervals;
}

bool ReservationSection::is_cell_safe(int time, int section_id, int cell_idx) const
{
    // High-level에서 이미 '나보다 우선순위 높은 애들'만 cell_table에 넣었음!
    // 따라서 map에 key가 존재하기만 하면 무조건 충돌(막힘)임.
    return cell_table.find(make_cell_key(time, section_id, cell_idx)) == cell_table.end();
}

// void ReservationSection::merge_intervals(std::list<SecInterval>& intervals) const {
//     if (intervals.empty()) return;
//     intervals.sort([](const SecInterval& a, const SecInterval& b){
//         return std::get<0>(a) < std::get<0>(b);
//     });

//     auto it = intervals.begin();
//     auto next = std::next(it);
//     while (next != intervals.end()) {
//         int a0 = std::get<0>(*it), a1 = std::get<1>(*it), a2 = std::get<2>(*it);
//         int b0 = std::get<0>(*next), b1 = std::get<1>(*next), b2 = std::get<2>(*next);

//         // 같은 “속성”(여기선 3번째 값)이면 인접/겹침 merge
//         if (a2 == b2 && b0 <= a1) {
//             std::get<1>(*it) = std::max(a1, b1);
//             next = intervals.erase(next);
//         } else {
//             ++it; ++next;
//         }
//     }
// }

int ReservationSection::get_congestion_count(int time, int section_id) const{
    auto it = sit_cache.find(section_id);

    if (it == sit_cache.end()) return 0;
    
    for (const auto& interval: it->second){
        if (time >= std::get<0>(interval) && time < std::get<1>(interval)){
            return std::get<2>(interval);
        }
    }

    return 0;
}