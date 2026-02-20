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

inline uint32_t ReservationSection::make_section_key(int time, int section_id) const {
    // (디버깅용) 범위 체크가 필요하다면 assert를 넣으세요
    // assert(cell_idx <= MAX_CELL && section_id <= MAX_SECTION && time <= MAX_TIME);

    return (static_cast<uint32_t>(time) << SHIFT_TIME) | 
        (static_cast<uint32_t>(section_id) << SHIFT_SECTION);
}

// 무조건 덮어씀
void ReservationSection::add_reservation(int agent_id, int start_time, int end_time, int section_id, int cell_idx) {
    for (int t = start_time; t < end_time; ++t) {
        uint32_t c_key = make_cell_key(t, section_id, cell_idx);
        cell_table[c_key] = agent_id; 
    }

    // 2. Section Timeline (SIPP용)
    // "이 시간(start)에 한 명 늘고, 이 시간(end)에 한 명 줄었다"
    section_timeline[section_id][start_time]++;
    section_timeline[section_id][end_time]--;

    if (section_timeline[section_id][end_time] == 0) section_timeline[section_id].erase(end_time);

    // 데이터가 변했으니 캐시 삭제
    sit_cache.erase(section_id);
}

void ReservationSection::remove_reservation(int agent_id, int start_time, int end_time, int section_id, int cell_idx) {
    // 1. Cell 점유 해제
    for (int t = start_time; t < end_time; ++t) {
        uint32_t c_key = make_cell_key(t, section_id, cell_idx);
        auto it = cell_table.find(c_key);
        if (it != cell_table.end() && it->second == agent_id) {
            cell_table.erase(it);
        }
    
    }
    
    section_timeline[section_id][start_time]--;
    section_timeline[section_id][end_time]++;

    if (section_timeline[section_id][start_time] == 0) section_timeline[section_id].erase(start_time);

    sit_cache.erase(section_id);
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



void ReservationSection::add_section_constraint(int time, int section_id, int agent_id) {
    uint32_t key = make_section_key(time, section_id);
    section_constraints[key].push_back(agent_id); // 기존 제약에 합치기 (OR)
}

// 제약 해제
void ReservationSection::remove_section_constraint(int time, int section_id, int agent_id) {
    uint32_t key = make_section_key(time, section_id);

    auto it = section_constraints.find(key);
    if (it != section_constraints.end()) {
        std::vector<int>& agents = it->second;

        agents.erase(std::remove(agents.begin(), agents.end(), agent_id), agents.end());

        if (agents.empty()) {
            section_constraints.erase(it);
        }
    }
}

bool ReservationSection::is_safe(int time, int section_id, int pre_section_id, int cell_idx, int my_id, 
            int capacity, const PriorityGraph* pg) const 
{
        
    uint32_t s_key = make_section_key(time, section_id);

    // [Check 1] Section Constraints (환경/CBS 제약)
    auto s_it = section_constraints.find(s_key);
    if (s_it != section_constraints.end()) {
        const std::vector<int>& banned_agents = s_it->second;
        for (int cons : banned_agents){
            if(cons == my_id){
                return false;
            }
        }
    }

    /* // [Check 2] Section 진입 가능 여부 (Section에 들어갈때만 사용)
    if (pre_section_id != section_id) { 
        auto s_it = section_count_table.find(s_key);
        if (s_it != section_count_table.end() && s_it->second >= capacity) {
            // 자리가 없으면 진입 불가 (Entry Gate Block)
            return false; 
        }
    } */
    // [Check 3] Cell Conflict (PBS 우선순위)
    uint32_t c_key = make_cell_key(time, section_id, cell_idx);
    auto c_it = cell_table.find(c_key);
    
    if (c_it != cell_table.end()) {
        int other_id = c_it->second;
        
        // 나 자신이면 OK
        if (other_id == my_id) return true;

        // 상대가 나보다 형님(우선순위 높음)이면 비켜야 함
        /* if (pg != nullptr && pg->is_higher_priority(other_id, my_id)) {
            return false;
        } */
        
        // 상대가 나보다 동생이면? -> 내가 밀고 들어감 (True)
    }

    return true; // 모든 관문 통과
}

/* int ReservationSection::get_congestion_count(int time, int section_id) const {

    uint32_t s_key = make_section_key(time, section_id);
    
    auto it = section_count_table.find(s_key);
    if (it != section_count_table.end()) {
        return it->second;
    }
    return 0;
}
 */
std::list<SecInterval> ReservationSection::get_safe_intervals(int section_id, int capacity){
    // 캐시 확인 (이미 계산된 적이 있다면 바로 반환)
    if (sit_cache.find(section_id) != sit_cache.end()){
        return sit_cache[section_id];
    }

    std::list<SecInterval> all_intervals;
    auto& timeline = section_timeline[section_id]; // timeline[time]=capacity

    int current_occupancy = 0;
    int last_time = 0;

    // Timeline Sweep (시간 순서대로 훑기)
    for (auto const& [event_time, delta]: timeline){
        // [이전 시점, 현재 이벤트 시점) 구간의 상태 기록
        // SIPP는 이 구간의 occupancy가 capacity보다 낮을 때만 '진짜 Safe'로 간주하지만,
        // 나중에 가중치 계산(혼잡도)을 위해 일단 모든 구간을 생성합니다.

        if (event_time > last_time){
            if (current_occupancy < capacity){
                all_intervals.emplace_back(last_time, event_time, current_occupancy);
            }
        }

        current_occupancy += delta;
        last_time = event_time;

        if (last_time >= MAX_TIME_LIMIT) break;
    }

    // after last event, ~ MAX 처리
    if (last_time < MAX_TIME_LIMIT && current_occupancy < capacity){
        all_intervals.emplace_back(last_time, MAX_TIME_LIMIT, current_occupancy);
    }

    // 인원수가 똑같은 연속된 구간 병합
    if (!all_intervals.empty()){
        auto it = all_intervals.begin();
        while (std::next(it) != all_intervals.end()){
            auto next_it = std::next(it);
            if (std::get<1>(*it) == std::get<0>(*next_it) && std::get<2>(*it) == std::get<2>(*next_it)){
                std::get<1>(*it) = std::get<1>(*next_it);
                all_intervals.erase(next_it);
            } else{
                ++it;
            }
        }
    }

    sit_cache[section_id] = all_intervals;
    return all_intervals;
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