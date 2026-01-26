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
void ReservationSection::add_reservation(int agent_id, int time, int section_id, int cell_idx) {
    uint32_t c_key = make_cell_key(time, section_id, cell_idx);
    auto [it, inserted] = cell_table.emplace(c_key, agent_id);
    if (!inserted){
        cell_table[c_key] = agent_id;
    }
    else {
        uint32_t s_key = make_section_key(time, section_id);
        section_count_table[s_key]++;
    }
}

void ReservationSection::remove_reservation(int agent_id, int time, int section_id, int cell_idx) {
    // 1. Cell 점유 해제
    uint32_t c_key = make_cell_key(time, section_id, cell_idx);
    
    auto it = cell_table.find(c_key);
    if (it != cell_table.end() && it->second == agent_id) {
        cell_table.erase(it);

        // 2. Section Count 감소
        uint32_t s_key = make_section_key(time, section_id);
        auto s_it = section_count_table.find(s_key);
        if (s_it != section_count_table.end()) {
            s_it->second--;
            if (s_it->second <= 0) {
                section_count_table.erase(s_it); // 0이면 메모리에서 삭제
            }
        }
    }
}

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

    // [Check 2] Section 진입 가능 여부 (Section에 들어갈때만 사용)
    if (pre_section_id != section_id) { 
        auto s_it = section_count_table.find(s_key);
        if (s_it != section_count_table.end() && s_it->second >= capacity) {
            // 자리가 없으면 진입 불가 (Entry Gate Block)
            return false; 
        }
    }
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

int ReservationSection::get_congestion_count(int time, int section_id) const {

    uint32_t s_key = make_section_key(time, section_id);
    
    auto it = section_count_table.find(s_key);
    if (it != section_count_table.end()) {
        return it->second;
    }
    return 0;
}