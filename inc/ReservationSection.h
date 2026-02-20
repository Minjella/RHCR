#pragma once
#include <unordered_map>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <iostream>
#include <list>
#include <tuple>
#include <map>
#include "PriorityGraph.h"

// // <start_time, end_time, current_capacity>
// typedef std::tuple<int, int, int> SecInterval;

static constexpr int MAX_TIME_LIMIT = (1 << 14) -1;

class ReservationSection {
    private:
        // ---------------------------------------------------------
        // 1. 32-bit Packing Logic
        // Layout: [ Time (14bit) | SectionID (14bit) | CellIdx (4bit) ]
        // ---------------------------------------------------------
        
        // 각 데이터가 차지할 비트 수
        static constexpr int BITS_CELL    = 4;  // Max 15
        static constexpr int BITS_SECTION = 14; // Max 16383
        // Time은 나머지 14비트 사용 (Max 16383)

        // 시프트 위치 계산
        static constexpr int SHIFT_SECTION = BITS_CELL;             // 4
        static constexpr int SHIFT_TIME    = BITS_CELL + BITS_SECTION; // 4 + 14 = 18

        // 마스크 (유효성 검사용, 선택 사항)
        static constexpr int MAX_CELL    = (1 << BITS_CELL) - 1;
        static constexpr int MAX_SECTION = (1 << BITS_SECTION) - 1;
        static constexpr int MAX_TIME    = (1 << (32 - SHIFT_TIME)) - 1;

        // 키 생성 함수 (32비트 반환)
        inline uint32_t make_cell_key(int time, int section_id, int cell_idx) const;
        inline uint32_t make_section_key(int time, int section_id) const;

        // ---------------------------------------------------------
        // 2. Data Structures
        // ---------------------------------------------------------
        
        // Key: Combined(Time, Section, Cell) -> Value: AgentID
        std::unordered_map<uint32_t, int> cell_table; 
        
        // SectionID -> { Time -> CountDelta }
        std::unordered_map<int, std::map<int, int>> section_timeline;

        // SIT (Safe Interval Table) SectionID -> Value
        std::unordered_map<int, std::list<SecInterval>> sit_cache;

        // Key: SectionKey (Time | Section) -> Value: Agent ID vector
        // CBS에서 제약을 걸거나, 맵의 문이 닫히는 경우 사용
        std::unordered_map<uint32_t, std::vector<int>> section_constraints;


    public:
        ReservationSection(){
            cell_table.reserve(10000);
            // section_count_table.reserve(5000);
        }

        void add_reservation(int agent_id, int start_time, int end_time, int section_id, int cell_idx);

        void remove_reservation(int agent_id, int start_time, int end_time, int section_id, int cell_idx);

        // 충돌 체크 (PBS 우선순위 로직 포함)
        // True: 지나갈 수 있음, False: 막힘
        //bool is_cell_available(int time, int section_id, int cell_idx, int my_agent_id, const PriorityGraph* pg) const;

        // 섹션 혼잡도 체크 (Capacity Constraint)
        //bool is_section_capacity_okay(int time, int section_id, int capacity) const;

        void add_section_constraint(int time, int section_id, int agent_id);

        // 제약 해제
        void remove_section_constraint(int time, int section_id, int agent_id);

        bool is_safe(int time, int section_id, int pre_section_id, int cell_idx, int my_id, int capacity, const PriorityGraph* pg) const ;

        int get_congestion_count(int time, int section_id) const;

        // void update_sit(int section_id, int capacity);

        // void merge_intervals(std::list<SecInterval>& intervals) const;

        std::list<SecInterval> get_safe_intervals(int section_id, int capacity);
        
};
