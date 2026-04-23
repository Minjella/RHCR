#pragma once
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <iostream>
#include <list>
#include <tuple>
#include <map>

#include <SectionState.h>
#include <MapSystem.h>

// <start_time, end_time, current_capacity>
typedef std::tuple<int, int, int> SecInterval;

static constexpr int MAX_TIME_LIMIT = (1 << 14) - 1;

class ReservationSection {
    private:
        // 32-bit packing: [ Time (14bit) | SectionID (14bit) | CellIdx (4bit) ]
        static constexpr int BITS_CELL    = 4;
        static constexpr int BITS_SECTION = 14;

        static constexpr int SHIFT_SECTION = BITS_CELL;
        static constexpr int SHIFT_TIME    = BITS_CELL + BITS_SECTION;

        static constexpr int MAX_CELL    = (1 << BITS_CELL) - 1;
        static constexpr int MAX_SECTION = (1 << BITS_SECTION) - 1;
        static constexpr int MAX_TIME    = (1 << (32 - SHIFT_TIME)) - 1;

        inline uint32_t make_cell_key(int time, int section_id, int cell_idx) const;

        // Bit-vector cell block storage (Grid-ECBS의 CAT vector<vector<bool>> 패턴).
        // cell_blocked_mask[time][section_id] = uint32_t, bit i = cell i blocked.
        // is_cell_safe는 hash lookup(~100ns) 대신 직접 배열 접근(~5ns)으로 20x 빠름.
        // Dirty tracking으로 clear() O(|constraints|) 유지.
        std::vector<std::vector<uint32_t>> cell_blocked_mask;
        std::vector<std::pair<int, int>> dirty_cells;  // (time, section) touched

        // SectionID -> { Time -> CountDelta } (capacity / congestion tracking).
        std::unordered_map<int, std::map<int, int>> section_timeline;

        // SIT cache: SectionID -> intervals.
        std::unordered_map<int, std::vector<SecInterval>> sit_cache;

        // CBS에서 제약을 걸거나 맵 문이 닫힐 때 사용 (예약됨, 현재 미사용).
        std::unordered_map<uint32_t, std::vector<int>> section_constraints;

        void update_sit(int section_id);

    public:
        ReservationSection() {
            cell_blocked_mask.reserve(32);  // typical time horizon
            dirty_cells.reserve(256);
        }

        void clear() {
            // Zero only the dirty (time, section) cells - O(|dirty|).
            for (auto& ts : dirty_cells) {
                if (ts.first < (int)cell_blocked_mask.size() &&
                    ts.second < (int)cell_blocked_mask[ts.first].size())
                    cell_blocked_mask[ts.first][ts.second] = 0;
            }
            dirty_cells.clear();
            section_timeline.clear();
            sit_cache.clear();
            section_constraints.clear();
        }

        // PBS용: 높은 우선순위 agent들의 경로를 cell_table + section_timeline에 반영.
        void build(const std::vector<SectionPath*>& paths,
                   const boost::unordered_set<int>& high_priority_agents,
                   MapSystem* MapSys);

        bool use_cat;
        bool prioritize_start;

        int get_congestion_count(int time, int section_id) const;

        const std::vector<SecInterval>& get_safe_intervals(int section_id);

        bool is_cell_safe(int time, int section_id, int cell_idx) const;

        // ECBS용 CT constraint: cell_table에 sentinel(-1)로 주입.
        // is_cell_safe가 map 존재 여부만 보므로 이 (time, section, cell)은 low-level에서 차단됨.
        // 해당 agent의 low-level search 직전에만 호출하고, clear() / init_empty()로 되돌린다.
        void add_cell_constraint(int time, int section_id, int cell_idx);

        // ECBSSection 강화용: 해당 time에 section 내 모든 cell을 차단한다.
        // 단일 진입 cell 차단(add_cell_constraint)만으로는 다른 진입 cell로 우회가
        // 쉬워 CT 수렴이 느리다. 섹션 전체를 막으면 agent는 해당 time에 이
        // section에 "존재" 자체가 불가능해 retiming 또는 다른 route 강제.
        // 9-cell section 기준으로 비트 0~8을 모두 세팅.
        void add_section_constraint(int time, int section_id);

        // 3x3 section의 4 corner entry cells (0, 2, 6, 8)만 차단.
        // add_section_constraint보다 덜 제약 (내부 cell 1,3,4,5,7엔 머물 수 있음)
        // 이지만 해당 time에 섹션 진입 경로를 전부 막아 retiming을 유도.
        void add_section_entry_constraint(int time, int section_id);

        // Positive-constraint 구현: section 안에서 cell_idx를 제외한 모든 cell을
        // 차단. SIPP 입장에서는 여전히 negative (cell_blocked_mask 기반) 이지만,
        // 의미적으로 "agent가 해당 time에 이 section에 있으면 반드시 cell_idx에 있어야 함".
        // 이걸 negative("cell_idx에 없어야 함") 쌍과 함께 쓰면 표준 CBS disjoint
        // splitting이 된다 (positive constraint의 SIPP 확장 없이도 완전성 유지).
        void add_section_complement_constraint(int time, int section_id, int cell_idx);

        // Pure-CBS용 empty 초기화. 모든 테이블을 비우고 add_cell_constraint로만 제약을 주입.
        void init_empty();
};
