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

        // Pure-CBS용 empty 초기화. 모든 테이블을 비우고 add_cell_constraint로만 제약을 주입.
        void init_empty();
};
