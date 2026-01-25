#pragma once
#include <vector>
#include <array>
#include <map>
#include <unordered_map>
#include <memory>
#include <limits>
#include <cstdint>
#include <bit>
#include "common.h"


enum class SectionType{
    Eject_CW, Eject_CCW,
    Eject_UP, Eject_DOWN,
    Induct_C_TOP, Induct_C_BOTTOM,
    Induct_N_TOP, Induct_N_BOTTOM,
    Travel_LEFT_TOP, Travel_LEFT_BOTTOM,
    Travel_RIGHT_TOP, Travel_RIGHT_BOTTOM,
    Corner_LEFT_TOP, Corner_RIGHT_TOP,
    Corner_LEFT_BOTTOM, Corner_RIGHT_BOTTOM

};

enum class CellType {TRAVEL, OBSTACLE, EJECT, INDUCT};

class SectionTemplate {
    public:
        SectionType type;
        int capacity;
        int width = 3; 
        int height = 3;

        // Grid Layout & Cell Attributes
        struct CellInfo {
            CellType type = CellType::TRAVEL;
            bool is_valid = true; // Induct (6,7,8) = false
        };
        vector<CellInfo> cells;

        // IN/OUT Cell
        vector<int> entries;
        vector<int> exits;
        vector<int> entry_cells_by_dir;
        vector<int> exit_cells_by_dir;

        // Internal Graph (move) adj[from] = {to, to, ...}
        vector<vector<int>> adj;

        // path_table[from][to] = {start, node, ..., goal}
        vector<vector<vector<int>>> path_table;

        // Internal Distance Matrix internal_dist_matrix[from][to] = time (못가면 inf)
        vector<vector<int>> internal_dist_matrix;

        SectionTemplate(SectionType t, int cap);

        void setup_connectivity();
        void precompute_all();

};

class Section{
    public:
        int id;
        const SectionTemplate* info;

        // Section Loc
        int grid_x, grid_y;

        // Standard Cell Loc
        int anchor_x, anchor_y;

        // Connectivity (Neighbors) <exit index, {section pointer, entry index}>
        struct PortConnection {
            Section* target_sec;
            int target_entry_idx;
        };
        std::map<int, std::vector<PortConnection>> neighbors; // 특정 출구에서의 연결정보 저장
        
        // Bottleneck Schedule (Cell Level) <time -> cell_bitmask>
        std::map<int, uint16_t> bottleneck_timeline;

        // Congestion Score
        double congestion_score = 0.0;

        Section(int _id, const SectionTemplate* _info, int _gx, int _gy, int _ax, int _ay);

        // Count agent
        int get_agent_count(int t) const;

        // Capacity check
        bool check_capacity_available(int entry_time, int duration);

        // bottleneck check
        bool check_cell_available(int time, int cell_idx);

        // add reservation
        bool add_reservation(int entry_time, int entry_idx, int exit_idx, const vector<int>& wait_nodes);

        // remove reservation
        void remove_reservation(int entry_time, int entry_idx, int exit_idx, const vector<int>& wait_nodes);

        vector<pair<int, int>> get_internal_path(int start_time, int entry_idx, int exit_idx, const vector<int>& wait_nodes);
};
