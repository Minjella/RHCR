#pragma once
#include "ECBSNodeSection.h"
#include "MAPFSolver.h"
#include <cstdint>
#include <ctime>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Section-level ECBS (v2, clean rewrite).
//
// Core idea: conflict resolution is done at **section-entry** level instead
// of cell level. State space is section-based; constraints should be too.
//
//   * Constraint = (agent, section_id, entry_cell, entry_time)
//     "agent cannot be at <entry_cell> of <section_id> at <entry_time>"
//     — effectively blocks the agent from ENTERING the section at that time.
//
//   * SIPP은 기존 is_cell_safe 로직으로 자동 차단 (별도 SIPP 수정 없음).
//     add_cell_constraint(time, section, entry_cell)을 그대로 사용.
//
//   * For conflicts in an agent's START section (entry_time == 0), entry-time
//     constraint는 infeasible이므로 cell-level fallback (기존 방식).
//
// Removed from v1:
//   * CAT (section + cell level)
//   * BYPASS (원조 ECBS에 없음)
//   * Cardinal prioritization, CG heuristic
//
// High-level은 표준 ECBS: open_list (min_f_val 정렬) + focal_list
// (num_of_collisions 정렬) with suboptimal_bound w.
class ECBSSection : public MAPFSolver
{
public:
    bool disjoint_splitting = false;          // v1: standard splitting only
    std::string potential_function = "NONE";
    double potential_threshold = 0.0;
    double suboptimal_bound = 1.0;            // w

    ECBSNodeSection* dummy_start = nullptr;
    ECBSNodeSection* best_node = nullptr;

    uint64_t HL_num_expanded = 0;
    uint64_t HL_num_generated = 0;
    uint64_t LL_num_expanded = 0;
    uint64_t LL_num_generated = 0;

    // Diagnostic counters (safety-net hits = bucketing이 놓친 conflict 수).
    uint64_t safety_net_fires = 0;
    uint64_t safety_net_conflicts_found = 0;

    // Runtime breakdown
    double runtime_rt = 0;
    double runtime_plan_paths = 0;
    double runtime_detect_conflicts = 0;
    double runtime_copy_conflicts = 0;
    double runtime_choose_conflict = 0;
    double runtime_update_focal = 0;

    // MAPFSolver 순수 가상 override: 섹션 전용이라 no-op.
    virtual bool run(const vector<State>& starts,
                     const vector<vector<pair<int, int>>>& goal_locations,
                     int time_limit) override;

    bool run_section(const vector<SectionState>& start_sections,
                     const vector<vector<pair<SectionState, int>>> goal_sections,
                     int time_limit, MapSystem* mapsys) override;

    ECBSSection(const BasicGraph& G, SingleAgentSolver& path_planner);
    ECBSSection(const BasicGraph& G, SingleAgentSolver& path_planner,
                SIPPSection& section_path_planner);
    ~ECBSSection();

    void save_results(const std::string& fileName, const std::string& instanceName) const override;
    void save_search_tree(const std::string& fileName) const override;
    void save_constraints_in_goal_node(const std::string& /*fileName*/) const override {}
    std::string get_name() const override { return "ECBS Section (v2, section-entry)"; }
    void clear() override;

private:
    typedef boost::heap::fibonacci_heap<ECBSNodeSection*, boost::heap::compare<ECBSNodeSection::compare_node>> heap_open_t;
    typedef boost::heap::fibonacci_heap<ECBSNodeSection*, boost::heap::compare<ECBSNodeSection::secondary_compare_node>> heap_focal_t;
    heap_open_t open_list;
    heap_focal_t focal_list;
    std::list<ECBSNodeSection*> allNodes_table;

    std::vector<SectionPath*> paths;
    std::vector<double> path_min_costs;
    std::vector<double> path_costs;

    std::clock_t start_time = 0;

    double min_f_val = 0;
    double focal_threshold = 0;

    MapSystem* current_mapsys = nullptr;

    // Scratch buffers for find_conflicts (ported from PBSSection Opt B).
    // Reused across calls; assign()/clear() preserves capacity.
    std::vector<int> fc_sizes;
    std::vector<int> fc_sec_idx;
    std::vector<int> fc_sec_inside_idx;
    std::unordered_set<uint64_t> fc_pair_reported;
    std::unordered_map<uint32_t, std::vector<int>> fc_bucket;

    // Incremental per-agent timestep-major (section_id, cell_idx) key cache
    // for find_conflicts(list&, new_agent). Rebuilt lazily via
    // refresh_keys_cache() (only rows whose paths[a] pointer changed since
    // last sync) and eagerly via build_keys_for_agent(a) after find_path.
    std::vector<uint32_t> fc_keys_flat;
    std::vector<int> fc_keys_size;
    std::vector<const SectionPath*> fc_cache_path_ptr;
    int fc_W = 0;
    void build_keys_for_agent(int a);
    void refresh_keys_cache();

    // ---- High-level ----
    bool generate_root_node(MapSystem* mapsys);
    void push_node(ECBSNodeSection* node);
    void reinsert_node(ECBSNodeSection* node);
    ECBSNodeSection* pop_node();
    void update_focal_list();

    void update_paths(ECBSNodeSection* curr);

    bool find_path(ECBSNodeSection* node, int agent, MapSystem* mapsys);
    void resolve_conflict(const SectionConflict& c, ECBSNodeSection* n1, ECBSNodeSection* n2);
    void resolve_for_agent(int agent, const SectionConflict& c, ECBSNodeSection* node);
    bool generate_child(ECBSNodeSection* child, ECBSNodeSection* parent, MapSystem* mapsys);

    void collect_constraints(ECBSNodeSection* node, int agent,
                             std::list<SectionConstraint>& out) const;

    // 주어진 time에 agent가 어느 SectionState에 있는지 찾기.
    const SectionState* find_state_at_time(int agent, int timestep) const;

    // ---- Conflict detection ----
    void find_conflicts(std::list<SectionConflict>& conflicts);
    void find_conflicts(std::list<SectionConflict>& new_conflicts, int new_agent);
    void find_conflicts(std::list<SectionConflict>& conflicts, int a1, int a2);
    void find_conflicts(const std::list<SectionConflict>& old_conflicts,
                        std::list<SectionConflict>& new_conflicts,
                        const std::list<int>& new_agents);

    void copy_conflicts(const std::list<SectionConflict>& conflicts,
                        std::list<SectionConflict>& copy,
                        const std::list<int>& excluded_agents) const;
    void remove_conflicts(std::list<SectionConflict>& conflicts, int excluded_agent) const;
    void choose_conflict(ECBSNodeSection& node) const;

    inline void release_closed_list();
    double get_path_cost(const SectionPath& path) const;

    void get_solution();
    bool validate_solution();
    void print_results() const;
    void print_paths() const;
};
