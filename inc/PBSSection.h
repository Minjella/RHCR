#pragma once
#include "PBSNodeSection.h"
#include "MAPFSolver.h"
#include <ctime>

// TODO: add topological sorting

class PBSSection:
	public MAPFSolver
{
public:
    bool lazyPriority;
    bool prioritize_start = true;

	 // runtime breakdown
    double runtime_rt = 0;
    double runtime_plan_paths = 0;
    double runtime_get_higher_priority_agents = 0;
    double runtime_copy_priorities = 0;
    double runtime_detect_conflicts = 0;
    double runtime_copy_conflicts = 0;
    double runtime_choose_conflict = 0;
    double runtime_find_consistent_paths = 0;
    double runtime_find_replan_agents = 0;


	PBSNodeSection* dummy_start = nullptr;
	PBSNodeSection* best_node;

	uint64_t HL_num_expanded = 0;
	uint64_t HL_num_generated = 0;
	uint64_t LL_num_expanded = 0;
	uint64_t LL_num_generated = 0;


	double min_f_val = 0;

    virtual bool run(const vector<State>& starts,
            const vector< vector<pair<int, int> > >& goal_locations,  // an ordered list of pairs of <location, release time>
            int time_limit) override;

	// Runs the algorithm until the problem is solved or time is exhausted
    bool run_section(const vector<SectionState>& start_sections,
            const vector<vector<pair<SectionState, int> > > goal_sections, // an ordered list of pairs of <location, release time>
            int time_limit, MapSystem* mapsys) override;


    PBSSection(const BasicGraph& G, SingleAgentSolver& path_planner);
    PBSSection(const BasicGraph& G, SingleAgentSolver& path_planner, SIPPSection& section_path_planner);
	~PBSSection();

    void update_paths(PBSNodeSection* curr);
	// Save results
	void save_results(const std::string &fileName, const std::string &instanceName) const override;
	void save_search_tree(const std::string &fileName) const override;
	void save_constraints_in_goal_node(const std::string &fileName) const override;

	string get_name() const override {return "PBS Section"; }

	void clear() override;

	void setRS(bool use_cat, bool prioritize_start)
	{
		rs.use_cat = use_cat; // false
		rs.prioritize_start = prioritize_start; // true
	}

private:

    std::vector< SectionPath* > paths;
    list<PBSNodeSection*> allNodes_table;
    list<PBSNodeSection*> dfs;

    std::clock_t start = 0;

    // Scratch buffers for find_conflicts — reused across calls to avoid
    // per-call heap allocation. clear()/assign() preserves capacity.
    std::vector<int> fc_sizes;
    std::vector<int> fc_sec_idx;
    std::vector<int> fc_sec_inside_idx;
    std::vector<uint32_t> fc_new_keys;
    std::unordered_set<uint64_t> fc_pair_reported;
    std::unordered_map<uint32_t, std::vector<int>> fc_bucket;

    // Per-agent timestep-major (section_id, cell_idx) key cache. Incremental:
    // rows are refreshed only when their underlying paths[a] pointer changed
    // since the last cache sync (refresh_keys_cache scan), plus eager update
    // when find_path succeeds. Survives across find_consistent_paths calls,
    // so no repeated O(N·W) rebuild per fcp entry.
    //
    // Flat layout: [a * fc_W + ts]. UINT32_MAX at out-of-range ts.
    // Invariant: fc_cache_path_ptr[a] == paths[a] after refresh_keys_cache().
    std::vector<uint32_t> fc_keys_flat;
    std::vector<int> fc_keys_size;                 // per-agent valid length
    std::vector<const SectionPath*> fc_cache_path_ptr; // path pointer seen at last cache sync
    int fc_W = 0; // cached (window + 1) for current cache layout
    void build_keys_for_agent(int a);
    void refresh_keys_cache();

    unordered_set<pair<int, int>> nogood; // ?? nogood이 뭔지 찾아서 고쳐야함 -> a1, a2

    bool generate_root_node(MapSystem* mapsys);
    void push_node(PBSNodeSection* node);
    PBSNodeSection* pop_node();

    // high level search
	bool find_path(PBSNodeSection*  node, int agent, MapSystem* mapsys);
    bool find_consistent_paths(PBSNodeSection* node, int agent, MapSystem* mapsys); // find paths consistent with priorities, 우선순위 제약 기반으로 새로운 경로 찾기 (t/f)
    static void resolve_conflict(const SectionConflict& conflict, PBSNodeSection* n1, PBSNodeSection* n2); 
	bool generate_child(PBSNodeSection* child, PBSNodeSection* curr, MapSystem* mapsys);

	// conflicts
    void remove_conflicts(list< SectionConflict>& conflicts, int excluded_agent);
    void find_conflicts(const list<SectionConflict>& old_conflicts, list<SectionConflict> & new_conflicts, int new_agent);
	void find_conflicts(list<SectionConflict> & conflicts, int a1, int a2);
    void find_conflicts(list<SectionConflict> & new_conflicts, int new_agent);
    // stop_after_first_ts: if true, break out of timestep loop after the
    // first timestep that produced any conflict. Safe only when choose_conflict
    // won't need conflicts beyond that timestep — i.e. nogood is empty (no
    // risk of picking a later non-nogood when earliest is all nogood).
    void find_conflicts(list<SectionConflict> & new_conflicts,
                        bool stop_after_first_ts = false);

	bool choose_conflict(PBSNodeSection &parent);
	void copy_conflicts(const list<SectionConflict>& conflicts, list<SectionConflict>& copy, int excluded_agent);
    void copy_conflicts(const list<SectionConflict>& conflicts,
                       list<SectionConflict>& copy, const vector<bool>& excluded_agents);

    double get_path_cost(const SectionPath& path) const;
	
    // update information
    void get_solution();

    void update_CAT(int ex_ag); // update conflict avoidance table -> 우선 안씀
	void update_focal_list();
	inline void release_closed_list();
    void update_best_node(PBSNodeSection* node);

	// print and save
	void print_paths() const;
	void print_results() const;
	static void print_conflicts(const PBSNodeSection &curr) ;

	// validate
	bool validate_solution();
    static bool validate_consistence(const list<SectionConflict>& conflicts, const PriorityGraph &G) ;

    // tools
    static bool wait_at_start(const SectionPath& path, int section_id, int start_index, int timestep) ;
    void find_replan_agents(PBSNodeSection* node, const list<SectionConflict>& conflicts,
            unordered_set<int>& replan, const unordered_set<int>& already_replanned);
};
