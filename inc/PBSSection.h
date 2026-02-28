// #pragma once
// #include "PBSNodeSection.h"
// #include "MAPFSolver.h"
// #include <ctime>

// // TODO: add topological sorting

// class PBSSection:
// 	public MAPFSolver
// {
// public:
//     bool lazyPriority;
//     bool prioritize_start = true;

// 	 // runtime breakdown
//     double runtime_rt = 0;
//     double runtime_plan_paths = 0;
//     double runtime_get_higher_priority_agents = 0;
//     double runtime_copy_priorities = 0;
//     double runtime_detect_conflicts = 0;
//     double runtime_copy_conflicts = 0;
//     double runtime_choose_conflict = 0;
//     double runtime_find_consistent_paths = 0;
//     double runtime_find_replan_agents = 0;


// 	PBSNodeSection* dummy_start = nullptr;
// 	PBSNodeSection* best_node;

// 	uint64_t HL_num_expanded = 0;
// 	uint64_t HL_num_generated = 0;
// 	uint64_t LL_num_expanded = 0;
// 	uint64_t LL_num_generated = 0;


// 	double min_f_val = 0;


// 	// Runs the algorithm until the problem is solved or time is exhausted 
//     bool run_section(const vector<SectionState>& start_sections,
//             const vector< vector<pair<SectionState, int> > >& goal_sections, // an ordered list of pairs of <location, release time>
//             int time_limit);


//     PBSSection(const BasicGraph& G, SingleAgentSolver& path_planner);
//     PBSSection(const BasicGraph& G, SingleAgentSolver& path_planner, SIPPSection& section_path_planner);
// 	~PBSSection();

//     void update_paths(PBSNodeSection* curr);
// 	// Save results
// 	void save_results(const std::string &fileName, const std::string &instanceName) const;
// 	void save_search_tree(const std::string &fileName) const;
// 	void save_constraints_in_goal_node(const std::string &fileName) const;

// 	string get_name() const {return "PBS Section"; }

// 	void clear();

// 	void setRS(bool use_cat, bool prioritize_start)
// 	{
// 		rs.use_cat = use_cat; // false
// 		rs.prioritize_start = prioritize_start; // true
// 	}

// private:

//     std::vector< SectionPath* > paths;
//     list<PBSNodeSection*> allNodes_table;
//     list<PBSNodeSection*> dfs;

//     std::clock_t start = 0;

//     unordered_set<pair<int, int>> nogood; // ?? nogood이 뭔지 찾아서 고쳐야함

//     bool generate_root_node();
//     void push_node(PBSNodeSection* node);
//     PBSNodeSection* pop_node();

//     // high level search
// 	bool find_path(PBSNodeSection*  node, int ag);
//     bool find_consistent_paths(PBSNodeSection* node, int a); // find paths consistent with priorities, 우선순위 제약 기반으로 새로운 경로 찾기 (t/f)
//     static void resolve_conflict(const Conflict& conflict, PBSNodeSection* n1, PBSNodeSection* n2); 
// 	bool generate_child(PBSNodeSection* child, PBSNodeSection* curr);

// 	// conflicts
//     void remove_conflicts(list<Conflict>& conflicts, int excluded_agent);
//     void find_conflicts(const list<Conflict>& old_conflicts, list<Conflict> & new_conflicts, int new_agent);
// 	void find_conflicts(list<Conflict> & conflicts, int a1, int a2);
//     void find_conflicts(list<Conflict> & new_conflicts, int new_agent);
//     void find_conflicts(list<Conflict> & new_conflicts);

// 	void choose_conflict(PBSNodeSection &parent);
// 	void copy_conflicts(const list<Conflict>& conflicts, list<Conflict>& copy, int excluded_agent);
//     void copy_conflicts(const list<Conflict>& conflicts,
//                        list<Conflict>& copy, const vector<bool>& excluded_agents);

//     double get_path_cost(const SectionPath& path) const;
	
//     // update information
//     void get_solution();

//     void update_CAT(int ex_ag); // update conflict avoidance table -> 우선 안씀
// 	void update_focal_list();
// 	inline void release_closed_list();
//     void update_best_node(PBSNodeSection* node);

// 	// print and save
// 	void print_paths() const;
// 	void print_results() const;
// 	static void print_conflicts(const PBSNodeSection &curr) ;

// 	// validate
// 	bool validate_solution();
//     static bool validate_consistence(const list<Conflict>& conflicts, const PriorityGraph &G) ;

//     // tools
//     static bool wait_at_start(const SectionPath& path, SectionState start_section, int timestep) ;
//     void find_replan_agents(PBSNodeSection* node, const list<Conflict>& conflicts,
//             unordered_set<int>& replan);
// };
