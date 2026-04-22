#pragma once
#include "BasicGraph.h"
#include "States.h"
#include "PriorityGraph.h"
#include "PBS.h"
#include "WHCAStar.h"
#include "ECBS.h"
#include "LRAStar.h"
#include "SectionState.h"
#include "MapSystem.h"
#include "PBSSection.h"
#include "ECBSSection.h"


class BasicSystem
{
public:
    // params for MAPF algotithms
	MAPFSolver& solver;
    // 섹션 기반 solver는 MAPFSolver를 상속한 구현체(PBSSection, ECBSSection 등)를
    // 받는다. BasicSystem은 run_section/save_* 가상 인터페이스만 호출한다.
    MAPFSolver* solver_section;
	bool hold_endpoints;
	bool useDummyPaths;
    int time_limit;
    int travel_time_window;
	//string potential_function;
	//double potential_threshold;
	//double suboptimal_bound;
    int screen;
	bool log;
    int num_of_drives;
    int seed;
    int simulation_window;
    int planning_window;
    int simulation_time;

    // params for drive model
    bool consider_rotation;
    int k_robust;

    BasicSystem(const BasicGraph& G, MAPFSolver& solver);
    BasicSystem(const BasicGraph& G, MAPFSolver& solver, MAPFSolver& solver_section);
    ~BasicSystem();

	// TODO
    /*bool load_config(std::string fname);
    bool generate_random_MAPF_instance();
    bool run();
	void print_MAPF_instance() const;
	void save_MAPF_instance(std::string fname) const;
	bool read_MAPF_instance(std::string fname);*/

    // I/O
    std::string outfile;
    void save_results();
	double saving_time = 0; // time for saving results to files, in seconds
    int num_of_tasks; // number of finished tasks

	list<int> new_agents; // used for replanning a subgroup of agents

    // used for MAPF instance
    vector<State> starts;
    vector< vector<pair<int, int> > > goal_locations;
	// unordered_set<int> held_endpoints;
    int timestep;


    ///////////////////추가//////////////////////
    vector<SectionState> start_sections;
    vector<vector<pair<SectionState, int> > > goal_sections;
    ///////////////////////////////////////////

    // record movements of drives
    std::vector<Path> paths;
    std::vector<std::list<std::pair<int, int> > > finished_tasks; // location + finish time

    bool congested() const;
	bool check_collisions(const vector<Path>& input_paths) const;

    // update
    void update_start_locations();
    void update_travel_times(unordered_map<int, double>& travel_times);
    void update_paths(const std::vector<Path*>& MAPF_paths, int max_timestep);
    void update_paths(const std::vector<Path>& MAPF_paths, int max_timestep);
    void update_paths_section(const std::vector<SectionPath>& MAPF_paths, MapSystem* mapsys, int max_timestep);
    void update_initial_paths(vector<Path>& initial_paths) const;
    void update_initial_constraints(list< tuple<int, int, int> >& initial_constraints) const;
    
	void add_partial_priorities(const vector<Path>& initial_paths, PriorityGraph& initial_priorities) const;
	list<tuple<int, int, int>> move(); // return finished tasks
	void solve();
    void solve_by_Section(MapSystem &mapSys);
	void initialize_solvers();
	bool load_records();
	bool load_locations();

    ///////////////////추가//////////////////////
    void conversion_to_sections(MapSystem& mapSys, int current_time);
    void print_conversion_debug(int grid_cols) const;
    ///////////////////////////////////////////

    ///////////// Wall-clock 진단용 (section 모드) /////////////
    // Phase별 누적 wall-time (std::chrono::steady_clock 기반)
    double diag_wall_total_sec = 0;          // solve_by_Section 전체
    double diag_wall_conversion_sec = 0;     // conversion_to_sections
    double diag_wall_primary_sec = 0;        // solver_section->run_section (성공/실패 무관)
    double diag_wall_primary_success_sec = 0;// 성공 case만
    double diag_wall_primary_fail_sec = 0;   // 실패 case만
    double diag_wall_fallback_sec = 0;       // fallback solver.run (baseline PBS)
    double diag_wall_update_sec = 0;         // update_paths_section / update_paths + lra
    double diag_wall_save_sec = 0;           // solver_section->save_results (I/O)
    int diag_num_calls = 0;
    int diag_num_primary_success = 0;
    int diag_num_fallback = 0;
    void print_diagnostics() const;
    //////////////////////////////////////////////////////////

protected:
	bool solve_by_WHCA(vector<Path>& planned_paths,
		const vector<State>& new_starts, const vector< vector<pair<int, int> > >& new_goal_locations);
    bool LRA_called = false;

private:
	const BasicGraph& G;
};

