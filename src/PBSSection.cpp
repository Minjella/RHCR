#include "PBSSection.h"
#include <ctime>
#include <iostream>
#include "PathTableSection.h"





void PBSSection::clear()
{
    runtime = 0;
    runtime_rt = 0;
	runtime_plan_paths = 0;
    runtime_get_higher_priority_agents = 0;
    runtime_copy_priorities = 0;
    runtime_detect_conflicts = 0;
    runtime_copy_conflicts = 0;
    runtime_choose_conflict = 0;
    runtime_find_consistent_paths = 0;
    runtime_find_replan_agents = 0;

    HL_num_expanded = 0;
    HL_num_generated = 0;
    LL_num_expanded = 0;
    LL_num_generated = 0;
    solution_found = false;
    solution_cost = -2;
    min_f_val = -1;
    avg_path_length = -1;
    paths.clear();
    nogood.clear();
    dfs.clear();
    release_closed_list();
    start_sections.clear();
    goal_locations.clear();
    best_node = nullptr;

}

// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
void PBSSection::update_paths(PBSNodeSection* curr)
{
    vector<bool> updated(num_of_agents, false);  // initialized for false
	while (curr != nullptr)
	{
        for (auto p = curr->paths.begin(); p != curr->paths.end(); ++p)
        {
		    if (!updated[std::get<0>(*p)])
		    {
			    paths[std::get<0>(*p)] = &(std::get<1>(*p));
			    updated[std::get<0>(*p)] = true;
		    }
        }
		curr = curr->parent;
	}
}


// deep copy of all conflicts except ones that involve the particular agent
// used for copying conflicts from the parent node to the child nodes
void PBSSection::copy_conflicts(const list<SectionConflict>& conflicts,
	list<SectionConflict>& copy, const vector<bool>& excluded_agents)
{
    clock_t t = clock();
	for (auto conflict : conflicts)
	{
		if (!excluded_agents[conflict.agent1] && !excluded_agents[conflict.agent2])
		{
			copy.push_back(conflict);
		}
	}
    runtime_copy_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}


void PBSSection::copy_conflicts(const list<SectionConflict>& conflicts, list<SectionConflict>& copy, int excluded_agent)
{
    clock_t t = clock();
    for (auto conflict : conflicts)
    {
        if (excluded_agent != conflict.agent1 && excluded_agent != conflict.agent2)
        {
            copy.push_back(conflict);
        }
    }
    runtime_copy_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}


void PBSSection::find_conflicts(list<SectionConflict>& conflicts, int a1, int a2)
{
    clock_t t = clock();
    if (paths[a1] == nullptr || paths[a2] == nullptr)
        return;

    int size1 = min(window + 1, (int)paths[a1]->size());
    int size2 = min(window + 1, (int)paths[a2]->size());
    int sec_idx_1 = 0;
    int sec_idx_2 = 0;
    int sec_inside_idx_1 = 0;
    int sec_inside_idx_2 = 0;
    int section_id_1 = (*paths[a1])[sec_idx_1].section_id;
    int section_id_2 = (*paths[a2])[sec_idx_2].section_id;
    auto& internal_path_1 = (*paths[a1])[sec_idx_1].full_path;
    auto& internal_path_2 = (*paths[a2])[sec_idx_2].full_path;

    for (int timestep = 0; timestep < size1; timestep++)
    {
        if (size2 <= timestep)
            break;

        if (sec_idx_1 + 1 < paths[a1]->size() && (*paths[a1])[sec_idx_1+1].timestep >= timestep){
            sec_idx_1 += 1;
            section_id_1 = (*paths[a1])[sec_idx_1].section_id;
            sec_inside_idx_1 = 0;
            internal_path_1 = (*paths[a1])[sec_idx_1].full_path;
        }
        if (sec_idx_2 + 1 < paths[a2]->size() && (*paths[a2])[sec_idx_2+1].timestep >= timestep){
            sec_idx_2 += 1;
            section_id_2 = (*paths[a2])[sec_idx_2].section_id;
            sec_inside_idx_1 = 0;
            internal_path_2 = (*paths[a2])[sec_idx_2].full_path;
        }

        if (section_id_1 == section_id_2){
            if (internal_path_1[sec_inside_idx_1] == internal_path_2[sec_inside_idx_2]){
                conflicts.emplace_back(a1, a2, section_id_1, internal_path_1[sec_inside_idx_1].second, timestep, ConflictType::TILE_VERTEX);
                runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
                return;
            }
        }

        sec_inside_idx_1 += 1;
        sec_inside_idx_2 += 1;
    }

	runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}

void PBSSection::find_conflicts(list<SectionConflict>& conflicts)
{
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
        {
            find_conflicts(conflicts, a1, a2);
        }
    }
}

void PBSSection::find_conflicts(list<SectionConflict>& new_conflicts, int new_agent)
{
    for (int a2 = 0; a2 < num_of_agents; a2++)
    {
        if(new_agent == a2)
            continue;
        find_conflicts(new_conflicts, new_agent, a2);
    }
}


// New agent 에 대한 conflict 추가 (나머지는 parents 복사)
void PBSSection::find_conflicts(const list<SectionConflict>& old_conflicts, list<SectionConflict>& new_conflicts, int new_agent)
{
    // Copy from parent
    copy_conflicts(old_conflicts, new_conflicts, new_agent);

    // detect new conflicts
    find_conflicts(new_conflicts, new_agent);
}


void PBSSection::remove_conflicts(list<SectionConflict>& conflicts, int excluded_agent)
{
    clock_t t = clock();
    for (auto it = conflicts.begin(); it != conflicts.end();)
    {
        if((*it).agent1 == excluded_agent || (*it).agent2 == excluded_agent)
        {
            it = conflicts.erase(it);
        }
		else
		{
			++it;
		}
    }
    runtime_copy_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}

void PBSSection::choose_conflict(PBSNodeSection &node)
{
    clock_t t = clock();
	if (node.conflicts.empty())
	    return;

    node.conflict = node.conflicts.front();

	// choose the earliest
    for (auto conflict : node.conflicts)
    {
        if (conflict.timestep < node.conflict.timestep)
            node.conflict = conflict;
    }
    node.earliest_collision = node.conflict.timestep;

    if (!nogood.empty())
    {
        for (auto conflict : node.conflicts)
        {
            int a1 = conflict.agent1;
            int a2 = conflict.agent2;
            for (auto p : nogood)
            {
                if ((a1 == p.first && a2 == p.second) || (a1 == p.second && a2 == p.first))
                {
                    node.conflict = conflict;
                    runtime_choose_conflict += (double)(std::clock() - t) / CLOCKS_PER_SEC;
                    return;
                }
            }
        }
    }

    runtime_choose_conflict += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}


double PBSSection::get_path_cost(const SectionPath& path) const
{
    double cost = 0;
    for (int i = 0; i < (int)path.size() - 1; i++)
    {
        cost += path[i].full_path.size();
    }
    return cost;
}

bool PBSSection::find_path(PBSNodeSection* node, int agent, MapSystem* mapsys)
{
    SectionPath path;
    double path_cost;

    clock_t t = std::clock();
	rs.clear();
    rs.build(paths, node->priorities.get_reachable_nodes(agent), mapsys);
    runtime_get_higher_priority_agents += node->priorities.runtime;

    runtime_rt += (double)(std::clock() - t) / CLOCKS_PER_SEC;

    t = std::clock();
    path = section_path_planner->run_section(start_sections[agent], goal_sections[agent], rs, agent, 8, mapsys);
	runtime_plan_paths += (double)(std::clock() - t) / CLOCKS_PER_SEC;
    path_cost = section_path_planner->path_cost;

    LL_num_expanded += section_path_planner->num_expanded;
    LL_num_generated += section_path_planner->num_generated;

    if (path.empty())
    {
        if (screen == 2)
            std::cout << "Fail to find a path" << std::endl;
        return false;
    }
    double old_cost = 0;
    if (paths[agent] != nullptr)
        old_cost = get_path_cost(*paths[agent]);
    node->g_val = node->g_val - old_cost + path_cost;
    for (auto it = node->paths.begin(); it != node->paths.end(); ++it)
    {
        if (std::get<0>(*it) == agent)
        {
            node->paths.erase(it);
            break;
        }
    }
    node->paths.emplace_back(agent, path);
    paths[agent] = &node->paths.back().second;
    return true;
}


// return true if the agent keeps waiting at its start location until at least timestep
bool PBSSection::wait_at_start(const SectionPath& path, int section_id, int start_index, int timestep)
{
    for (auto& state : path)
    {
        if (state.timestep > timestep)
            return true;
        else if (state.section_id != section_id)
            return false;
        else if (state.section_id == section_id)
            for (auto index: state.full_path){
                if (index.second != start_index){
                    return false;
                }
            }
    }
    return false; // when the path is empty
}

// conflict 보다 timestep이 뒤면 -> true
// conflict의 위치와 다르면 -> false


void PBSSection::find_replan_agents(PBSNodeSection* node, const list<SectionConflict>& conflicts,
        unordered_set<int>& replan)
{
    clock_t t2 = clock();
    for (const auto& conflict : conflicts)
    {

        int a1, a2, si, li, t;
        ConflictType ty;

        a1 = conflict.agent1;
        a2 = conflict.agent2;
        si = conflict.section_id;
        li = conflict.local_index;
        t = conflict.timestep;
        ty = conflict.type;

        if (replan.find(a1) != replan.end() || replan.find(a2) != replan.end())
            continue;
        else if (prioritize_start && wait_at_start(*paths[a1], si, li, t))
        {
            replan.insert(a2);
            continue;
        }
        else if (prioritize_start && wait_at_start(*paths[a2], si, li, t))
        {
            replan.insert(a1);
            continue;
        }
        if (node->priorities.connected(a1, a2))
        {
            replan.insert(a1);
            continue;
        }
        if (node->priorities.connected(a2, a1))
        {
            replan.insert(a2);
            continue;
        }
    }
    runtime_find_replan_agents += (double)(std::clock() - t2) / CLOCKS_PER_SEC;
}


bool PBSSection::find_consistent_paths(PBSNodeSection* node, int agent, MapSystem* mapsys)
{
    clock_t t = clock();
    int count = 0; // count the times that we call the low-level search.
    unordered_set<int> replan;
    if (agent >= 0 && agent < num_of_agents)
        replan.insert(agent);
    find_replan_agents(node, node->conflicts, replan);

    while (!replan.empty())
    {
        if (count > (int) node->paths.size() * 5)
        {
            runtime_find_consistent_paths += (double)(std::clock() - t) / CLOCKS_PER_SEC;
            return false;
        }
        int a = *replan.begin();
        replan.erase(a);
        count++;

        if (!find_path(node, a, mapsys))
        {
            runtime_find_consistent_paths += (double)(std::clock() - t) / CLOCKS_PER_SEC;
            return false;
        }
        remove_conflicts(node->conflicts, a);
        list<SectionConflict> new_conflicts;
        find_conflicts(new_conflicts, a);

        find_replan_agents(node, new_conflicts, replan);

        node->conflicts.splice(node->conflicts.end(), new_conflicts);
    }
    runtime_find_consistent_paths += (double)(std::clock() - t) / CLOCKS_PER_SEC;
    if (screen == 2)
        return validate_consistence(node->conflicts, node->priorities);
    return true;
}

// 우선순위갸 정해진 애들끼리 또 충돌했다면, false -> 노드는 가망이 없음
bool PBSSection::validate_consistence(const list<SectionConflict>& conflicts, const PriorityGraph &G)
{
    for (auto conflict : conflicts)
    {
        int a1 = conflict.agent1;
        int a2 = conflict.agent2;
        if (G.connected(a1, a2) || G.connected(a2, a1))
            return false;
    }
    return true;
}


bool PBSSection::generate_child(PBSNodeSection* node, PBSNodeSection* parent, MapSystem* mapsys)
{
	node->parent = parent;
	node->g_val = parent->g_val;
	node->makespan = parent->makespan;
	node->depth = parent->depth + 1;

    
    clock_t t = clock();
    node->priorities.copy(node->parent->priorities);
    node->priorities.add(node->priority.first, node->priority.second);
    runtime_copy_priorities += (double)(std::clock() - t) / CLOCKS_PER_SEC;
    copy_conflicts(node->parent->conflicts, node->conflicts, -1); // copy all conflicts
    if (!find_consistent_paths(node, node->priority.first, mapsys))
        return false;

    node->num_of_collisions = node->conflicts.size();

	//Estimate h value
	node->h_val = 0;
	node->f_val = node->g_val + node->h_val;

	return true;
}


bool PBSSection::generate_root_node(MapSystem* mapsys)
{
    clock_t time = std::clock();
	dummy_start = new PBSNodeSection();
	
	// initialize paths_found_initially
	paths.resize(num_of_agents, nullptr);
	
    if (screen == 2)
        std::cout << "Generate root CT node ..." << std::endl;

    // if (!initial_paths.empty())
    // {
    //     for (int i = 0; i < num_of_agents; i++)
    //     {
    //         if (!initial_paths[i].empty())
    //         {
    //             dummy_start->paths.emplace_back(make_pair(i, initial_paths[i]));
    //             SectionState sec_state;
    //             sec_state.section_id = 
    //             paths[i] = &dummy_start->paths.back().second;
    //             dummy_start->makespan = std::max(dummy_start->makespan, paths[i]->size() - 1);
    //             dummy_start->g_val += get_path_cost(*paths[i]);
    //         }
    //     }
    // }

    for (int i = 0; i < num_of_agents; i++) 
	{
        if (paths[i] != nullptr)
            continue;
        SectionPath path;
        double path_cost;
        int start_section_id  = start_sections[i].section_id;
        int start_section_index = start_sections[i].start_index;
        clock_t t = std::clock();
		rs.clear();
        rs.build(paths, dummy_start->priorities.get_reachable_nodes(i), mapsys);
        runtime_get_higher_priority_agents += dummy_start->priorities.runtime;
        runtime_rt += (double)(std::clock() - t) / CLOCKS_PER_SEC;
        vector< vector<double> > h_values(goal_locations[i].size());
        t = std::clock();
        path = section_path_planner->run_section(start_sections[i], goal_sections[i], rs, i, 8, mapsys);
		runtime_plan_paths += (double)(std::clock() - t) / CLOCKS_PER_SEC;
        path_cost = section_path_planner->path_cost;
        rs.clear();
        LL_num_expanded += section_path_planner->num_expanded;
        LL_num_generated += section_path_planner->num_generated;

        if (path.empty())
        {
            std::cout << "NO SOLUTION EXISTS";
            return false;
        }

        dummy_start->paths.emplace_back(i, path);
        paths[i] = &dummy_start->paths.back().second;
        dummy_start->makespan = std::max<size_t>(dummy_start->makespan, paths[i]->back().timestep);
        dummy_start->g_val += path_cost;
	}
    find_conflicts(dummy_start->conflicts);
    if (!lazyPriority)
    {
        if(!find_consistent_paths(dummy_start, -1, mapsys))
            return false;
    }

	dummy_start->f_val = dummy_start->g_val;
    dummy_start->num_of_collisions = dummy_start->conflicts.size();
    min_f_val = dummy_start->f_val;
    // focal_list_threshold = min_f_val * focal_w;
    best_node = dummy_start;
    HL_num_generated++;
    dummy_start->time_generated = HL_num_generated;
    push_node(dummy_start);
    if (screen == 2)
    {
        double runtime = (double)(std::clock() - time) / CLOCKS_PER_SEC;
        std::cout << "Done! (" << runtime << "s)" << std::endl;
    }
    return true;
}

void PBSSection::push_node(PBSNodeSection* node)
{
    dfs.push_back(node);
    allNodes_table.push_back(node);
}

PBSNodeSection* PBSSection::pop_node()
{
    PBSNodeSection* node = dfs.back();
    dfs.pop_back();
    return node;
}

void PBSSection::update_best_node(PBSNodeSection* node)
{
    if (node->earliest_collision > best_node->earliest_collision or
        (node->earliest_collision == best_node->earliest_collision &&
            node->f_val < best_node->f_val))
        best_node = node;
}

bool PBSSection::run_section(const vector<SectionState>& start_sections,
                    const vector<vector<pair<SectionState, int> > > goal_sections,
                    int _time_limit, MapSystem* mapsys)
{
    clear();

    // set timer
	start = std::clock();
    
    this->start_sections = start_sections;
    this->goal_sections = goal_sections;
    this->num_of_agents = starts.size();
    this->time_limit = _time_limit;

    solution_cost = -2;
    solution_found = false;

    //rs.num_of_agents = num_of_agents;
    //rs.map_size = G.size();
    //rs.k_robust = k_robust;
    //rs.window = window;
	//rs.hold_endpoints = hold_endpoints;
    //section_path_planner.travel_times = travel_times;
	//section_path_planner.hold_endpoints = hold_endpoints;
	//section_path_planner.prioritize_start = prioritize_start;

    if (!generate_root_node(mapsys))
        return false;

    if (dummy_start->num_of_collisions == 0) //no conflicts at the root node
    {// found a solution (and finish the while look)
        solution_found = true;
        solution_cost = dummy_start->g_val;
        best_node = dummy_start;
    }

    // start the loop
	while (!dfs.empty() && !solution_found)
	{
		runtime = (double)(std::clock() - start)  / CLOCKS_PER_SEC;
        if (runtime > time_limit)
		{  // timeout
			solution_cost = -1;
			solution_found = false;
			break;
		}

		PBSNodeSection* curr = pop_node();
		update_paths(curr);

        if (curr->conflicts.empty())
        {// found a solution (and finish the while look)
            solution_found = true;
            solution_cost = curr->g_val;
            best_node = curr;
            break;
        }
	
		choose_conflict(*curr);

        update_best_node(curr);

		 //Expand the node
		HL_num_expanded++;

		curr->time_expanded = HL_num_expanded;
		if(screen == 2)
			std::cout << "Expand Node " << curr->time_generated << " ( cost = " << curr->f_val << " , #conflicts = " <<
			curr->num_of_collisions << " ) on conflict " << curr->conflict << std::endl;
		PBSNodeSection* n[2];
        for (auto & i : n)
                i = new PBSNodeSection();
	    resolve_conflict(curr->conflict, n[0], n[1]);

        vector<SectionPath*> copy(paths);
        for (auto & i : n)
        {
            bool sol = generate_child(i, curr, mapsys);
            if (sol)
            {
                HL_num_generated++;
                i->time_generated = HL_num_generated;
            }
            if (sol)
            {
                if (screen == 2)
                {
                    std::cout << "Generate #" << i->time_generated << " with "
                              << i->paths.size() << " new paths, "
                              << i->g_val - curr->g_val << " delta cost and "
                              << i->num_of_collisions << " conflicts " << std::endl;
                }
                if (i->f_val == min_f_val && i->num_of_collisions == 0) //no conflicts
                {// found a solution (and finish the while look)
                    solution_found = true;
                    solution_cost = i->g_val;
                    best_node = i;
                    allNodes_table.push_back(i);
                    break;
                }
            }
		    else
		    {
			    delete i;
			    i = nullptr;
		    }
		    paths = copy;
        }

        if (!solution_found)
        {
            if (n[0] != nullptr && n[1] != nullptr) // 두가지 노드 모두 생성
            {
                if (n[0]->f_val < n[1]->f_val ||
                    (n[0]->f_val == n[1]->f_val && n[0]->num_of_collisions < n[1]->num_of_collisions))
                {
                    push_node(n[1]);
                    push_node(n[0]);
                }
                else
                {
                    push_node(n[0]);
                    push_node(n[1]);
                }
            }
            else if (n[0] != nullptr)
            {
                push_node(n[0]);
            }
            else if (n[1] != nullptr)
            {
                push_node(n[1]);
            } else // 둘다 경로가 없는 경우
            {
                // std::cout << "*******A new nogood ********" << endl;
                nogood.emplace(curr->conflict.agent1, curr->conflict.agent2);
            }
            curr->clear();
        }
	}  // end of while loop


	runtime = (double)(std::clock() - start) / CLOCKS_PER_SEC;
    get_solution();
	if (solution_found && !validate_solution())
	{
        std::cout << "Solution invalid!!!" << std::endl;
        // print_paths();
        exit(-1);
	}
    min_sum_of_costs = 0;
    for (int i = 0; i < num_of_agents; i++)
    {
        int start_loc = starts[i].location;
        std::vector<double> dist_goal = mapsys->goal_to_goal_dist(goal_sections[i]);
        min_sum_of_costs += mapsys->compute_h_value(start_sections[i].section_id, start_sections[i].start_index, 0, goal_sections[i], dist_goal);
    }
	if (screen > 0) // 1 or 2
		print_results();
	return solution_found;
}


void PBSSection::resolve_conflict(const SectionConflict& conflict, PBSNodeSection* n1, PBSNodeSection* n2)
{
    int a1, a2;
    a1 = conflict.agent1;
    a2 = conflict.agent2;
    n1->priority = std::make_pair(a1, a2);
    n2->priority = std::make_pair(a2, a1);
}



PBSSection::PBSSection(const BasicGraph& G, SingleAgentSolver& path_planner) : MAPFSolver(G, path_planner),
        lazyPriority(false), best_node(nullptr) {}

PBSSection::PBSSection(const BasicGraph& G, SingleAgentSolver& path_planner, SIPPSection& section_path_planner) : MAPFSolver(G, path_planner, section_path_planner),
        lazyPriority(false), best_node(nullptr) {}


inline void PBSSection::release_closed_list()
{
	for (auto & it : allNodes_table)
		delete it;
	allNodes_table.clear();
}


PBSSection::~PBSSection()
{
	release_closed_list();
}


bool PBSSection::validate_solution()
{
    list<SectionConflict> conflict;
	for (int a1 = 0; a1 < num_of_agents; a1++)
	{
		for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
		{
            find_conflicts(conflict, a1, a2);
            if (!conflict.empty())
            {
                int a1_, a2_, si_, li_, t;
                ConflictType ty;

                a1_ = conflict.front().agent1;
                a2_ = conflict.front().agent2;
                si_ = conflict.front().section_id;
                li_ = conflict.front().local_index;
                t = conflict.front().timestep;
                ty = conflict.front().type;

                std::cout << "Agents "  << a1 << " and " << a2 << " collides at section: " << si_ << ", index: " << li_ <<
                " at timestep " << t << std::endl;

                return false;
            }
		}
	}
	return true;
}

void PBSSection::print_paths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		if (paths[i] == nullptr)
            continue;
        std::cout << "Agent " << i << " (" << paths[i]->back().timestep << "): ";
		for (const auto& s : (*paths[i]))
			std::cout << s.section_id << "->";
		std::cout << std::endl;
	}
}


// adding new nodes to FOCAL (those with min-f-val*f_weight between the old and new LB)
void PBSSection::update_focal_list()
{
	/*PBSNode* open_head = open_list.top();
	if (open_head->f_val > min_f_val)
	{
		if (screen == 2)
		{
			std::cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << open_list.size() << " to |FOCAL|=";
		}
		min_f_val = open_head->f_val;
		double new_focal_list_threshold = min_f_val * focal_w;
		for (PBSNode* n : open_list)
		{
			if (n->f_val > focal_list_threshold &&
				n->f_val <= new_focal_list_threshold)
				n->focal_handle = focal_list.push(n);
		}
		focal_list_threshold = new_focal_list_threshold;
		if (screen == 2)
		{
			std::cout << focal_list.size() << std::endl;
		}
	}*/
}

void PBSSection::update_CAT(int ex_ag)
{
    // size_t makespan = 0;
	// for (int ag = 0; ag < num_of_agents; ag++) 
    // {
    //     if (ag == ex_ag || paths[ag] == nullptr)
    //         continue;
    //     makespan = std::max(makespan, paths[ag]->size());
    // }
	// cat.clear();
    // cat.resize(makespan);
    // for (int t = 0; t < (int)makespan; t++)
    //     cat[t].resize(G.size(), false);

    // for (int ag = 0; ag < num_of_agents; ag++) 
	// {
    //     if (ag == ex_ag || paths[ag] == nullptr)
    //         continue; 
	// 	for (int timestep = 0; timestep < (int)paths[ag]->size() - 1; timestep++)
	// 	{
	// 		int loc = paths[ag]->at(timestep).location;
	// 		if (loc < 0)
	// 		    continue;
	// 		for (int t = max(0, timestep - k_robust); t <= timestep + k_robust; t++)
	// 		    cat[t][loc] = true;
	// 	}
	// }
}

void PBSSection::print_results() const
{
    std::cout << "PBSSection:";
	if(solution_cost >= 0) // solved
		std::cout << "Succeed,";
	else if(solution_cost == -1) // time_out
		std::cout << "Timeout,";
	else if(solution_cost == -2) // no solution
		std::cout << "No solutions,";
	else if (solution_cost == -3) // nodes out
		std::cout << "Nodesout,";

	std::cout << runtime << "," <<
        //LL_num_expanded << ", " << LL_num_generated << ", " <<
		HL_num_expanded << "," << HL_num_generated << "," <<
		solution_cost << "," << min_sum_of_costs << "," <<
		avg_path_length << "," << dummy_start->num_of_collisions << "," <<
		runtime_plan_paths << "," << runtime_rt << "," <<
		runtime_get_higher_priority_agents << "," <<
		runtime_copy_priorities << "," <<
		runtime_detect_conflicts << "," <<
		runtime_copy_conflicts << "," <<
		runtime_choose_conflict << "," <<
		runtime_find_consistent_paths << "," <<
		runtime_find_replan_agents <<
		std::endl;
}

void PBSSection::save_results(const std::string &fileName, const std::string &instanceName) const
{
	std::ofstream stats;
	stats.open(fileName, std::ios::app);
	stats << runtime << "," <<
		HL_num_expanded << "," << HL_num_generated << "," <<
		LL_num_expanded << "," << LL_num_generated << "," <<
		solution_cost << "," << min_sum_of_costs << "," <<
		avg_path_length << "," << dummy_start->num_of_collisions << "," <<
		instanceName << std::endl;
	stats.close();
}

void PBSSection::print_conflicts(const PBSNodeSection &curr)
{
	for (auto c : curr.conflicts)
	{
		std::cout << c << std::endl;
	}
}


void PBSSection::save_search_tree(const std::string &fname) const
{
    std::ofstream output;
    output.open(fname, std::ios::out);
    output << "digraph G {" << std::endl;
    output << "size = \"5,5\";" << std::endl;
    output << "center = true;" << std::endl;
    for (auto node : allNodes_table)
    {
        if (node == dummy_start)
            continue;
        else if (node->time_expanded == 0) // this node is in the openlist
            output << node->time_generated << " [color=blue]" << std::endl;
        output << node->parent->time_generated << " -> " << node->time_generated << std::endl;
    }
    output << "}" << std::endl;
    output.close();
}

void PBSSection::save_constraints_in_goal_node(const std::string &fileName) const
{
	best_node->priorities.save_as_digraph(fileName );
}

void PBSSection::get_solution()
{
    update_paths(best_node);
    section_solution.resize(num_of_agents);
    for (int k = 0; k < num_of_agents; k++)
    {
        section_solution[k] = *paths[k];
    }

    //solution_cost  = 0;
    avg_path_length = 0;

    for (int k = 0; k < num_of_agents; k++)
    {
        avg_path_length += paths[k]->back().timestep;
    }
    avg_path_length /= num_of_agents;
}

