#include "PBSSection.h"
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include "PathTableSection.h"



bool PBSSection::run(const vector<State>& starts,
        const vector< vector<pair<int, int> > >& goal_locations,  // an ordered list of pairs of <location, release time>
        int time_limit){
            std::cout << "Warning: PBSSection::run() called but not implemented." << std::endl;
            return false;
        }




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
    // Drop stale pointers from previous run_section — allNodes (and their
    // embedded SectionPaths) are freed in release_closed_list() below.
    std::fill(fc_cache_path_ptr.begin(), fc_cache_path_ptr.end(), nullptr);
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
	for (const auto& conflict : conflicts)
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
    for (const auto& conflict : conflicts)
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
    // std::cout << "find_conflicts agent: " << a1 << ", " << a2 << std::endl;
    clock_t t = clock();
    if (paths[a1] == nullptr || paths[a2] == nullptr)
        return;

    // std::cout << "paths size: " << paths[a2]->size() << std::endl;
    // std::cout << "paths timestep: " << paths[a2]->back().timestep << std::endl;
    // std::cout << "window: " << window << std::endl;
    // std::cout << "full_path: " << (*paths[a2])[0].full_path.size();
    // std::cout << "timestep: " << (*paths[a2])[1].timestep << std::endl;
    int size1 = min(window + 1, paths[a1]->back().timestep);
    int size2 = min(window + 1, paths[a2]->back().timestep);
    int sec_idx_1 = 0;
    int sec_idx_2 = 0;
    int sec_inside_idx_1 = 0;
    int sec_inside_idx_2 = 0;
    int section_id_1 = (*paths[a1])[sec_idx_1].section_id;
    int section_id_2 = (*paths[a2])[sec_idx_2].section_id;
    auto* internal_path_1 = &((*paths[a1])[sec_idx_1].full_path);
    auto* internal_path_2 = &((*paths[a2])[sec_idx_2].full_path);
    
    //std::cout << "max timestep" << size1 << std::endl;
    for (int timestep = 0; timestep < size1; timestep++)
    {
        if (size2 <= timestep)
            break;

        if (sec_idx_1 + 1 < paths[a1]->size() && (*paths[a1])[sec_idx_1+1].timestep == timestep){
            sec_idx_1 += 1;
            section_id_1 = (*paths[a1])[sec_idx_1].section_id;
            sec_inside_idx_1 = 0;
            internal_path_1 = &((*paths[a1])[sec_idx_1].full_path);
        }
        if (sec_idx_2 + 1 < paths[a2]->size() && (*paths[a2])[sec_idx_2+1].timestep == timestep){
            sec_idx_2 += 1;
            section_id_2 = (*paths[a2])[sec_idx_2].section_id;
            sec_inside_idx_2 = 0;
            internal_path_2 = &((*paths[a2])[sec_idx_2].full_path);
        }

        //std::cout << "timestep: " << timestep << ", internal_path_1 size: " << (*internal_path_1).size() << ", cur index: " << sec_inside_idx_1 << ", internal_path_2 size: " << (*internal_path_2).size() << ", cur index: " << sec_inside_idx_2 << std::endl;
        //std::cout << "timestep: " << timestep  << "agent " << a1 << ": (" << section_id_1 << ", " <<  (*internal_path_1)[sec_inside_idx_1].second << ")" << "// agent " << a2 << ": (" << section_id_2 << ", " << (*internal_path_2)[sec_inside_idx_2].second << ")" << std::endl; 

        if (section_id_1 == section_id_2){
            // 같은 위치(vertex)에 있다면 충돌!
            if ((*internal_path_1)[sec_inside_idx_1].second == (*internal_path_2)[sec_inside_idx_2].second){
                conflicts.emplace_back(a1, a2, section_id_1, (*internal_path_1)[sec_inside_idx_1].second, timestep, ConflictType::TILE_VERTEX);
                runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
                return;
            }
        }


        sec_inside_idx_1 += 1;
        sec_inside_idx_2 += 1;
    }

	runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}

void PBSSection::find_conflicts(list<SectionConflict>& conflicts, bool stop_after_first_ts)
{
    // Bucketing: at each timestep, group agents by (section_id, cell_idx).
    // For each new agent arriving in a bucket, emit pairwise conflict with
    // every prior occupant (captures 3-way+ collisions). First-time-only
    // semantics preserved via fc_pair_reported set.
    //
    // stop_after_first_ts: see header comment. choose_conflict picks the
    // earliest-timestep non-nogood conflict, so the first ts that produces
    // conflicts carries all that choose_conflict needs — unless those are
    // all nogood and a later-ts non-nogood exists. Caller guards on
    // nogood.empty() to avoid that case.
    clock_t t = clock();

    const int W = window + 1;

    fc_sizes.assign(num_of_agents, 0);
    fc_sec_idx.assign(num_of_agents, 0);
    fc_sec_inside_idx.assign(num_of_agents, 0);

    for (int a = 0; a < num_of_agents; a++) {
        if (paths[a] != nullptr && !paths[a]->empty())
            fc_sizes[a] = std::min(W, paths[a]->back().timestep);
    }

    fc_pair_reported.clear();
    for (auto& kv : fc_bucket) kv.second.clear();
    if (fc_bucket.bucket_count() < (size_t)num_of_agents * 2)
        fc_bucket.reserve(num_of_agents * 2);

    for (int ts = 0; ts < W; ts++)
    {
        for (auto& kv : fc_bucket) kv.second.clear();

        const size_t conflicts_before_ts = conflicts.size();

        for (int a = 0; a < num_of_agents; a++)
        {
            if (fc_sizes[a] <= ts) continue;

            auto& path_a = *paths[a];

            if (fc_sec_idx[a] + 1 < (int)path_a.size() && path_a[fc_sec_idx[a] + 1].timestep == ts) {
                fc_sec_idx[a] += 1;
                fc_sec_inside_idx[a] = 0;
            }

            const auto& state = path_a[fc_sec_idx[a]];
            const auto& fp = state.full_path;

            // full_path 범위 초과 시 마지막 셀에서 대기 중인 것으로 처리.
            // 이전 코드는 skip + double-increment 해서 이후 index가 전부 밀려
            // 충돌 감지를 놓치는 버그가 있었음.
            int cell_idx;
            if (fc_sec_inside_idx[a] >= (int)fp.size()) {
                cell_idx = fp.back().second;
            } else {
                cell_idx = fp[fc_sec_inside_idx[a]].second;
                fc_sec_inside_idx[a] += 1;
            }

            const int section_id = state.section_id;
            const uint32_t key = ((uint32_t)section_id << 4) | (uint32_t)cell_idx;

            auto& occ = fc_bucket[key];
            for (int other : occ) {
                int a1 = std::min(a, other);
                int a2 = std::max(a, other);
                uint64_t pkey = ((uint64_t)a1 << 32) | (uint32_t)a2;
                if (fc_pair_reported.insert(pkey).second) {
                    conflicts.emplace_back(a1, a2, section_id, cell_idx, ts, ConflictType::TILE_VERTEX);
                }
            }
            occ.push_back(a);
        }

        if (stop_after_first_ts && conflicts.size() > conflicts_before_ts)
            break;
    }

    runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}

// (agent-key caching removed — invalidation bug caused spurious conflict miss)

void PBSSection::build_keys_for_agent(int a)
{
    if (fc_W <= 0) return;
    const int base = a * fc_W;
    // Default: no key (indicates no conflict possible at that ts).
    for (int i = 0; i < fc_W; i++) fc_keys_flat[base + i] = UINT32_MAX;
    fc_keys_size[a] = 0;
    if (a < 0 || a >= num_of_agents) return;
    if (paths[a] == nullptr || paths[a]->empty()) return;
    auto& path_a = *paths[a];
    const int size_a = std::min(fc_W, path_a.back().timestep);
    if (size_a <= 0) return;
    int si = 0, sii = 0;
    for (int ts = 0; ts < size_a; ts++) {
        if (si + 1 < (int)path_a.size() && path_a[si + 1].timestep == ts) {
            si += 1; sii = 0;
        }
        const auto& state = path_a[si];
        const auto& fp = state.full_path;
        int cell_idx;
        if (sii >= (int)fp.size()) {
            cell_idx = fp.back().second;
        } else {
            cell_idx = fp[sii].second;
            sii += 1;
        }
        fc_keys_flat[base + ts] = ((uint32_t)state.section_id << 4) | (uint32_t)cell_idx;
    }
    fc_keys_size[a] = size_a;
}

void PBSSection::refresh_keys_cache()
{
    const int W = window + 1;
    const bool layout_stale =
        (fc_W != W) ||
        ((int)fc_keys_size.size() != num_of_agents) ||
        (fc_keys_flat.size() != (size_t)num_of_agents * (size_t)W);

    if (layout_stale) {
        fc_W = W;
        fc_keys_flat.assign((size_t)num_of_agents * fc_W, UINT32_MAX);
        fc_keys_size.assign(num_of_agents, 0);
        fc_cache_path_ptr.assign(num_of_agents, nullptr);
    }

    // Rebuild only rows whose underlying path pointer changed since last sync.
    for (int a = 0; a < num_of_agents; a++) {
        const SectionPath* cur = paths[a];
        if (fc_cache_path_ptr[a] != cur) {
            build_keys_for_agent(a);
            fc_cache_path_ptr[a] = cur;
        }
    }
}

void PBSSection::find_conflicts(list<SectionConflict>& new_conflicts, int new_agent)
{
    // Uses fc_keys_flat cache populated by find_consistent_paths and kept
    // fresh for the replanned agent via build_keys_for_agent(new_agent).
    clock_t t = clock();

    if (fc_W <= 0 || new_agent < 0 || new_agent >= num_of_agents) {
        runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
        return;
    }

    const int size_new = fc_keys_size[new_agent];
    if (size_new <= 0) {
        runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
        return;
    }

    const uint32_t* new_row = fc_keys_flat.data() + (size_t)new_agent * fc_W;

    for (int a = 0; a < num_of_agents; a++)
    {
        if (a == new_agent) continue;
        const int size_a = fc_keys_size[a];
        const int max_ts = std::min(size_a, size_new);
        if (max_ts <= 0) continue;
        const uint32_t* a_row = fc_keys_flat.data() + (size_t)a * fc_W;
        for (int ts = 0; ts < max_ts; ts++) {
            const uint32_t k = a_row[ts];
            if (k == new_row[ts] && k != UINT32_MAX) {
                // Decode section_id and cell_idx from key for conflict record.
                const int section_id = (int)(k >> 4);
                const int cell_idx = (int)(k & 0xFu);
                new_conflicts.emplace_back(new_agent, a, section_id, cell_idx, ts, ConflictType::TILE_VERTEX);
                break;
            }
        }
    }

    runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
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

// void PBSSection::choose_conflict(PBSNodeSection &node)
// {
//     clock_t t = clock();
// 	if (node.conflicts.empty())
// 	    return;

//     node.conflict = node.conflicts.front();

// 	// choose the earliest
//     for (auto conflict : node.conflicts)
//     {
//         if (conflict.timestep < node.conflict.timestep)
//             node.conflict = conflict;
//     }
//     node.earliest_collision = node.conflict.timestep;

//     if (!nogood.empty())
//     {
//         for (auto conflict : node.conflicts)
//         {
//             int a1 = conflict.agent1;
//             int a2 = conflict.agent2;
//             for (auto p : nogood)
//             {
//                 if ((a1 == p.first && a2 == p.second) || (a1 == p.second && a2 == p.first))
//                 {
//                     node.conflict = conflict;
//                     runtime_choose_conflict += (double)(std::clock() - t) / CLOCKS_PER_SEC;
//                     return;
//                 }
//             }
//         }
//     }

//     runtime_choose_conflict += (double)(std::clock() - t) / CLOCKS_PER_SEC;
// }

bool PBSSection::choose_conflict(PBSNodeSection &node)
{
    if (node.conflicts.empty()) return false;

    auto in_nogood = [&](const SectionConflict& c) {
        return nogood.count(std::make_pair(c.agent1, c.agent2)) ||
               nogood.count(std::make_pair(c.agent2, c.agent1));
    };

    // Opt 2: earliest-ts primary (stable with previous behavior);
    // tiebreak at the same earliest ts on the "most tangled" pair —
    // the one whose agents appear in more other non-nogood conflicts.
    // Resolving a tangled pair cascades further via priority edges.
    std::unordered_map<int, int> agent_deg;
    agent_deg.reserve(node.conflicts.size() * 2);
    for (const auto& c : node.conflicts) {
        if (in_nogood(c)) continue;
        agent_deg[c.agent1]++;
        agent_deg[c.agent2]++;
    }

    const SectionConflict* picked = nullptr;
    int best_ts = std::numeric_limits<int>::max();
    int best_score = -1;
    for (const auto& c : node.conflicts) {
        if (in_nogood(c)) continue;
        const int score = agent_deg[c.agent1] + agent_deg[c.agent2];
        if (c.timestep < best_ts ||
            (c.timestep == best_ts && score > best_score)) {
            best_ts = c.timestep;
            best_score = score;
            picked = &c;
        }
    }
    if (picked == nullptr) return false;

    node.conflict = *picked;
    node.earliest_collision = node.conflict.timestep;
    return true;
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
    // std::cout << "🔙 [run_section 반환] agent=" << agent << " path.size()=" << path.size() << "\n";

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
        unordered_set<int>& replan, const unordered_set<int>& /*already_replanned*/)
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

        // connected(x, y) == true 면 x < y (x가 더 낮은 priority).
        bool a1_lower = node->priorities.connected(a1, a2);
        bool a2_lower = node->priorities.connected(a2, a1);

        // prioritize_start는 건너뛴다: 섹션 기반 PBS에서는 start 셀 보호가
        // priority DAG을 어겨 ping-pong을 유발한다. start에 머무는 agent는
        // 일반 priority 규칙으로도 충분히 보호받는다.
        if (a1_lower)
        {
            replan.insert(a1);
            continue;
        }
        if (a2_lower)
        {
            replan.insert(a2);
            continue;
        }
        // 우선순위 관계가 없는 충돌은 high-level(PBS 트리)가 분기로 해결한다.
    }
    runtime_find_replan_agents += (double)(std::clock() - t2) / CLOCKS_PER_SEC;
}


bool PBSSection::find_consistent_paths(PBSNodeSection* node, int agent, MapSystem* mapsys)
{
    clock_t t = clock();
    int count = 0; // count the times that we call the low-level search.
    unordered_set<int> replan;
    unordered_set<int> already_replanned;
    unordered_map<int, int> replan_count;
    // Ping-pong detection: 직전에 replan된 agent를 기억해뒀다가 다음 pop에서
    // 같은 agent가 올라오면 탈출한다. 사이에 다른 agent가 replan되지 않았다는
    // 뜻이고, 그 agent의 rs는 변하지 않았으므로 SIPP는 같은 결과를 낸다.
    int last_replanned = -1;
    if (agent >= 0 && agent < num_of_agents)
        replan.insert(agent);
    find_replan_agents(node, node->conflicts, replan, already_replanned);

    // Opt C: refresh per-agent key cache incrementally. Only rows whose
    // underlying path pointer changed since last sync get rebuilt — the
    // rest survive across find_consistent_paths calls. Replanned agents
    // inside the loop below also update their row via build_keys_for_agent.
    refresh_keys_cache();

    

    // node->paths는 이 노드에서 "새로 replan된" agent만 담는 list라 child에선 매우
     // 작다. 전체 count 한계는 num_of_agents 기반으로 둬야 의미 있는 conflict chain을
     // 허용한다.
    const int count_limit = num_of_agents * 5;
    while (!replan.empty())
    {
        if (count > count_limit)
        {
            runtime_find_consistent_paths += (double)(std::clock() - t) / CLOCKS_PER_SEC;
            return false;
        }
        int a = *replan.begin();

        // Ping-pong: 바로 직전에 replan한 agent가 다시 올라왔다는 건, 그 사이에
        // 다른 agent를 replan하지 않았다는 뜻. rs가 바뀔 일이 없으니 SIPP는 같은
        // path를 내놓고 같은 conflict가 생기는 deterministic loop → 탈출.
        if (a == last_replanned) {
            return false;
        }
        replan.erase(a);
        replan_count[a]++;
        if (replan_count[a] > 10) {
            return false;
        }
        last_replanned = a;

        count++;

        if (!find_path(node, a, mapsys))
        {
            return false;
        }

        //already_replanned.insert(a);

        // std::cout << "✅ [find_path 성공] agent=" << a << "\n";

        // find_path updated paths[a]; refresh cached key row so subsequent
        // find_conflicts sees the new path, and record new pointer so
        // refresh_keys_cache on next fcp entry won't redundantly rebuild.
        build_keys_for_agent(a);
        fc_cache_path_ptr[a] = paths[a];

        remove_conflicts(node->conflicts, a);
        list<SectionConflict> new_conflicts;
        find_conflicts(new_conflicts, a);

        // std::cout << "   새 conflict 수=" << new_conflicts.size() << "\n";
        // for (auto& c : new_conflicts)
        //     std::cout << "   conflict: a1=" << c.agent1 << " a2=" << c.agent2 
        //             << " sec=" << c.section_id << " t=" << c.timestep << "\n";

        find_replan_agents(node, new_conflicts, replan, already_replanned);

        // for (int done : already_replanned)
        //     replan.erase(done);

        // for (auto& c : new_conflicts) {
        //     std::cout << "   우선순위 체크: (" << c.agent1 << ">" << c.agent2 << ")=" 
        //             << node->priorities.connected(c.agent1, c.agent2)
        //             << " (" << c.agent2 << ">" << c.agent1 << ")="
        //             << node->priorities.connected(c.agent2, c.agent1) << "\n";
        // }

        // std::cout << "   replan 남은 agents: ";
        // for (int x : replan) std::cout << x << " ";
        // std::cout << "\n";


        node->conflicts.splice(node->conflicts.end(), new_conflicts);

       // already_replanned.clear();
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

    // 방금 이 child에서 강제한 우선순위(priority.first < priority.second)를
    // cascade replan 이후에도 실제로 지키지 못한 경우 — 즉 같은 pair가 다시
    // conflict 리스트에 남아있으면 — 이 fork는 진전을 못 만든다. 실패 처리해서
    // 두 방향 모두 실패 시 상위 로직이 pair를 nogood에 등록하게 한다.
    {
        const int pa = node->priority.first;
        const int pb = node->priority.second;
        for (const auto& c : node->conflicts) {
            if ((c.agent1 == pa && c.agent2 == pb) ||
                (c.agent1 == pb && c.agent2 == pa)) {
                return false;
            }
        }
    }

    node->num_of_collisions = node->conflicts.size();

	//Estimate h value
	node->h_val = 0;
	node->f_val = node->g_val + node->h_val;

	return true;
}


bool PBSSection::generate_root_node(MapSystem* mapsys)
{
   // std::cout << "--- generate_root_node Start ---" << std::endl;
    
    clock_t time = std::clock();
	dummy_start = new PBSNodeSection();
    
	// initialize paths_found_initially
	paths.resize(num_of_agents, nullptr);
    //std::cout << "initial_path resize?" << num_of_agents << std::endl;
	
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
        if (paths[i] != nullptr){
            continue;
        }
        SectionPath path;
        double path_cost;
        int start_section_id  = start_sections[i].section_id;
        int start_section_index = start_sections[i].start_index;
        clock_t t = std::clock();
		rs.clear();
        rs.build(paths, dummy_start->priorities.get_reachable_nodes(i), mapsys);
        runtime_get_higher_priority_agents += dummy_start->priorities.runtime;
        runtime_rt += (double)(std::clock() - t) / CLOCKS_PER_SEC;
        //std::cout << "here????" << std::endl;
        vector< vector<double> > h_values(goal_sections[i].size());
        t = std::clock();
        //std::cout << "here????" << std::endl;
        path = section_path_planner->run_section(start_sections[i], goal_sections[i], rs, i, 8, mapsys);
		runtime_plan_paths += (double)(std::clock() - t) / CLOCKS_PER_SEC;
        //std::cout << "path cost of agent " << i << ", " << section_path_planner->path_cost << std::endl;
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
    // Full scan at root: we want find_consistent_paths (line below) to see
    // every initial conflict so early replan commits more priorities upfront.
    // Measured: truncating here cost +4–6% HL expansion.
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

    // Reset SIPPSection profiling timers (누적 across all find_path calls in this run_section)
    if (section_path_planner != nullptr)
    {
        auto* sp = section_path_planner;
        sp->t_find_wait_list = 0; sp->t_compute_h = 0;
        sp->t_safe_intervals = 0; sp->t_generate_node = 0;
        sp->t_focal_update = 0; sp->t_update_path = 0;
        sp->fwl_calls = 0; sp->gn_calls = 0;
    }

    this->start_sections = start_sections;
    this->goal_sections = goal_sections;
    this->num_of_agents = start_sections.size();
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
        //std::cout << "dummy end?" << std::endl;
        solution_found = true;
        solution_cost = dummy_start->g_val;
        best_node = dummy_start;
    }

    int hl_iteration = 0;
    int iter_nogood_became_nonempty = -1;
    // Nogood budget tunable via RHCR_NOGOOD_BUDGET env var; default 50000.
    static int nogood_budget = -1;
    if (nogood_budget < 0) {
        const char* env = std::getenv("RHCR_NOGOOD_BUDGET");
        nogood_budget = (env && *env) ? std::atoi(env) : 50000;
    }
    // start the loop
	while (!dfs.empty() && !solution_found)
	{
        if (++hl_iteration > 100000) {
            break;
        }

        // Fast-fail: nogood이 발생한 뒤 제한된 예산만 더 쓰고 진전이 없으면 포기.
        if (!nogood.empty()) {
            if (iter_nogood_became_nonempty < 0)
                iter_nogood_became_nonempty = hl_iteration;
            if (hl_iteration - iter_nogood_became_nonempty > nogood_budget) {
                solution_cost = -2;
                solution_found = false;
                break;
            }
        }



		runtime = (double)(std::clock() - start)  / CLOCKS_PER_SEC;
        if (runtime > time_limit)
		{  // timeout
			solution_cost = -1;
			solution_found = false;
			break;
		}

		PBSNodeSection* curr = pop_node();

        if (curr->depth > num_of_agents * 2) {
            // node는 push_node 단계에서 allNodes_table에 등록되어 있다.
            // 여기서 delete 하면 allNodes_table에 dangling pointer가 남고,
            // 다음 run_section의 release_closed_list()가 double-free한다.
            // skip만 하고 정리는 release_closed_list에 위임.
            continue;
        }
		update_paths(curr);

        if (curr->conflicts.empty())
        {
            // Opt 1: trust incremental conflict tracking (backed by the
            // per-agent key cache in find_consistent_paths) and skip the
            // former O(N·W) bucketing safety-net scan. validate_solution()
            // at end-of-run is the final correctness check.
            solution_found = true;
            solution_cost = curr->g_val;
            best_node = curr;
            break;
        }
	
		if (!choose_conflict(*curr))
        {
            // 모든 conflict가 nogood — 이 노드는 어떤 분기로도 진전을 만들 수 없다.
            // dfs에 남은 다른 경로를 탐색하러 넘어간다. (node는 allNodes_table이
            // 소유하므로 delete 하지 않는다.)
            continue;
        }

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
            // if (sol) {
            //     std::cout << "✅ [child 생성] depth=" << i->depth 
            //             << " conflicts=" << i->num_of_collisions << "\n";
            // } else {
            //     std::cout << "❌ [child 실패] depth=" << curr->depth + 1 << "\n";
            // }

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
                // Opt 3 (Variant E): DFS pops from the back, so the child we
                // push LAST is expanded first.
                //
                // Use num_collisions as the primary key ONLY when the two
                // children's collision counts differ meaningfully (|Δc| ≥ 2).
                // Otherwise fall back to f_val primary / collisions tiebreak
                // (the original RHCR ordering).
                //
                // Rationale: a pure collisions-primary rule (earlier attempt)
                // halved w=10 wall but exploded w=5 primary-fail rate 5.5×.
                // The low-collision child at w=5 often commits priority edges
                // that later force nogood; f_val (=g_val, cost) is a gentler
                // signal when the two children are close. Only when one child
                // is clearly closer to feasible do we trust collisions.
                const int c0 = n[0]->num_of_collisions;
                const int c1 = n[1]->num_of_collisions;
                const int dc = c0 > c1 ? c0 - c1 : c1 - c0;
                const int kCollisionMargin = 2;

                bool prefer_n0;
                if (dc >= kCollisionMargin) {
                    prefer_n0 = (c0 < c1);
                } else {
                    prefer_n0 = (n[0]->f_val < n[1]->f_val ||
                                 (n[0]->f_val == n[1]->f_val && c0 < c1));
                }

                if (prefer_n0)
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
        //int start_loc = starts[i].location;
        std::vector<double> dist_goal = mapsys->goal_to_goal_dist(goal_sections[i]);
        min_sum_of_costs += mapsys->compute_h_value(start_sections[i].section_id, start_sections[i].start_index, 0, goal_sections[i], dist_goal);
    }
	// if (screen > 0) // 1 or 2
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

