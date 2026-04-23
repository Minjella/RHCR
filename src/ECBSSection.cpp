#include "ECBSSection.h"
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

// Grid-space run(): 섹션 전용 솔버이므로 no-op.
bool ECBSSection::run(const vector<State>& /*starts*/,
                     const vector<vector<pair<int, int>>>& /*goal_locations*/,
                     int /*time_limit*/)
{
    std::cerr << "[ECBSSection] run() called on section-only solver." << std::endl;
    return false;
}

// --- clear / update_paths ---------------------------------------------------

void ECBSSection::clear()
{
    runtime = 0;
    runtime_rt = 0;
    runtime_plan_paths = 0;
    runtime_detect_conflicts = 0;
    runtime_copy_conflicts = 0;
    runtime_choose_conflict = 0;
    runtime_update_focal = 0;

    HL_num_expanded = 0;
    HL_num_generated = 0;
    LL_num_expanded = 0;
    LL_num_generated = 0;
    safety_net_fires = 0;
    safety_net_conflicts_found = 0;

    solution_found = false;
    solution_cost = -2;
    min_f_val = 0;
    focal_threshold = 0;
    avg_path_length = -1;

    paths.clear();
    path_min_costs.clear();
    path_costs.clear();

    open_list.clear();
    focal_list.clear();
    release_closed_list();

    start_sections.clear();
    goal_sections.clear();
    best_node = nullptr;
    dummy_start = nullptr;
    current_mapsys = nullptr;

    // Drop stale pointers from previous run_section — allNodes released above.
    std::fill(fc_cache_path_ptr.begin(), fc_cache_path_ptr.end(), nullptr);
}

void ECBSSection::update_paths(ECBSNodeSection* curr)
{
    std::vector<bool> updated(num_of_agents, false);
    while (curr != nullptr)
    {
        for (auto& p : curr->paths)
        {
            int agent = std::get<0>(p);
            if (!updated[agent])
            {
                paths[agent] = &std::get<1>(p);
                path_min_costs[agent] = std::get<2>(p);
                path_costs[agent] = std::get<3>(p);
                updated[agent] = true;
            }
        }
        curr = curr->parent;
    }
}

double ECBSSection::get_path_cost(const SectionPath& path) const
{
    double cost = 0;
    for (int i = 0; i < (int)path.size() - 1; i++)
        cost += path[i].full_path.size();
    return cost;
}

// --- conflict detection (cell-level, PBSSection 버킷팅 이식) ------------------

void ECBSSection::copy_conflicts(const std::list<SectionConflict>& conflicts,
                                 std::list<SectionConflict>& copy,
                                 const std::list<int>& excluded_agents) const
{
    for (const auto& c : conflicts)
    {
        bool skip = false;
        for (int a : excluded_agents)
        {
            if (a == c.agent1 || a == c.agent2) { skip = true; break; }
        }
        if (!skip) copy.push_back(c);
    }
}

void ECBSSection::remove_conflicts(std::list<SectionConflict>& conflicts, int excluded_agent) const
{
    for (auto it = conflicts.begin(); it != conflicts.end(); )
    {
        if (it->agent1 == excluded_agent || it->agent2 == excluded_agent)
            it = conflicts.erase(it);
        else
            ++it;
    }
}

void ECBSSection::find_conflicts(std::list<SectionConflict>& conflicts, int a1, int a2)
{
    clock_t t = clock();
    if (paths[a1] == nullptr || paths[a2] == nullptr) return;

    int size1 = std::min(window + 1, paths[a1]->back().timestep);
    int size2 = std::min(window + 1, paths[a2]->back().timestep);
    int sec_idx_1 = 0, sec_idx_2 = 0;
    int sec_inside_idx_1 = 0, sec_inside_idx_2 = 0;

    for (int timestep = 0; timestep < size1; timestep++)
    {
        if (size2 <= timestep) break;

        if (sec_idx_1 + 1 < (int)paths[a1]->size() &&
            (*paths[a1])[sec_idx_1 + 1].timestep == timestep)
        {
            sec_idx_1 += 1;
            sec_inside_idx_1 = 0;
        }
        if (sec_idx_2 + 1 < (int)paths[a2]->size() &&
            (*paths[a2])[sec_idx_2 + 1].timestep == timestep)
        {
            sec_idx_2 += 1;
            sec_inside_idx_2 = 0;
        }

        const auto& st1 = (*paths[a1])[sec_idx_1];
        const auto& st2 = (*paths[a2])[sec_idx_2];
        const auto& fp1 = st1.full_path;
        const auto& fp2 = st2.full_path;

        int cell1 = sec_inside_idx_1 >= (int)fp1.size() ? (fp1.empty() ? -1 : fp1.back().second)
                                                       : fp1[sec_inside_idx_1].second;
        int cell2 = sec_inside_idx_2 >= (int)fp2.size() ? (fp2.empty() ? -1 : fp2.back().second)
                                                       : fp2[sec_inside_idx_2].second;

        if (st1.section_id == st2.section_id && cell1 == cell2 && cell1 >= 0)
        {
            conflicts.emplace_back(a1, a2, st1.section_id, cell1, timestep, ConflictType::TILE_VERTEX);
            runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
            return;
        }

        if (sec_inside_idx_1 < (int)fp1.size()) sec_inside_idx_1 += 1;
        if (sec_inside_idx_2 < (int)fp2.size()) sec_inside_idx_2 += 1;
    }
    runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}

void ECBSSection::find_conflicts(std::list<SectionConflict>& conflicts)
{
    // Opt B (ported from PBSSection): reuse class-member scratch buffers to
    // avoid per-call heap allocation of the sizes/section-index arrays and
    // the bucket/pair_reported hash containers.
    clock_t t = clock();
    const int W = window + 1;

    fc_sizes.assign(num_of_agents, 0);
    fc_sec_idx.assign(num_of_agents, 0);
    fc_sec_inside_idx.assign(num_of_agents, 0);

    for (int a = 0; a < num_of_agents; a++)
        if (paths[a] != nullptr && !paths[a]->empty())
            fc_sizes[a] = std::min(W, paths[a]->back().timestep);

    fc_pair_reported.clear();
    for (auto& kv : fc_bucket) kv.second.clear();
    if (fc_bucket.bucket_count() < (size_t)num_of_agents * 2)
        fc_bucket.reserve(num_of_agents * 2);

    for (int ts = 0; ts < W; ts++)
    {
        for (auto& kv : fc_bucket) kv.second.clear();

        for (int a = 0; a < num_of_agents; a++)
        {
            if (fc_sizes[a] <= ts) continue;
            auto& path_a = *paths[a];

            if (fc_sec_idx[a] + 1 < (int)path_a.size() && path_a[fc_sec_idx[a] + 1].timestep == ts)
            {
                fc_sec_idx[a] += 1;
                fc_sec_inside_idx[a] = 0;
            }

            const auto& state = path_a[fc_sec_idx[a]];
            const auto& fp = state.full_path;
            if (fp.empty()) continue;

            int cell_idx;
            if (fc_sec_inside_idx[a] >= (int)fp.size()) cell_idx = fp.back().second;
            else { cell_idx = fp[fc_sec_inside_idx[a]].second; fc_sec_inside_idx[a] += 1; }

            const uint32_t key = ((uint32_t)state.section_id << 4) | (uint32_t)cell_idx;

            auto& occ = fc_bucket[key];
            for (int other : occ)
            {
                int a1 = std::min(a, other);
                int a2 = std::max(a, other);
                uint64_t pkey = ((uint64_t)a1 << 32) | (uint32_t)a2;
                if (fc_pair_reported.insert(pkey).second)
                    conflicts.emplace_back(a1, a2, state.section_id, cell_idx, ts, ConflictType::TILE_VERTEX);
            }
            occ.push_back(a);
        }
    }

    runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}

void ECBSSection::build_keys_for_agent(int a)
{
    if (fc_W <= 0) return;
    const int base = a * fc_W;
    for (int i = 0; i < fc_W; i++) fc_keys_flat[base + i] = UINT32_MAX;
    fc_keys_size[a] = 0;
    if (a < 0 || a >= num_of_agents) return;
    if (paths[a] == nullptr || paths[a]->empty()) return;
    auto& path_a = *paths[a];
    const int size_a = std::min(fc_W, path_a.back().timestep);
    if (size_a <= 0) return;
    int si = 0, sii = 0;
    for (int ts = 0; ts < size_a; ts++)
    {
        if (si + 1 < (int)path_a.size() && path_a[si + 1].timestep == ts) { si += 1; sii = 0; }
        const auto& state = path_a[si];
        const auto& fp = state.full_path;
        if (fp.empty()) continue;
        int cell_idx;
        if (sii >= (int)fp.size()) cell_idx = fp.back().second;
        else { cell_idx = fp[sii].second; sii += 1; }
        fc_keys_flat[base + ts] = ((uint32_t)state.section_id << 4) | (uint32_t)cell_idx;
    }
    fc_keys_size[a] = size_a;
}

void ECBSSection::refresh_keys_cache()
{
    const int W = window + 1;
    const bool layout_stale =
        (fc_W != W) ||
        ((int)fc_keys_size.size() != num_of_agents) ||
        (fc_keys_flat.size() != (size_t)num_of_agents * (size_t)W);
    if (layout_stale)
    {
        fc_W = W;
        fc_keys_flat.assign((size_t)num_of_agents * fc_W, UINT32_MAX);
        fc_keys_size.assign(num_of_agents, 0);
        fc_cache_path_ptr.assign(num_of_agents, nullptr);
    }
    for (int a = 0; a < num_of_agents; a++)
    {
        const SectionPath* cur = paths[a];
        if (fc_cache_path_ptr[a] != cur)
        {
            build_keys_for_agent(a);
            fc_cache_path_ptr[a] = cur;
        }
    }
}

void ECBSSection::find_conflicts(std::list<SectionConflict>& new_conflicts, int new_agent)
{
    // Opt C-v2 (ported): use the per-agent key cache (fc_keys_flat) instead of
    // re-traversing each agent's SectionPath on every call. Caller must have
    // ensured the cache is fresh (refresh_keys_cache() on generate_child entry,
    // build_keys_for_agent(a) after find_path(a) succeeds).
    clock_t t = clock();

    if (fc_W <= 0 || new_agent < 0 || new_agent >= num_of_agents)
    {
        runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
        return;
    }
    const int size_new = fc_keys_size[new_agent];
    if (size_new <= 0)
    {
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
        for (int ts = 0; ts < max_ts; ts++)
        {
            const uint32_t k = a_row[ts];
            if (k == new_row[ts] && k != UINT32_MAX)
            {
                const int a1 = std::min(new_agent, a);
                const int a2 = std::max(new_agent, a);
                const int section_id = (int)(k >> 4);
                const int cell_idx = (int)(k & 0xFu);
                new_conflicts.emplace_back(a1, a2, section_id, cell_idx, ts, ConflictType::TILE_VERTEX);
                break;
            }
        }
    }

    runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}

void ECBSSection::find_conflicts(const std::list<SectionConflict>& old_conflicts,
                                 std::list<SectionConflict>& new_conflicts,
                                 const std::list<int>& new_agents)
{
    copy_conflicts(old_conflicts, new_conflicts, new_agents);
    for (int a : new_agents) find_conflicts(new_conflicts, a);
}

void ECBSSection::choose_conflict(ECBSNodeSection& node) const
{
    if (node.conflicts.empty()) return;

    // Primary key: earliest timestep (standard ECBS). Tiebreak: "most
    // tangled" pair — agents that appear in more other conflicts in the
    // same node. Resolving a tangled pair tends to cascade through the
    // priority/constraint structure and prune more future forks.
    // (Same heuristic as PBSSection Opt 2.)
    std::unordered_map<int, int> agent_deg;
    agent_deg.reserve(node.conflicts.size() * 2);
    for (const auto& c : node.conflicts) {
        agent_deg[c.agent1]++;
        agent_deg[c.agent2]++;
    }

    const SectionConflict* picked = &node.conflicts.front();
    int best_ts = picked->timestep;
    int best_score = agent_deg[picked->agent1] + agent_deg[picked->agent2];
    for (const auto& c : node.conflicts) {
        const int score = agent_deg[c.agent1] + agent_deg[c.agent2];
        if (c.timestep < best_ts ||
            (c.timestep == best_ts && score > best_score)) {
            best_ts = c.timestep;
            best_score = score;
            picked = &c;
        }
    }
    node.conflict = *picked;
}

// --- Entry-time helper ------------------------------------------------------

const SectionState* ECBSSection::find_state_at_time(int agent, int timestep) const
{
    if (agent < 0 || agent >= num_of_agents) return nullptr;
    if (paths[agent] == nullptr || paths[agent]->empty()) return nullptr;
    const auto& path = *paths[agent];
    // path[i].timestep = section entry time. 해당 timestep이 속하는 state는
    // state[i].timestep <= ts < state[i+1].timestep (마지막이면 ∞로 간주).
    for (int i = 0; i < (int)path.size(); ++i)
    {
        int entry_t = path[i].timestep;
        int exit_t = (i + 1 < (int)path.size()) ? path[i + 1].timestep : INT_MAX;
        if (entry_t <= timestep && timestep < exit_t) return &path[i];
    }
    return nullptr;
}

// --- Constraint collection --------------------------------------------------

void ECBSSection::collect_constraints(ECBSNodeSection* node, int agent,
                                      std::list<SectionConstraint>& out) const
{
    ECBSNodeSection* curr = node;
    while (curr != nullptr)
    {
        for (const auto& c : curr->constraints)
        {
            if (disjoint_splitting)
            {
                if (c.agent == agent) out.push_back(c);
                else if (c.positive)
                    out.emplace_back(agent, c.section_id, c.cell_idx, c.timestep, false);
            }
            else if (c.agent == agent)
            {
                out.push_back(c);
            }
        }
        curr = curr->parent;
    }
}

// --- Low-level --------------------------------------------------------------

bool ECBSSection::find_path(ECBSNodeSection* node, int agent, MapSystem* mapsys)
{
    SectionPath path;
    double path_cost = 0;
    double path_lb = 0;

    clock_t t = std::clock();

    std::list<SectionConstraint> my_constraints;
    collect_constraints(node, agent, my_constraints);

    // Pure-CBS: rs 비어있고 이 agent의 constraint만 cell_table에 sentinel로 심음.
    // Constraint는 대부분 "section-entry" 의미: (section_id, entry_cell, entry_time)
    // → add_cell_constraint(entry_time, section_id, entry_cell)이면 SIPP이
    // is_cell_safe 체크로 해당 time에 해당 entry cell 진입 자동 차단.
    // Start-section 충돌이면 resolve_conflict가 conflict cell/time 그대로
    // 넣었을 것 — 동일 로직으로 cell 차단됨.
    // RHCR_ECBS_CONSTRAINT_MODE:
    //   0 (default) cell-specific (current behavior)
    //   1           section entry cells (0,2,6,8) — 3x3 section 4-entry block
    //   2           whole section (all 9 cells) — most aggressive, often
    //               over-restricts and tanks primary success rate.
    // The cell-level start-section fallback (cell_idx==-1 sentinel convention
    // is not used here — resolve_for_agent keeps a real cell_idx for that
    // fallback) is unaffected: it uses add_cell_constraint.
    static int constraint_mode = -1;
    if (constraint_mode < 0) {
        const char* e = std::getenv("RHCR_ECBS_CONSTRAINT_MODE");
        constraint_mode = (e && *e) ? std::atoi(e) : 0;
    }

    rs.init_empty();
    for (const auto& c : my_constraints)
    {
        if (c.positive) {
            // Disjoint splitting의 positive 쪽: "agent가 이 time에 이 section
            // 안에 있다면 반드시 cell_idx에 있어야 함". 이걸 구현하려면 section
            // 내 cell_idx를 제외한 모든 cell을 negative로 막으면 됨. SIPP 수정
            // 불필요 (기존 is_cell_safe만 사용).
            rs.add_section_complement_constraint(c.timestep, c.section_id, c.cell_idx);
            continue;
        }
        if (constraint_mode == 2) {
            rs.add_section_constraint(c.timestep, c.section_id);
        } else if (constraint_mode == 1) {
            rs.add_section_entry_constraint(c.timestep, c.section_id);
        } else {
            rs.add_cell_constraint(c.timestep, c.section_id, c.cell_idx);
        }
    }
    runtime_rt += (double)(std::clock() - t) / CLOCKS_PER_SEC;

    t = std::clock();
    path = section_path_planner->run_section(start_sections[agent], goal_sections[agent],
                                             rs, agent, 8, mapsys);
    runtime_plan_paths += (double)(std::clock() - t) / CLOCKS_PER_SEC;

    path_cost = section_path_planner->path_cost;
    path_lb   = section_path_planner->min_f_val;
    if (path_lb <= 0) path_lb = path_cost;

    LL_num_expanded += section_path_planner->num_expanded;
    LL_num_generated += section_path_planner->num_generated;
    rs.clear();

    if (path.empty()) return false;

    // 기존 entry 삭제 후 새 path 삽입. node->paths는 이 CT node에서 갱신된
    // agent들의 덮어쓰기 레코드로만 사용 (parent chain을 통해 update_paths가 추적).
    for (auto it = node->paths.begin(); it != node->paths.end(); ++it)
    {
        if (std::get<0>(*it) == agent)
        {
            node->paths.erase(it);
            break;
        }
    }
    node->g_val     = node->g_val     - path_costs[agent]     + path_cost;
    node->min_f_val = node->min_f_val - path_min_costs[agent] + path_lb;

    node->paths.emplace_back(agent, std::move(path), path_lb, path_cost);
    paths[agent]          = &std::get<1>(node->paths.back());
    path_costs[agent]     = path_cost;
    path_min_costs[agent] = path_lb;
    return true;
}

// --- Branching --------------------------------------------------------------

void ECBSSection::resolve_for_agent(int agent, const SectionConflict& c, ECBSNodeSection* node)
{
    const SectionState* st = find_state_at_time(agent, c.timestep);

    // Start section fallback: agent의 첫 state (timestep==0, section==start)에서
    // 발생한 conflict면 entry block이 무의미하므로 conflict cell/time block.
    bool use_entry = (st != nullptr) && !(st->timestep == 0 &&
                    agent >= 0 && agent < (int)start_sections.size() &&
                    start_sections[agent].section_id == c.section_id);

    if (use_entry)
    {
        // Section-entry constraint: (agent, section, entry_cell, entry_time).
        // add_cell_constraint가 time/section/cell 조합으로 저장하므로
        // SectionConstraint의 (cell_idx, timestep) 필드에 entry_cell, entry_time을 담음.
        node->constraints.emplace_back(agent, c.section_id, st->start_index, st->timestep, false);
    }
    else
    {
        // Cell-level fallback (start section).
        node->constraints.emplace_back(agent, c.section_id, c.local_index, c.timestep, false);
    }
}

void ECBSSection::resolve_conflict(const SectionConflict& c, ECBSNodeSection* n1, ECBSNodeSection* n2)
{
    // Default (and historical) branching: both children get a negative
    // constraint, one for a1 and one for a2. Agent whose constraint is
    // added replans to avoid the (section,cell,time). Easy for the agent
    // to bypass via a different entry cell → CT stagnates when entry cells
    // are interchangeable.
    //
    // Optional disjoint splitting (RHCR_ECBS_DISJOINT=1):
    //   n1: "a1 NOT at (section, cell, time)" (negative, as before)
    //   n2: "a1 MUST be at (section, cell, time)" (positive)
    // Positive forces every OTHER agent (including a2) whose current plan
    // puts them at that cell/time to replan. Stronger branching step.
    static int disjoint = -1;
    if (disjoint < 0) {
        const char* e = std::getenv("RHCR_ECBS_DISJOINT");
        disjoint = (e && *e && e[0] != '0') ? 1 : 0;
    }

    if (disjoint)
    {
        // Both children impose something on a1. Fork on agent1 always;
        // alternative heuristics (most-constrained agent) could pick either.
        resolve_for_agent(c.agent1, c, n1);

        const SectionState* st = find_state_at_time(c.agent1, c.timestep);
        const bool start_sec = (st != nullptr) && (st->timestep == 0) &&
                               c.agent1 >= 0 &&
                               c.agent1 < (int)start_sections.size() &&
                               start_sections[c.agent1].section_id == c.section_id;
        if (st != nullptr && !start_sec)
        {
            // Positive section-entry constraint for a1.
            n2->constraints.emplace_back(c.agent1, c.section_id, st->start_index,
                                         st->timestep, /*positive=*/true);
        }
        else
        {
            // Start-section fallback: positive at conflict cell/time.
            n2->constraints.emplace_back(c.agent1, c.section_id, c.local_index,
                                         c.timestep, /*positive=*/true);
        }
    }
    else
    {
        resolve_for_agent(c.agent1, c, n1);
        resolve_for_agent(c.agent2, c, n2);
    }
}

bool ECBSSection::generate_child(ECBSNodeSection* node, ECBSNodeSection* parent, MapSystem* mapsys)
{
    node->parent = parent;
    node->g_val = parent->g_val;
    node->min_f_val = parent->min_f_val;
    node->depth = parent->depth + 1;
    node->window = parent->window;

    // Replan: constraint가 추가된 agent(=constraints.front().agent). 표준 ECBS.
    std::list<int> to_replan;
    if (!node->constraints.empty())
    {
        const auto& c = node->constraints.front();
        if (c.positive)
        {
            // Positive constraint: 다른 agent들이 해당 time에 해당 cell을 쓰는지
            // 체크해서 replan. v1 disjoint=false라 보통 발생 안 함.
            for (int i = 0; i < num_of_agents; i++)
            {
                if (i == c.agent) continue;
                if (paths[i] == nullptr) continue;
                auto& path_i = *paths[i];
                if (path_i.empty() || path_i.back().timestep <= c.timestep) continue;
                int si = 0, sii = 0;
                for (int ts = 0; ts <= c.timestep; ts++)
                {
                    if (si + 1 < (int)path_i.size() && path_i[si + 1].timestep == ts) { si += 1; sii = 0; }
                    const auto& st = path_i[si];
                    const auto& fp = st.full_path;
                    if (fp.empty()) continue;
                    int cell_idx = sii >= (int)fp.size() ? fp.back().second : fp[sii].second;
                    if (ts == c.timestep && st.section_id == c.section_id && cell_idx == c.cell_idx)
                    {
                        to_replan.push_back(i);
                        break;
                    }
                    if (sii < (int)fp.size()) sii += 1;
                }
            }
        }
        else
        {
            to_replan.push_back(c.agent);
        }
    }

    // Bring the per-agent key cache into sync with the current paths[] before
    // the replan (this catches pointer changes since the last call — e.g. a
    // sibling's generate_child that modified paths[] and was then rolled back).
    refresh_keys_cache();

    for (int a : to_replan)
    {
        if (!find_path(node, a, mapsys)) return false;
        // find_path updated paths[a]; refresh the cache row so the subsequent
        // find_conflicts sees the new path.
        build_keys_for_agent(a);
        fc_cache_path_ptr[a] = paths[a];
    }

    find_conflicts(parent->conflicts, node->conflicts, to_replan);
    node->num_of_collisions = (int)node->conflicts.size();
    node->h_val = 0;
    node->f_val = node->g_val + node->h_val;
    return true;
}

// --- Root & HL main loop ----------------------------------------------------

bool ECBSSection::generate_root_node(MapSystem* mapsys)
{
    dummy_start = new ECBSNodeSection();
    paths.assign(num_of_agents, nullptr);
    path_min_costs.assign(num_of_agents, 0);
    path_costs.assign(num_of_agents, 0);

    for (int i = 0; i < num_of_agents; i++)
    {
        clock_t t = std::clock();
        rs.init_empty();
        runtime_rt += (double)(std::clock() - t) / CLOCKS_PER_SEC;

        t = std::clock();
        SectionPath path = section_path_planner->run_section(
            start_sections[i], goal_sections[i], rs, i, 8, mapsys);
        runtime_plan_paths += (double)(std::clock() - t) / CLOCKS_PER_SEC;

        double path_cost = section_path_planner->path_cost;
        double path_lb   = section_path_planner->min_f_val;
        if (path_lb <= 0) path_lb = path_cost;

        LL_num_expanded += section_path_planner->num_expanded;
        LL_num_generated += section_path_planner->num_generated;
        rs.clear();

        if (path.empty())
        {
            std::cout << "NO SOLUTION EXISTS (agent " << i << ")" << std::endl;
            return false;
        }

        dummy_start->paths.emplace_back(i, std::move(path), path_lb, path_cost);
        paths[i] = &std::get<1>(dummy_start->paths.back());
        path_min_costs[i] = path_lb;
        path_costs[i] = path_cost;
        dummy_start->g_val     += path_cost;
        dummy_start->min_f_val += path_lb;
    }

    find_conflicts(dummy_start->conflicts);
    dummy_start->window = window;
    dummy_start->num_of_collisions = (int)dummy_start->conflicts.size();
    dummy_start->h_val = 0;
    dummy_start->f_val = dummy_start->g_val;

    min_f_val = dummy_start->min_f_val;
    focal_threshold = min_f_val * suboptimal_bound;

    HL_num_generated++;
    dummy_start->time_generated = HL_num_generated;
    push_node(dummy_start);
    best_node = dummy_start;
    return true;
}

void ECBSSection::push_node(ECBSNodeSection* node)
{
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    if (node->f_val <= focal_threshold)
        node->focal_handle = focal_list.push(node);
    allNodes_table.push_back(node);
}

void ECBSSection::reinsert_node(ECBSNodeSection* node)
{
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    if (node->f_val <= focal_threshold)
        node->focal_handle = focal_list.push(node);
}

ECBSNodeSection* ECBSSection::pop_node()
{
    update_focal_list();
    ECBSNodeSection* node = focal_list.top();
    focal_list.pop();
    open_list.erase(node->open_handle);
    node->in_openlist = false;
    return node;
}

void ECBSSection::update_focal_list()
{
    clock_t t = std::clock();
    if (open_list.empty()) return;
    ECBSNodeSection* open_head = open_list.top();
    if (open_head->min_f_val > min_f_val)
    {
        min_f_val = open_head->min_f_val;
        double new_threshold = min_f_val * suboptimal_bound;
        for (ECBSNodeSection* n : open_list)
        {
            if (n->f_val > focal_threshold && n->f_val <= new_threshold)
                n->focal_handle = focal_list.push(n);
        }
        focal_threshold = new_threshold;
    }
    runtime_update_focal += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}

bool ECBSSection::run_section(const vector<SectionState>& start_sections_in,
                              const vector<vector<pair<SectionState, int>>> goal_sections_in,
                              int _time_limit, MapSystem* mapsys)
{
    clear();
    start_time = std::clock();

    // Reset profiling timers (accumulated across SIPP calls within this run_section).
    if (section_path_planner != nullptr)
    {
        auto* sp = section_path_planner;
        sp->t_find_wait_list = 0; sp->t_compute_h = 0;
        sp->t_safe_intervals = 0; sp->t_generate_node = 0;
        sp->t_focal_update = 0; sp->t_update_path = 0;
        sp->fwl_calls = 0; sp->gn_calls = 0;
    }

    this->start_sections = start_sections_in;
    this->goal_sections  = goal_sections_in;
    this->num_of_agents  = (int)start_sections_in.size();
    this->time_limit     = _time_limit;
    this->current_mapsys = mapsys;

    solution_cost = -2;
    solution_found = false;

    if (section_path_planner != nullptr)
    {
        section_path_planner->suboptimal_bound = suboptimal_bound;
        section_path_planner->hold_endpoints = hold_endpoints;
        section_path_planner->prioritize_start = false;
    }

    // Disjoint splitting flag mirrors RHCR_ECBS_DISJOINT. Needs to be set on
    // the instance (not just in resolve_conflict) so collect_constraints
    // propagates positive constraints to other agents as negative — otherwise
    // the positive branch can never force others out of the protected cell.
    {
        const char* e = std::getenv("RHCR_ECBS_DISJOINT");
        disjoint_splitting = (e && *e && e[0] != '0');
    }

    if (!generate_root_node(mapsys))
    {
        runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
        print_results();
        return false;
    }

    if (dummy_start->num_of_collisions == 0)
    {
        solution_found = true;
        solution_cost = dummy_start->g_val;
        best_node = dummy_start;
    }

    const int hl_iter_limit = 100000;
    int hl_iter = 0;

    // Diagnostics for failure-mode classification. Env RHCR_ECBS_DIAG=1
    // enables a printed trajectory of best-so-far (min-collision) node.
    const bool diag_on = [](){
        const char* e = std::getenv("RHCR_ECBS_DIAG");
        return e && *e && e[0] != '0';
    }();
    int diag_min_col = dummy_start->num_of_collisions;
    int diag_min_col_iter = 0;
    int diag_last_improve_iter = 0;
    int diag_reinsert_count = 0;
    int diag_safety_fires = 0;

    // Stall-based early-exit. Profiling of 6/100 failing calls on w=5 seed=0
    // showed min-collision node reached within first 200 iterations, then 53k+
    // iterations of zero improvement before timeout. Treat that as hopeless
    // and fall back instead of burning the full time budget.
    // Env RHCR_ECBS_STALL_LIMIT overrides; default 5000 (tunable).
    static int stall_limit = -1;
    if (stall_limit < 0) {
        const char* e = std::getenv("RHCR_ECBS_STALL_LIMIT");
        stall_limit = (e && *e) ? std::atoi(e) : 5000;
    }

    while (!open_list.empty() && !solution_found)
    {
        if (++hl_iter > hl_iter_limit) break;

        runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
        if (runtime > time_limit)
        {
            solution_cost = -1;
            solution_found = false;
            break;
        }

        // Stall-based early-exit: if min-collision has not improved for
        // stall_limit iterations AND we have some baseline (not root),
        // declare failure. Skip when still in early phase (hl_iter <
        // stall_limit) so short searches can complete normally.
        if (stall_limit > 0 &&
            hl_iter > stall_limit &&
            (hl_iter - diag_last_improve_iter) > stall_limit)
        {
            solution_cost = -1;
            solution_found = false;
            break;
        }

        if (diag_on && (hl_iter % 2000 == 0))
        {
            std::cout << "[ECBSDIAG] iter=" << hl_iter
                      << " t=" << runtime
                      << "s open=" << open_list.size()
                      << " focal=" << focal_list.size()
                      << " min_f=" << min_f_val
                      << " min_col=" << diag_min_col
                      << " at_iter=" << diag_min_col_iter
                      << " last_improve=" << diag_last_improve_iter
                      << " reinsert=" << diag_reinsert_count
                      << std::endl;
        }

        ECBSNodeSection* curr = pop_node();
        update_paths(curr);

        if (window > curr->window)
        {
            curr->conflicts.clear();
            find_conflicts(curr->conflicts);
            curr->window = window;
            curr->num_of_collisions = (int)curr->conflicts.size();
            reinsert_node(curr);
            diag_reinsert_count++;
            continue;
        }

        if (curr->conflicts.empty())
        {
            // Safety-net: bucketing all-pair (O(N·W)). 기존 O(N²·W) pairwise 대체.
            std::list<SectionConflict> safety;
            find_conflicts(safety);
            if (!safety.empty())
            {
                safety_net_fires++;
                safety_net_conflicts_found += safety.size();
                curr->conflicts = std::move(safety);
                curr->num_of_collisions = (int)curr->conflicts.size();
                reinsert_node(curr);
                diag_safety_fires++;
                continue;
            }
            solution_found = true;
            solution_cost = curr->g_val;
            best_node = curr;
            break;
        }

        choose_conflict(*curr);
        if (best_node == nullptr ||
            curr->conflict.timestep > best_node->conflict.timestep ||
            (curr->conflict.timestep == best_node->conflict.timestep &&
             curr->f_val < best_node->f_val))
            best_node = curr;

        if (curr->num_of_collisions < diag_min_col)
        {
            diag_min_col = curr->num_of_collisions;
            diag_min_col_iter = hl_iter;
            diag_last_improve_iter = hl_iter;
        }

        HL_num_expanded++;
        curr->time_expanded = HL_num_expanded;

        ECBSNodeSection* n[2];
        for (int i = 0; i < 2; i++) n[i] = new ECBSNodeSection(curr);
        resolve_conflict(curr->conflict, n[0], n[1]);

        std::vector<SectionPath*> path_copy(paths);
        std::vector<double> cost_copy(path_costs);
        std::vector<double> lb_copy(path_min_costs);

        for (int i = 0; i < 2; i++)
        {
            bool sol = generate_child(n[i], curr, mapsys);
            if (sol)
            {
                HL_num_generated++;
                n[i]->time_generated = HL_num_generated;
                push_node(n[i]);
            }
            else
            {
                delete n[i];
                n[i] = nullptr;
            }
            paths = path_copy;
            path_costs = cost_copy;
            path_min_costs = lb_copy;
        }

        curr->clear();
    }

    runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
    get_solution();

    if (diag_on)
    {
        std::cout << "[ECBSDIAG-END] solved=" << solution_found
                  << " iters=" << hl_iter
                  << " t=" << runtime << "s"
                  << " HL_exp=" << HL_num_expanded
                  << " HL_gen=" << HL_num_generated
                  << " LL_exp=" << LL_num_expanded
                  << " min_col=" << diag_min_col
                  << " at_iter=" << diag_min_col_iter
                  << " last_improve=" << diag_last_improve_iter
                  << " stalled_iters=" << (hl_iter - diag_last_improve_iter)
                  << " reinsert=" << diag_reinsert_count
                  << " safety_fires=" << diag_safety_fires
                  << " final_best_col=" << (best_node ? best_node->num_of_collisions : -1)
                  << std::endl;
    }

    if (solution_found && !validate_solution())
    {
        std::cout << "Solution invalid!!!" << std::endl;
    }

    min_sum_of_costs = 0;
    if (mapsys != nullptr)
    {
        for (int i = 0; i < num_of_agents; i++)
        {
            std::vector<double> dist_goal = mapsys->goal_to_goal_dist(goal_sections[i]);
            min_sum_of_costs += mapsys->compute_h_value(
                start_sections[i].section_id,
                start_sections[i].start_index,
                0, goal_sections[i], dist_goal);
        }
    }

    print_results();
    return solution_found;
}

// --- Housekeeping ------------------------------------------------------------

ECBSSection::ECBSSection(const BasicGraph& G, SingleAgentSolver& path_planner)
    : MAPFSolver(G, path_planner) {}

ECBSSection::ECBSSection(const BasicGraph& G, SingleAgentSolver& path_planner,
                         SIPPSection& section_path_planner)
    : MAPFSolver(G, path_planner, section_path_planner) {}

inline void ECBSSection::release_closed_list()
{
    for (auto& n : allNodes_table) delete n;
    allNodes_table.clear();
}

ECBSSection::~ECBSSection() { release_closed_list(); }

void ECBSSection::get_solution()
{
    if (best_node == nullptr) return;
    update_paths(best_node);
    section_solution.assign(num_of_agents, SectionPath());
    for (int k = 0; k < num_of_agents; k++)
        if (paths[k] != nullptr) section_solution[k] = *paths[k];
    avg_path_length = 0;
    for (int k = 0; k < num_of_agents; k++)
        if (paths[k] != nullptr && !paths[k]->empty())
            avg_path_length += paths[k]->back().timestep;
    if (num_of_agents > 0) avg_path_length /= num_of_agents;
}

bool ECBSSection::validate_solution()
{
    std::list<SectionConflict> conflict;
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
        {
            find_conflicts(conflict, a1, a2);
            if (!conflict.empty())
            {
                const auto& c = conflict.front();
                std::cout << "Agents " << c.agent1 << " and " << c.agent2
                          << " collide at section " << c.section_id
                          << " cell " << c.local_index
                          << " at t=" << c.timestep << std::endl;
                return false;
            }
        }
    }
    return true;
}

void ECBSSection::print_paths() const
{
    for (int i = 0; i < num_of_agents; i++)
    {
        if (paths[i] == nullptr) continue;
        std::cout << "Agent " << i << " (" << paths[i]->back().timestep << "): ";
        for (const auto& s : *paths[i]) std::cout << s.section_id << "->";
        std::cout << std::endl;
    }
}

void ECBSSection::print_results() const
{
    std::cout << "ECBSSection:";
    if (solution_cost >= 0) std::cout << "Succeed,";
    else if (solution_cost == -1) std::cout << "Timeout,";
    else if (solution_cost == -2) std::cout << "No solutions,";
    else if (solution_cost == -3) std::cout << "Nodesout,";

    std::cout << runtime << ","
              << HL_num_expanded << "," << HL_num_generated << ","
              << LL_num_expanded << "," << LL_num_generated << ","
              << solution_cost << "," << min_f_val << ","
              << avg_path_length << ","
              << (dummy_start ? dummy_start->num_of_collisions : 0) << ","
              << window << ","
              << suboptimal_bound << ","
              << "snf=" << safety_net_fires << ","
              << "snc=" << safety_net_conflicts_found
              << std::endl;
}

void ECBSSection::save_results(const std::string& fileName, const std::string& instanceName) const
{
    std::ofstream stats;
    stats.open(fileName, std::ios::app);
    stats << runtime << ","
          << HL_num_expanded << "," << HL_num_generated << ","
          << LL_num_expanded << "," << LL_num_generated << ","
          << solution_cost << "," << min_sum_of_costs << ","
          << avg_path_length << ","
          << (dummy_start ? dummy_start->num_of_collisions : 0) << ","
          << instanceName << "," << window << "," << suboptimal_bound << std::endl;
    stats.close();
}

void ECBSSection::save_search_tree(const std::string& fname) const
{
    std::ofstream output;
    output.open(fname, std::ios::out);
    output << "digraph G {" << std::endl;
    output << "size = \"5,5\";" << std::endl;
    output << "center = true;" << std::endl;
    for (auto node : allNodes_table)
    {
        if (node == dummy_start) continue;
        if (node->time_expanded == 0) output << node->time_generated << " [color=blue]" << std::endl;
        if (node->parent) output << node->parent->time_generated << " -> " << node->time_generated << std::endl;
    }
    output << "}" << std::endl;
    output.close();
}
