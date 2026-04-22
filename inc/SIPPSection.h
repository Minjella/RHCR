#pragma once
#include "StateTimeAStar.h"
#include "SingleAgentSolver.h"
#include "ReservationSection.h"
#include "SectionState.h"
#include "MapSystem.h"
#include "PriorityGraph.h" // PBS 연동용
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <boost/functional/hash.hpp> // boost::hash_combine 사용
#include <unordered_set>
#include <vector>
#include <iostream>

using boost::heap::fibonacci_heap;
using boost::heap::d_ary_heap;
using boost::heap::compare;
using boost::heap::arity;
using boost::heap::mutable_;

// MapSystem에서 이웃 정보를 받아오기 위한 임시 구조체
struct NeighborInfo{
    int section_id;
    int curr_exit_index;
    int next_start_index;
    vector<int> wait_list;
    double internal_cost; 
    double edge_cost; 
};

// SIPPSectionNode가 StateTimeAStarNode를 상속받도록 하여 호환성 유지
class SIPPSectionNode : public StateTimeAStarNode
{
public:
    SectionState s_state;
    SIPPSectionNode* parent;
    SecInterval interval; // <start, end, occupancy>
    int parent_exit_index; // parent section에서 사용한 출구
    int wait_at_exit;
    int wait_at_goal;

    // OPEN 리스트용 비교 연산 (f-val 최소, 같으면 g-val 최대)
    struct compare_node
    {
        bool operator()(const SIPPSectionNode* n1, const SIPPSectionNode* n2) const
        {
            if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
                return n1->g_val <= n2->g_val; 
            return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
        }
    };

    // FOCAL 리스트용 비교 연산 (충돌 최소화 우선)
    struct secondary_compare_node
    {
        bool operator()(const SIPPSectionNode* n1, const SIPPSectionNode* n2) const
        {
            if (n1->conflicts == n2->conflicts)
            {
                if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
                    return n1->g_val <= n2->g_val;
                return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
            }
            return n1->conflicts >= n2->conflicts;
        }
    };

    // Binary heap (d-ary with arity=2). Fibonacci 대비 cache locality 우수.
    typedef d_ary_heap<SIPPSectionNode*, arity<2>, mutable_<true>, compare<SIPPSectionNode::compare_node>> open_heap_t;
    typedef d_ary_heap<SIPPSectionNode*, arity<2>, mutable_<true>, compare<SIPPSectionNode::secondary_compare_node>> focal_heap_t;
    open_heap_t::handle_type open_handle;
    focal_heap_t::handle_type focal_handle;

    SIPPSectionNode() : StateTimeAStarNode(), parent(nullptr), parent_exit_index(-1), wait_at_exit(0), wait_at_goal(0) {}

    SIPPSectionNode(const SectionState& state, double g_val, double h_val, const SecInterval& interval,
             SIPPSectionNode* parent, int conflicts, int parent_exit_index = -1, int wait_at_exit = 0, int wait_at_goal = 0)
        : StateTimeAStarNode(State(-1, -1), g_val, h_val, nullptr, conflicts), 
          s_state(state), parent(parent), interval(interval), parent_exit_index(parent_exit_index), wait_at_exit(wait_at_exit), wait_at_goal(wait_at_goal)
    {
        if (parent != nullptr) {
            depth = parent->depth + 1;
            goal_id = parent->goal_id;
        } else {
            depth = 0;
            goal_id = 0;
        }
    }

    SIPPSectionNode(const SectionState& state, const SecInterval& iv, int goal_id_)
    : StateTimeAStarNode(State(-1,-1), 0, 0, nullptr, 0),
      s_state(state), parent(nullptr), interval(iv), parent_exit_index(-1), wait_at_exit(0), wait_at_goal(0)
    {
        goal_id = goal_id_;
    }

    // 노드 중복 체크를 위한 EqNode
    struct EqNode
    {
        bool operator()(const SIPPSectionNode* n1, const SIPPSectionNode* n2) const
        {
            if (n1 == n2) return true;
            if (!n1 || !n2) return false;

            return (n1->s_state.section_id == n2->s_state.section_id) &&
                   (n1->s_state.start_index == n2->s_state.start_index) &&
                   (n1->interval == n2->interval) && 
                   (n1->goal_id == n2->goal_id);
        }
    };

    // 해시 함수 (Closed List용). boost::hash_combine 4회 대신 단일 64bit mix.
    // 필드: section_id (14b), start_index (4b), interval_start (14b), goal_id (4b).
    // 36 bits → uint64_t 안에 pack 후 splitmix64 hash.
    struct Hasher {
        size_t operator()(const SIPPSectionNode* n) const noexcept {
            uint64_t k = (uint64_t)(n->s_state.section_id & 0x3FFF)
                       | ((uint64_t)(n->s_state.start_index & 0xF) << 14)
                       | ((uint64_t)(std::get<0>(n->interval) & 0x3FFF) << 18)
                       | ((uint64_t)(n->goal_id & 0xF) << 32);
            // splitmix64 finalizer
            k ^= k >> 33; k *= 0xff51afd7ed558ccdULL;
            k ^= k >> 33; k *= 0xc4ceb9fe1a85ec53ULL;
            k ^= k >> 33;
            return (size_t)k;
        }
    };
};

class SIPPSection : public SingleAgentSolver
{
public:
    Path run(const BasicGraph& G, const State& start, 
             const vector<pair<int, int>>& goal_location, 
             ReservationTable& RT) override;

    SectionPath run_section(const SectionState& start_state, 
                                     const vector<pair<SectionState, int>>& goal_sections, 
                                     ReservationSection& rs, 
                                     int agent_id, int capacity, MapSystem* MapSys);

    string getName() const override { return "SIPPSection"; }
    
    SIPPSection() : SingleAgentSolver() {}
    
    // [추가] 소멸자: 프로그램 종료 시 메모리 풀 일괄 해제
    ~SIPPSection() {
        for (auto node : node_pool) {
            delete node;
        }
    }

private:
    SIPPSectionNode::open_heap_t open_list;
    SIPPSectionNode::focal_heap_t focal_list;

    unordered_set<SIPPSectionNode*, SIPPSectionNode::Hasher, SIPPSectionNode::EqNode> allNodes_table;

    // 메모리 풀
    std::vector<SIPPSectionNode*> node_pool;
    int pool_index = 0;

    // compute_h_value 캐시 (same run_section 내에서 재사용).
    // Key: (section_id << 20) | (cell_idx << 16) | goal_id — 한 agent의 search 동안
    // goal_sections / goal_to_goal이 고정이므로 (section, cell, goal_id) 3종으로 유일.
    std::unordered_map<uint64_t, double> h_cache;


    // 메모리 풀 할당 및 초기화 헬퍼
    SIPPSectionNode* allocate_node() {
        if (pool_index >= (int)node_pool.size()) {
            // 공간 부족 시 한 번에 1000개씩 할당해둠
            for(int i = 0; i < 1000; ++i) {
                node_pool.push_back(new SIPPSectionNode());
            }
        }
        SIPPSectionNode* node = node_pool[pool_index++];
        // [가장 중요] 이전 탐색에서 쓰인 잔여 플래그 완벽 초기화! (이거 없으면 길 못 찾음)
        node->in_openlist = false;
        node->parent = nullptr;
        return node;
    }

    void generate_node(const SecInterval& interval, SIPPSectionNode* curr, 
                                int next_section_id, int next_start_index, int curr_exit_index,
                                ReservationSection& rs,
                                double travel_cost, int arrival_time, double h_val, int section_congestion);

    SectionPath updatePath(const SIPPSectionNode* goal, ReservationSection& rs, MapSystem* mapsys);

    inline void releaseClosedListNodes();

    // Profiling timers (per run_section)
public:
    double t_find_wait_list = 0;
    double t_compute_h = 0;
    double t_safe_intervals = 0;
    double t_generate_node = 0;
    double t_focal_update = 0;
    double t_update_path = 0;
    uint64_t fwl_calls = 0;
    uint64_t gn_calls = 0;

private:

    int find_wait_list(int section_id, int start_index, int exit_index, int timestep, const ReservationSection& rs, MapSystem* MapSys, int& next_section_id, int& next_start_index, std::vector<int>& wait_list, std::vector<pair<int, int>>& full_path, int circle_flag, bool build_path = false);
};