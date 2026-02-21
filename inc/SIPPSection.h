#pragma once
#include "StateTimeAStar.h"
#include "SingleAgentSolver.h"
#include "ReservationSection.h"
#include "SectionState.h"
#include "MapSystem.h"
#include "PriorityGraph.h" // PBS 연동용
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/functional/hash.hpp> // boost::hash_combine 사용
#include <unordered_set>
#include <vector>
#include <iostream>

using boost::heap::fibonacci_heap;
using boost::heap::compare;

// MapSystem에서 이웃 정보를 받아오기 위한 임시 구조체
struct NeighborInfo{
    int section_id;
    int curr_exit_index;
    int next_start_index;
    vector<int> wait_list;
    double internal_cost; // curr_start -> curr_exit 비용
    double edge_cost; // curr_exit -> next_start 비용
};

// SIPPNode가 StateTimeAStarNode를 상속받도록 하여 호환성 유지
class SIPPNode : public StateTimeAStarNode
{
public:
    SectionState s_state;
    SIPPNode* parent;
    SecInterval interval; // <start, end, occupancy>
    int parent_exit_index; // parent section에서 사용한 출구
    std::vector<int> parent_wait_list;

    // OPEN 리스트용 비교 연산 (f-val 최소, 같으면 g-val 최대)
    struct compare_node
    {
        bool operator()(const SIPPNode* n1, const SIPPNode* n2) const
        {
            if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
                return n1->g_val <= n2->g_val; 
            return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
        }
    };

    // FOCAL 리스트용 비교 연산 (충돌 최소화 우선)
    struct secondary_compare_node
    {
        bool operator()(const SIPPNode* n1, const SIPPNode* n2) const
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

    // Fibonacci Heap 핸들 정의
    fibonacci_heap<SIPPNode*, compare<SIPPNode::compare_node>>::handle_type open_handle;
    fibonacci_heap<SIPPNode*, compare<SIPPNode::secondary_compare_node>>::handle_type focal_handle;

    SIPPNode() : StateTimeAStarNode(), parent(nullptr), parent_exit_index(-1) {}

    SIPPNode(const SectionState& state, double g_val, double h_val, const SecInterval& interval,
             SIPPNode* parent, int conflicts, int parent_exit_index = -1, const std::vector<int>& parent_wait_list = {})
        : StateTimeAStarNode(State(-1, -1), g_val, h_val, nullptr, conflicts),  // 부모의 기존 state 무효화
          s_state(state), parent(parent), interval(interval), parent_exit_index(parent_exit_index), parent_wait_list(parent_wait_list)
    {
        if (parent != nullptr) {
            depth = parent->depth + 1;
            goal_id = parent->goal_id;
        } else {
            depth = 0;
            goal_id = 0;
        }
    }

    // 노드 중복 체크를 위한 EqNode
    struct EqNode
    {
        bool operator()(const SIPPNode* n1, const SIPPNode* n2) const
        {
            return (n1 == n2) ||
                   (n1 && n2 && n1->s_state == n2->s_state &&
                    n1->interval == n2->interval && // 같은 구간인지가 중요!
                    n1->goal_id == n2->goal_id);
        }
    };

    // 해시 함수 (Closed List용)
    struct Hasher {
        size_t operator()(const SIPPNode* n) const {
            size_t seed = 0;
            boost::hash_combine(seed, SectionState::Hasher()(n->s_state));
            boost::hash_combine(seed, std::get<0>(n->interval)); // interval start
            boost::hash_combine(seed, std::get<2>(n->interval)); // current_occupancy
            boost::hash_combine(seed, n->goal_id);
            return seed;
        }
    };
};

class SIPPSection : public SingleAgentSolver
{
public:
    // 기존 프레임워크가 실수로 옛날 방식으로 호출하면 에러를 뿜게 만듦
    Path run(const BasicGraph& G, const State& start, 
             const vector<pair<int, int>>& goal_location, 
             ReservationTable& RT) override;

    // 기존 ReservationTable 대신 우리가 만든 ReservationSection을 받음
    SectionPath run_section(const SectionState& start_state, 
                                     const vector<pair<SectionState, int>>& goal_sections, 
                                     ReservationSection& rs, 
                                     int agent_id, int capacity, MapSystem* MapSys);

    string getName() const override { return "SIPPSection"; }
    SIPPSection() : SingleAgentSolver() {}

private:
    fibonacci_heap<SIPPNode*, compare<SIPPNode::compare_node>> open_list;
    fibonacci_heap<SIPPNode*, compare<SIPPNode::secondary_compare_node>> focal_list;
    
    // Closed List: RHCR 스타일로 unordered_set 사용
    unordered_set<SIPPNode*, SIPPNode::Hasher, SIPPNode::EqNode> allNodes_table;

    void generate_node(const SecInterval& interval, SIPPNode* curr, 
                                int next_section_id, int next_start_index, int curr_exit_index,
                                const std::vector<int>& wait_list,
                                double travel_cost, int arrival_time, double h_val, int section_congestion);

    SectionPath updatePath(const SIPPNode* goal);

    inline void releaseClosedListNodes();
};