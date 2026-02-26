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

// SIPPSectionNode가 StateTimeAStarNode를 상속받도록 하여 호환성 유지
class SIPPSectionNode : public StateTimeAStarNode
{
public:
    SectionState s_state;
    SIPPSectionNode* parent;
    SecInterval interval; // <start, end, occupancy>
    int parent_exit_index; // parent section에서 사용한 출구
    std::vector<int> parent_wait_list;

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

    // Fibonacci Heap 핸들 정의
    fibonacci_heap<SIPPSectionNode*, compare<SIPPSectionNode::compare_node>>::handle_type open_handle;
    fibonacci_heap<SIPPSectionNode*, compare<SIPPSectionNode::secondary_compare_node>>::handle_type focal_handle;

    SIPPSectionNode() : StateTimeAStarNode(), parent(nullptr), parent_exit_index(-1) {}

    SIPPSectionNode(const SectionState& state, double g_val, double h_val, const SecInterval& interval,
             SIPPSectionNode* parent, int conflicts, int parent_exit_index = -1, const std::vector<int>& parent_wait_list = {})
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

    // 해시 함수 (Closed List용)
    struct Hasher {
        size_t operator()(const SIPPSectionNode* n) const {
            size_t seed = 0;
            // ★ 핵심: SectionState::Hasher 대신 위치 정보만 직접 해싱!
            boost::hash_combine(seed, n->s_state.section_id);
            boost::hash_combine(seed, n->s_state.start_index);
            
            boost::hash_combine(seed, std::get<0>(n->interval)); // interval start
            // (interval start만으로도 같은 구역 내에서는 고유하게 구별되므로 충분합니다)
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
    fibonacci_heap<SIPPSectionNode*, compare<SIPPSectionNode::compare_node>> open_list;
    fibonacci_heap<SIPPSectionNode*, compare<SIPPSectionNode::secondary_compare_node>> focal_list;
    
    // Closed List: RHCR 스타일로 unordered_set 사용
    unordered_set<SIPPSectionNode*, SIPPSectionNode::Hasher, SIPPSectionNode::EqNode> allNodes_table;

    void generate_node(const SecInterval& interval, SIPPSectionNode* curr, 
                                int next_section_id, int next_start_index, int curr_exit_index,
                                const std::vector<int>& wait_list, ReservationSection& rs,
                                double travel_cost, int arrival_time, double h_val, int section_congestion);

    SectionPath updatePath(const SIPPSectionNode* goal);

    inline void releaseClosedListNodes();

    void find_wait_list(int section_id, int start_index, int exit_index, int timestep, const ReservationSection& rs, MapSystem* MapSys, int next_section_id, int next_start_index, std::vector<int>& wait_list);
};

