#pragma once
#include "StateTimeAStar.h"
#include "SingleAgentSolver.h"
#include "ReservationSection.h"
#include "PriorityGraph.h" // PBS 연동용
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/functional/hash.hpp> // boost::hash_combine 사용
#include <unordered_set>

using boost::heap::fibonacci_heap;
using boost::heap::compare;

// SIPPNode가 StateTimeAStarNode를 상속받도록 하여 호환성 유지
class SIPPNode : public StateTimeAStarNode
{
public:
    SIPPNode* parent;
    SecInterval interval; // <start, end, occupancy>

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

    SIPPNode() : StateTimeAStarNode(), parent(nullptr) {}

    SIPPNode(const State& state, double g_val, double h_val, const SecInterval& interval,
             SIPPNode* parent, int conflicts)
        : StateTimeAStarNode(state, g_val, h_val, nullptr, conflicts), 
          parent(parent), interval(interval)
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
                   (n1 && n2 && n1->state.location == n2->state.location &&
                    n1->interval == n2->interval && // 같은 구간인지가 중요!
                    n1->goal_id == n2->goal_id);
        }
    };

    // 해시 함수 (Closed List용)
    struct Hasher {
        size_t operator()(const SIPPNode* n) const {
            size_t seed = 0;
            boost::hash_combine(seed, n->state.location);
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
             ReservationTable& RT) override 
    {
        std::cerr << "[Error] SIPPSection은 ReservationTable을 사용하지 않습니다! 새로운 run을 호출하세요." << std::endl;
        exit(1); 
        return Path();
    }

    // 기존 ReservationTable 대신 우리가 만든 ReservationSection을 받음
    Path run(const BasicGraph& G, const State& start,
             const vector<pair<int, int>>& goal_location,
             ReservationSection& rs, const PriorityGraphj* pg, int agent_id, int capacity);

    string getName() const { return "SIPPSection"; }
    SIPPSection() : SingleAgentSolver() {}

private:
    fibonacci_heap<SIPPNode*, compare<SIPPNode::compare_node>> open_list;
    fibonacci_heap<SIPPNode*, compare<SIPPNode::secondary_compare_node>> focal_list;
    
    // Closed List: RHCR 스타일로 unordered_set 사용
    unordered_set<SIPPNode*, SIPPNode::Hasher, SIPPNode::EqNode> allNodes_table;

    void generate_node(const SecInterval& interval, SIPPNode* curr, const BasicGraph& G,
                       int next_section_id, int arrival_time, double h_val, int conflicts);
    
    Path updatePath(const BasicGraph& G, const SIPPNode* goal);

    inline void releaseClosedListNodes();
};