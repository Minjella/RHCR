#pragma once
#include "common.h"
#include "SectionState.h"

// Section-level CBS constraint.
// - agent: constraint를 받는 agent
// - section_id + cell_idx: 금지(또는 강제) cell
// - timestep: 해당 cell에서의 절대 시간
// - positive: true면 "이 agent는 반드시 이 cell에 있어야 한다" (disjoint splitting),
//             false면 "이 agent는 이 cell에 있을 수 없다"
// 메모리 절약을 위해 shared_ptr 대신 값 타입으로만 저장한다.
struct SectionConstraint
{
    int agent;
    int section_id;
    int cell_idx;
    int timestep;
    bool positive;

    SectionConstraint()
        : agent(-1), section_id(-1), cell_idx(-1), timestep(-1), positive(false) {}
    SectionConstraint(int a, int s, int c, int t, bool p)
        : agent(a), section_id(s), cell_idx(c), timestep(t), positive(p) {}
};

inline std::ostream& operator<<(std::ostream& os, const SectionConstraint& cst) {
    os << "[SCstr: a=" << cst.agent << " sec=" << cst.section_id
       << " cell=" << cst.cell_idx << " t=" << cst.timestep
       << " pos=" << cst.positive << "]";
    return os;
}

class ECBSNodeSection
{
public:
    // OPEN: min_f_val (= sum of LL lower bounds) 오름차순. top이 최솟값.
    struct compare_node
    {
        bool operator()(const ECBSNodeSection* n1, const ECBSNodeSection* n2) const
        {
            return n1->min_f_val >= n2->min_f_val;
        }
    };

    // FOCAL: num_of_collisions 오름차순, 동률이면 f_val 오름차순.
    // (top이 충돌 가장 적은 노드)
    struct secondary_compare_node
    {
        bool operator()(const ECBSNodeSection* n1, const ECBSNodeSection* n2) const
        {
            if (n1->num_of_collisions == n2->num_of_collisions)
                return n1->f_val >= n2->f_val;
            return n1->num_of_collisions >= n2->num_of_collisions;
        }
    };

    typedef fibonacci_heap<ECBSNodeSection*, compare<ECBSNodeSection::compare_node>>::handle_type open_handle_t;
    typedef fibonacci_heap<ECBSNodeSection*, compare<ECBSNodeSection::secondary_compare_node>>::handle_type focal_handle_t;
    open_handle_t open_handle;
    focal_handle_t focal_handle;
    bool in_openlist;

    // 현재 노드의 path들 사이에서 발견된 conflict 리스트 (값 타입, 힙 할당 없음).
    std::list<SectionConflict> conflicts;

    // 분기 대상으로 고른 conflict.
    SectionConflict conflict;

    ECBSNodeSection* parent;

    // <agent_id, section_path, lower_bound, path_cost> — ECBS와 동일한 4-tuple.
    // lower_bound는 SIPPSection의 min_f_val (제약 무시 최소 cost).
    std::list<std::tuple<int, SectionPath, double, double>> paths;

    // 이 노드에서 새로 부과된 constraint (parent chain을 따라 누적됨).
    std::list<SectionConstraint> constraints;

    double g_val;       // 현재 paths의 총 cost
    double h_val;       // 추정 h (보통 0)
    double f_val;       // g + h
    double min_f_val;   // sum of LL lower_bound → OPEN 정렬 키
    size_t depth;
    int num_of_collisions;
    uint64_t time_expanded;
    uint64_t time_generated;

    int window;

    void clear();

    ECBSNodeSection()
        : in_openlist(false),
          conflict(-1, -1, -1, -1, -1, ConflictType::TILE_VERTEX),
          parent(nullptr),
          g_val(0), h_val(0), f_val(0), min_f_val(0),
          depth(0), num_of_collisions(0),
          time_expanded(0), time_generated(0),
          window(0) {}
    explicit ECBSNodeSection(ECBSNodeSection* parent);
    ~ECBSNodeSection() {}
};
