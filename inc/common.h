#pragma once
#include <utility>
#include <tuple>
#include <list>
#include <vector>
#include <iostream>
#include <cfloat>
#include <ctime>
#include <fstream>
#include <set>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

using boost::heap::fibonacci_heap;
using boost::heap::compare;
using boost::unordered_set;
using boost::unordered_map;

using std::set;
using std::vector;
using std::tuple;
using std::deque;
using std::make_tuple;
using std::pair;
using std::make_pair;
using std::list;
using std::cout;
using std::endl;
using std::ostream;
using std::string;
using std::max;
using std::min;

//#include <boost/graph/adjacency_list.hpp>
//typedef boost::adjacency_list_traits<int, int, boost::undirectedS > confilctGraph_t;
//typedef confilctGraph_t::vertex_descriptor vertex_t;
//typedef confilctGraph_t::edge_descriptor edge_t;

enum heuristics_type { NONE, CG, DG, WDG, STRATEGY_COUNT };

typedef tuple<int, int, int, int, bool> Constraint;
typedef tuple<int, int, int, int, int> Conflict; // a1, a2, loc1, loc2, timestep

enum class ConflictType{
    TILE_VERTEX, // 같은 시간, 같은 위치
    SECTION_CAP  // capacity 초과
    // TILE_EDGE (안씀)
};

struct SectionConflict{
    int agent1;
    int agent2;
    int section_id;
    int local_index;
    int timestep;
    ConflictType type;

    SectionConflict(int a1, int a2, int sec_id, int loc, int t, ConflictType type)
    : agent1(a1), agent2(a2), section_id(sec_id), local_index(loc), timestep(t), type(type) {}
};
// typedef vector<unordered_set<std::pair<int,int> > > ConstraintTable;
typedef tuple<int, int, bool> Interval; // [t_min, t_max), have conflicts or not

// <start_time, end_time, current_capacity>
typedef std::tuple<int, int, int> SecInterval;

#define INTERVAL_MAX 10000

ostream& operator<<(ostream& os, const Constraint& constraint);

ostream& operator<<(ostream& os, const Conflict& conflict);

ostream& operator<<(ostream& os, const SectionConflict& sectionconflict);

ostream& operator<<(ostream& os, const Interval& interval);

ostream& operator<<(ostream& os, const SecInterval secinterval);

inline std::ostream& operator<<(std::ostream& os, const SectionConflict& c) {
    os << "[Conflict: Agent " << c.agent1 << " vs Agent " << c.agent2 
       << " at Loc " << c.section_id << c.local_index << ", t=" << c.timestep << "]";
    return os;
}