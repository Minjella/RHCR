#pragma once
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "common.h"


class PriorityGraph
{
public:
    double runtime;

    void clear();
    bool empty() const {return G.empty(); }
    void copy(const PriorityGraph& other);
    void copy(const PriorityGraph& other, const vector<bool>& excluded_nodes);
    void add(int from, int to); // from is lower than to
    void remove(int from, int to); // from is lower than to
    bool connected(int from, int to) const;
    boost::unordered_set<int> get_reachable_nodes(int root); // root 보다 높은 agent를 모두 반환

    void save_as_digraph(std::string fname) const;
    typedef boost::unordered_map<int, boost::unordered_set<int> > PGraph_t;

    void update_number_of_lower_nodes(vector<int>& lower_nodes, int node) const; // node agent 보다 우선순위가 낮은 노드 개수 계산

    PGraph_t G;

    // TODO:  connected components
};

