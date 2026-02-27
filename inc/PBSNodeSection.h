#pragma once
#include "common.h"
#include "PriorityGraph.h"
#include "SectionState.h"

class PBSNodeSection
{
public:
	// the following is used to comapre nodes in the OPEN list
	struct compare_node 
	{
		bool operator()(const PBSNodeSection* n1, const PBSNodeSection* n2) const
		{
            if (n1->f_val == n2->f_val)
            {
                return n1->num_of_collisions >= n2->num_of_collisions;
            }
			return n1->f_val >= n2->f_val;
		}
	};  

	typedef fibonacci_heap< PBSNodeSection*, compare<PBSNodeSection::compare_node> >::
	    handle_type open_handle_t;

	// conflicts in the current paths
	std::list<Conflict> conflicts;
	
	// The chosen conflict
	Conflict conflict;

	PBSNodeSection* parent;


    list< pair<int, SectionPath> > paths; // <agent_id, path>
    std::pair<int, int> priority; // a1 < a2

    PriorityGraph priorities;

	double g_val;
	double h_val;
	double f_val;
	size_t depth; // depath of this CT node
	size_t makespan; // makespan over all paths
	int num_of_collisions; // number of conflicts in the current paths
    int earliest_collision;

	uint64_t time_expanded;
	uint64_t time_generated;


    void print_priorities() const;

	void clear();

	PBSNodeSection(): parent(nullptr), g_val(0), h_val(0), earliest_collision(INT_MAX), time_expanded(0) {}
	~PBSNodeSection(){};


private:
};