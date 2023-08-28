#pragma once
#include <boost/functional/hash.hpp>
#include "LLNode.h"

class SIPPNode: public LLNode
{
public:
	// define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
	typedef boost::heap::pairing_heap< SIPPNode*, compare<SIPPNode::compare_node> >::handle_type open_handle_t;
	typedef boost::heap::pairing_heap< SIPPNode*, compare<SIPPNode::secondary_compare_node> >::handle_type focal_handle_t;
	open_handle_t open_handle;
	focal_handle_t focal_handle;
	int high_generation; // the upper bound with respect to generation
    int high_expansion; // the upper bound with respect to expansion
	bool collision_v;
	bool is_closed = false;
    int label = -1;
    int id = -1;
    SIPPNode() : LLNode() {}
	SIPPNode(int loc, int g_val, int h_val, SIPPNode* parent, int timestep, int high_generation, int high_expansion,
	        bool collision_v, int num_of_conflicts) :
            LLNode(loc, g_val, h_val, parent, timestep, num_of_conflicts), high_generation(high_generation),
            high_expansion(high_expansion), collision_v(collision_v), is_closed(false), label(-1), id(-1) {}
	// SIPPNode(const SIPPNode& other): LLNode(other), high_generation(other.high_generation), high_expansion(other.high_expansion),
        //                              collision_v(other.collision_v) {}
	~SIPPNode() {}

	void copy(const SIPPNode& other) // copy everything except for handles
    {
		cout << "search node copied" << endl;
	    LLNode::copy(other);
        high_generation = other.high_generation;
        high_expansion = other.high_expansion;
        collision_v = other.collision_v;
		is_closed = other.is_closed;
		label = other.label;
		id = other.id;
    }
	void close(){
		is_closed = true;
	}
	void reset(){
		location = -1;
		g_val = 0;
		h_val = 0;
		parent = nullptr;
		timestep = 0;
		high_generation = 0;
		high_expansion = 0;
	    collision_v = 0;
		num_of_conflicts = 0;
	}
	// The following is used by for generating the hash value of a nodes
	struct NodeHasher
	{
		std::size_t operator()(const SIPPNode* n) const
		{
            size_t seed = 0;
            boost::hash_combine(seed, n->location);
            boost::hash_combine(seed, n->high_generation);
            return seed;
		}
	};

	// The following is used for checking whether two nodes are equal
	// we say that two nodes, s1 and s2, are equal if
	// both are non-NULL and agree on the id and timestep
	struct eqnode
	{
		bool operator()(const SIPPNode* n1, const SIPPNode* n2) const
		{
			return (n1 == n2) ||
			            (n1 && n2 && n1->location == n2->location &&
				        n1->wait_at_goal == n2->wait_at_goal &&
				        n1->is_goal == n2->is_goal &&
                         n1->high_generation == n2->high_generation);
                        //max(n1->timestep, n2->timestep) <
                        //min(get<1>(n1->interval), get<1>(n2->interval))); //overlapping time intervals
		}
	};
};
