#pragma once
#include <boost/functional/hash.hpp>
#include "SingleAgentSolver.h"
#include "ReservationTable.h"
#include "SIPPNode.h"
#include "MemoryPool.h"

class SIPP : public SingleAgentSolver
{
public:
	// find path by SIPP
	// Returns a shortest path that satisfies the constraints of the give node  while
	// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
	// lowerbound is an underestimation of the length of the path in order to speed up the search.
	// Path findOptimalPath(const PathTable& path_table) {return Path(); } // TODO: To implement
	// Path findOptimalPath(const ConstraintTable& constraint_table, const PathTableWC& path_table);
	Path findOptimalPath(const HLNode &node, const ConstraintTable &initial_constraints,
						 const vector<Path *> &paths, int agent, int lowerbound);
	pair<Path, int> findSuboptimalPath(const HLNode &node, const ConstraintTable &initial_constraints,
									   const vector<Path *> &paths, int agent, int lowerbound, double w);					   // return the path and the lowerbound
	Path findPath(const ConstraintTable &constraint_table, int depth_limit = INFINITY);										   // return A path that minimizes collisions, breaking ties by cost
	Path findPath(SIPPIntervals &sipp_intervals, MemoryPool &memory_pool, int start_timestep = 0, int depth_limit = INFINITY); // return A path that minimizes collisions, breaking ties by cost

	int getTravelTime(int start, int end, const ConstraintTable &constraint_table, int upper_bound);

	string getName() const { return "SIPP"; }

	SIPP(const Instance &instance, int agent) : SingleAgentSolver(instance, agent) {}

private:
	// define typedefs and handles for heap
	typedef boost::heap::pairing_heap<SIPPNode *, boost::heap::compare<LLNode::compare_node>> heap_open_t;
	typedef boost::heap::pairing_heap<SIPPNode *, boost::heap::compare<LLNode::secondary_compare_node>> heap_focal_t;
	heap_open_t open_list;
	heap_focal_t focal_list;

	// define typedef for hash_map
	typedef boost::unordered_map<SIPPNode *, list<SIPPNode *>, SIPPNode::NodeHasher, SIPPNode::eqnode> hashtable_t;
	hashtable_t allNodes_table;
	list<SIPPNode *> useless_nodes;
	// Path findNoCollisionPath(const ConstraintTable& constraint_table);

	void updatePath(const LLNode *goal, std::vector<PathEntry> &path);
	void updatePath(const LLNode *goal, std::vector<PathEntry> &path, int start_time);

	inline void pushNodeToOpen(SIPPNode *node);
	inline void pushNodeToOpenAndFocal(SIPPNode *node);
	inline void pushNodeToFocal(SIPPNode *node);
	inline void eraseNodeFromLists(SIPPNode *node);
	void updateFocalList();
	void releaseNodes();
	bool dominanceCheck(SIPPNode *new_node);
	bool dominanceCheck(int id, SIPPNode *new_node, MemoryPool &memory_pool);
	void printSearchTree() const;
};
