#pragma once
#include "Instance.h"
#include "ConstraintTable.h"
#include "ReservationTable.h"
#include "MemoryPool.h"
#include "LLNode.h"
#include "common.h"

class SingleAgentSolver
{
public:
	uint64_t accumulated_num_expanded = 0;
	uint64_t accumulated_num_generated = 0;
	uint64_t accumulated_num_reopened = 0;
	uint64_t num_runs = 0;

	int num_collisions = -1;
	double runtime_build_CT = 0;  // runtimr of building constraint table
	double runtime_build_CAT = 0; // runtime of building conflict avoidance table

	int agent_id;
	int start_location;
	int goal_location;
	vector<int> my_heuristic;					  // this is the precomputed heuristic for this agent
	int compute_heuristic(int from, int to) const // compute admissible heuristic between two locations
	{
		return max(get_DH_heuristic(from, to), instance.getManhattanDistance(from, to));
	}
	const Instance &instance;

	// virtual Path findOptimalPath(const PathTable& path_table) = 0;
	// virtual Path findOptimalPath(const ConstraintTable& constraint_table, const PathTableWC& path_table) = 0;
	virtual Path findOptimalPath(const HLNode &node, const ConstraintTable &initial_constraints,
								 const vector<Path *> &paths, int agent, int lower_bound) = 0;
	virtual pair<Path, int> findSuboptimalPath(const HLNode &node, const ConstraintTable &initial_constraints,
											   const vector<Path *> &paths, int agent, int lowerbound, double w) = 0; // return the path and the lowerbound
	virtual Path findPath(const ConstraintTable &constraint_table, int planning_window_length = MAX_TIMESTEP) = 0;	  // return the path
	virtual Path findPath(ReservationTable &constraint_table, MemoryPool &memory_pool, int planning_window_length = MAX_TIMESTEP)
	{
		Path path;
		return path;
	} // return the path
	virtual Path findPath(SIPPIntervals &sipp_intervals, MemoryPool &memory_pool, int start = 0, int depth_limit = MAX_TIMESTEP)
	{
		Path path;
		return path;
	}; // return A path that minimizes collisions, breaking ties by cost

	void findMinimumSetofColldingTargets(vector<int> &goal_table, set<int> &A_target);
	virtual int getTravelTime(int start, int end, const ConstraintTable &constraint_table, int upper_bound) = 0;
	virtual string getName() const = 0;

	list<int> getNextLocations(int curr) const; // including itself and its neighbors
	list<int> getNeighbors(int curr) const { return instance.getNeighbors(curr); }
	uint64_t getNumExpanded() const { return num_expanded; }
	// int getStartLocation() const {return instance.start_locations[agent]; }
	// int getGoalLocation() const {return instance.goal_locations[agent]; }

	SingleAgentSolver(const Instance &instance, int agent) : instance(instance), agent_id(agent),
															 start_location(instance.start_locations[agent]),
															 goal_location(instance.goal_locations[agent])
	{
		compute_heuristics();
	}
	virtual ~SingleAgentSolver() = default;
	void reset()
	{
		if (num_generated > 0)
		{
			accumulated_num_expanded += num_expanded;
			accumulated_num_generated += num_generated;
			accumulated_num_reopened += num_reopened;
			num_runs++;
		}
		num_expanded = 0;
		num_generated = 0;
		num_reopened = 0;
	}

protected:
	uint64_t num_expanded = 0;
	uint64_t num_generated = 0;
	uint64_t num_reopened = 0;
	int min_f_val; // minimal f value in OPEN
	// int lower_bound; // Threshold for FOCAL
	double w = 1; // suboptimal bound

	void compute_heuristics();
	int get_DH_heuristic(int from, int to) const { return abs(my_heuristic[from] - my_heuristic[to]); }
};
