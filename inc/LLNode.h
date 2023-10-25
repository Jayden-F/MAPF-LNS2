#pragma once
#include "common.h"

class LLNode // low-level node
{
public:
	int location = -1;
	int g_val = 0;
	int h_val = 0;
	LLNode *parent = nullptr;
	int timestep = 0;
	int num_of_conflicts = 0;
	bool in_openlist = false;
	bool wait_at_goal = false; // the action is to wait at the goal vertex or not. This is used for >lenghth constraints
	bool is_goal = false;
	bool is_closed = false;
	int label = -1;
	uint64_t id = 0;
	// the following is used to comapre nodes in the OPEN list
	struct compare_node
	{
		// returns true if n1 > n2 (note -- this gives us *min*-heap).
		inline bool operator()(const LLNode *n1, const LLNode *n2) const
		{
			if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
			{
				if (n1->h_val == n2->h_val)
				{
					return rand() % 2 == 0; // break ties randomly
				}
				return n1->h_val < n2->h_val; // break ties towards smaller h_vals (closer to goal location)
			}
			return n1->g_val + n1->h_val < n2->g_val + n2->h_val;
		}
	}; // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

	// the following is used to compare nodes in the FOCAL list
	struct secondary_compare_node
	{
		bool operator()(const LLNode *n1, const LLNode *n2) const // returns true if n1 > n2
		{
			if (n1->num_of_conflicts == n2->num_of_conflicts)
			{
				if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
				{
					if (n1->h_val == n2->h_val)
					{
						return rand() % 2 == 0; // break ties randomly
					}
					return n1->h_val >= n2->h_val; // break ties towards smaller h_vals (closer to goal location)
				}
				return n1->g_val + n1->h_val >= n2->g_val + n2->h_val; // break ties towards smaller f_vals (prefer shorter solutions)
			}
			return n1->num_of_conflicts >= n2->num_of_conflicts; // n1 > n2 if it has more conflicts
		}
	}; // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)

	LLNode() {}
	LLNode(int id) : id(id), label(-1), location(-1), priority_(UINT32_MAX) {}
	LLNode(int location, int g_val, int h_val, LLNode *parent, int timestep, int num_of_conflicts) : location(location), g_val(g_val), h_val(h_val), parent(parent), timestep(timestep),
																									 num_of_conflicts(num_of_conflicts), is_closed(false), label(-1), id(0), priority_(UINT32_MAX) {}
	LLNode(const LLNode &other) { copy(other); }
	// LLNode(const LLNode &) = delete;

	~LLNode() = default;

	void copy(const LLNode &other)
	{
		location = other.location;
		g_val = other.g_val;
		h_val = other.h_val;
		parent = other.parent;
		timestep = other.timestep;
		num_of_conflicts = other.num_of_conflicts;
		wait_at_goal = other.wait_at_goal;
		is_goal = other.is_goal;
		is_closed = other.is_closed;
		label = other.label;
		id = other.id;
		priority_ = other.priority_;
	}
	inline int getFVal() const { return g_val + h_val; }

	void reset()
	{
		location = -1;
		g_val = 0;
		h_val = 0;
		parent = 0;
		timestep = 0;
		num_of_conflicts = 0;
		wait_at_goal = false;
		is_goal = 0;
		is_closed = false;
		label = -1;
		id = 0;
		priority_ = UINT32_MAX;
	}

	void close()
	{
		is_closed = true;
	}

	inline void
	print(std::ostream &out) const
	{
		out << "id=" << id << " loc=" << location << " g=" << g_val << " h=" << h_val << " f=" << getFVal() << " t=" << timestep << " #conf=" << num_of_conflicts << " inOL=" << in_openlist << " is_goal=" << is_goal << " wait=" << wait_at_goal << " closed=" << is_closed << " parent=" << parent << " label=" << label << endl;
	}

	inline uint32_t
	get_priority() const
	{
		return priority_;
	}

	inline void
	set_priority(uint32_t priority) { priority_ = priority; }

private:
	uint32_t priority_;
};

std::ostream &operator<<(std::ostream &os, const LLNode &node);