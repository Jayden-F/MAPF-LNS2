// This is used by SIPP
#pragma once
#include "ConstraintTable.h"

struct Interval:     
    tuple<int, int, bool, int>
{
    using tuple::tuple;
    constexpr Interval(int a, int b, bool c) noexcept : tuple(a, b, c, -1)
    { }
    constexpr Interval(tuple<int, int, bool, int> t) noexcept : tuple(t)
    { }
    using tuple::operator=;
}; // [t_min, t_max), has collision

struct Collision_Interval: 
    tuple<int, int, int, int, bool, bool> {
    using tuple::tuple;
    constexpr Collision_Interval(int a, int b, int c, bool e, bool f) noexcept : tuple(a, b, c, -1, e, f)
    { }
    using tuple::operator=;
};

class ReservationTable
{
public:
    int goal_location;
    const ConstraintTable& constraint_table;

    ReservationTable(const ConstraintTable& constraint_table, int goal_location) :
        constraint_table(constraint_table), goal_location(goal_location), sit(constraint_table.map_size) {}

    list<Collision_Interval> get_safe_intervals(int from, int to, int lower_bound, int upper_bound, bool clear_intervals = false);
    Interval get_first_safe_interval(size_t location, bool clear_intervals = false);
    bool find_safe_interval(Interval& interval, size_t location, int t_min, bool clear_intervals = false);

    void clear(){
        goal_location = NO_AGENT;
        for (int i = 0; i < sit.size(); i++)
            if (!sit[i].empty())
                sit[i].clear();

        // int map_size = (int)sit.size();
        // sit.clear();
        // sit.resize(map_size);
    }

private:
	// Safe Interval Table (SIT)
	typedef vector< list<Interval> > SIT;
    SIT sit; // location -> [t_min, t_max), num_of_collisions

    void insert2SIT(int location, int t_min, int t_max);
    void insertSoftConstraint2SIT(int location, int t_min, int t_max);
	// void mergeIntervals(list<Interval >& intervals) const;
	void updateSIT(int location); // update SIT at the given location
    int get_earliest_arrival_time(int from, int to, int lower_bound, int upper_bound) const;
    int get_earliest_no_collision_arrival_time(int from, int to, const Interval& interval,
                                               int lower_bound, int upper_bound) const;
};
