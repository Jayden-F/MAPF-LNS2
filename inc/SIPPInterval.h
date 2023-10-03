#pragma once
#include "common.h"

#define NO_AGENT -1

struct SIPPInterval
{
    int low;
    int high;
    int agent_id;

    SIPPInterval() : low(0), high(MAX_TIMESTEP), agent_id(-1) {}
    SIPPInterval(int low, int high) : low(low), high(high), agent_id(-1) {}
    SIPPInterval(int low, int high, int agent_id) : low(low), high(high), agent_id(agent_id) {}
};

class SIPPIntervals
{
public:
    friend class TestSIPPInterval;

private:
    vector<vector<SIPPInterval>> intervals_;
    vector<int> clear_intervals_;

public:
    // in the constructor initalise intervals_ with map_size
    SIPPIntervals(int map_size) : intervals_(map_size), clear_intervals_(0) {}
    const int get_first_interval(int location, int start_time = 0);
    const vector<int> get_intervals(int from, int interval, int timestep, int to);
    inline const SIPPInterval *get_interval(int location, int index) { return &intervals_[location][index]; }
    void insert_path(int agent_id, Path &path, int start = 0, int horizon = MAX_TIMESTEP);
    void remove_path(int agent_id, Path &path, int start = 0, int period = 0, int horizon = MAX_TIMESTEP);
    void unreserve_goal(int agent_id, int location, int timestep);
    void reserve_goal(int agent_id, int location, int timestep);
    void cleared_intervals(int timestep) const
    {
        for (int i = 0; i < intervals_.size(); i++)
            for (int j = 0; j < intervals_[i].size(); j++)
                assert(intervals_[i][j].high <= timestep || intervals_[i][j].agent_id == NO_AGENT);
    }
    void agent_removed(int agent_id)
    {
        for (int location = 0; location < intervals_.size(); location++)
        {
            for (int interval = 0; interval < intervals_[location].size(); interval++)
            {
                if (intervals_[location][interval].agent_id == agent_id)
                {
                    cout << "agent " << agent_id << " is not removed from the location " << location << " [" << intervals_[location][interval].low << "," << intervals_[location][interval].high << endl;
                }
            }
        }
    }
    void clear()
    {
        for (int i = 0; i < intervals_.size(); i++)
            intervals_[i].clear();
    }

private:
    void init_location(int location) { intervals_[location].emplace_back(0, MAX_TIMESTEP); }
    void split(int agent_id, int location, int low, int high);
    void merge(int agent_id, int location, int low, int high);
    void truncate(int agent_id, int location, int timestep);
    void validate(int location) const;

    inline int
    binary_search(int location, int timestep) const
    {
        int left(0);
        int right(intervals_[location].size() - 1);
        int mid;

        while (left <= right)
        {
            mid = (left + right) / 2;
            if (intervals_[location][mid].high <= timestep)
                left = mid + 1;
            else if (intervals_[location][mid].low > timestep)
                right = mid - 1;
            else
                return mid;
        }

#ifdef DEBUG_MODE
        this->validate(location);
        cerr << "ERROR: binary_search failed to find interval " << timestep << endl;
#endif
        return -1;
    }
};

// Iteration 23, group size = 3, solution cost = 80279, remaining time = 594.479
// Find a target conflict where agent 27(of length 2772)traverses agent 17(of length 1037)'s target location 214014 at timestep 1180