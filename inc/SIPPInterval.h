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
    // in the constructor initalise intervals_ with map_size
    SIPPIntervals(int map_size) : intervals_(map_size) {}

    const SIPPInterval *get_first_interval(int agent_id, int location, int start_time = 0);
    vector<const SIPPInterval *> get_intervals(int from, int to, int low, int high);
    void insert_path(int agent_id, vector<PathEntry> &path, int start = 0);
    void remove_path(int agent_id, vector<PathEntry> &path, int start = 0);
    void remove_horizon(int agent_id, vector<PathEntry> &path, int start, int period);
    void truncate_interval(int agent_id, int location, int timestep);

    // Make sure their are not invtervals beyond current_timestep
    void cleared_intervals(int current_timestep) const
    {
        for (auto &location : intervals_)
        {
            for (auto &interval : location)
            {
                if (interval.low >= current_timestep && interval.agent_id != NO_AGENT)
                {
                    cerr << "Error: interval " << interval.low << " " << interval.high << " " << interval.agent_id << " is beyond current_timestep " << current_timestep << endl;
                    // exit(1);
                }
            }
        }
    }

private:
    vector<vector<SIPPInterval>> intervals_;

    void init_location(int location)
    {
        // intervals_[location].reserve(10000);
        intervals_[location].emplace_back(0, MAX_TIMESTEP);
    }
    void split(int agent_id, int location, int low, int high);
    void merge(int location, int low);

    inline int
    binary_search(int location, int low, int left = 0) const;
    vector<const SIPPInterval *> find_intervals(int from, int to, int low, int high) const;

    void validate_intervals(int location) const;
};
