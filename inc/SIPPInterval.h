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
    SIPPIntervals(int map_size) : intervals_(map_size), clear_intervals_(1) {}

    int get_first_interval(int agent_id, int location, int start_time = 0);
    const vector<int> get_intervals(int from, int interval, int timestep, int to);
    const SIPPInterval *get_interval(int location, int index) const;
    void insert_path(int agent_id, vector<PathEntry> &path, int start = 0);
    void remove_path(int agent_id, vector<PathEntry> &path, int start = 0, int period = 0);

    void cleared_intervals(int current_timestep) const
    {
        for (int i = 0; i < (int)intervals_.size(); i++)
        {
            for (int j = 0; j < (int)intervals_[i].size(); j++)
            {
                if (intervals_[i][j].high > current_timestep && intervals_[i][j].agent_id != NO_AGENT)
                {
                    cerr << "Error: interval " << intervals_[i][j].low << " " << intervals_[i][j].high << " " << intervals_[i][j].agent_id << " is beyond current_timestep " << current_timestep << endl;
                    exit(1);
                }
            }
        }
    }

private:
    vector<vector<SIPPInterval>> intervals_;
    vector<int> clear_intervals_;

    void init_location(int location)
    {
        intervals_[location].emplace_back(0, MAX_TIMESTEP);
    }
    void split(int agent_id, int location, int low, int high);
    void merge(int location, int low);
    void truncate_interval(int agent_id, int location, int timestep);

    inline int
    binary_search(int location, int timestep) const;

    void validate_intervals(int location) const;
};
