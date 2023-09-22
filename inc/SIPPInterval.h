#pragma once
#include "common.h"

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
    SIPPInterval *get_first_interval(int location, int start_time = 0) {}
    vector<SIPPInterval *> get_intervals(int from, SIPPInterval *current_interval, int to) {}
    void insert_path(int agent_id, vector<PathEntry> &path, int start = 0, int length = INFINITY) {}
    void remove_path(int agent_id, vector<PathEntry> &path, int start = 0, int length = INFINITY) {}

private:
    vector<vector<SIPPInterval>> intervals_;

    void init_location(int location) { intervals_[location].push_back(SIPPInterval()); }
    void split(int agent_id, int location, int low, int high) {}
    void merge(int location, int low) {}

    inline int
    binary_search(int location, int low, int left = 0) const {}
    vector<SIPPInterval *> find_intervals(int from, SIPPInterval *current_interval, int to) const {}
};
