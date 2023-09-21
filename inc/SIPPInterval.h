#pragma once
#include "common.h"

struct SIPPInterval
{
    int low;
    int high;
    int agent_id;
};

class SIPPIntervals:
{
public:
    // in the constructor initalise intervals_ with map_size
    SIPPIntervals(int map_size) {}

    vector<SIPPInterval> get_interval(int from, int to, int low, int high) {}

    void insert_path(int agent_id, vector<PathEntry> &path, int start = 0, int length = INFINITY) {}
    void remove_path(int agent_id, vector<PathEntry> &path, int start = 0, int length = INFINITY) {}

private:
    vector<vector<SIPPInterval>> intervals_;

    void split(int location, int index) {}
    void merge(int location, int index) {}
};
