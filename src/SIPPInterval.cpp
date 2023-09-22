#include "SIPPInterval.h"
#include "common.h"

#define NO_AGENT -1

SIPPInterval *SIPPIntervals::get_first_interval(int location, int start_time = 0)
{
    if (intervals_[location].empty())
    {
        SIPPIntervals::init_location(location);
    }

    int index = SIPPIntervals::binary_search(location, start_time);
    if (index == -1)
        return nullptr;

    return &intervals_[location][index];
}

vector<SIPPInterval *> SIPPIntervals::get_intervals(int from, SIPPInterval *current_interval, int to)
{
    if (intervals_[to].empty())
    {
        SIPPIntervals::init_location(to);
    }

    return SIPPIntervals::find_intervals(from, current_interval, to);
}

void SIPPIntervals::insert_path(int agent_id, vector<PathEntry> &path, int start = 0, int length = INFINITY)
{
    if (path.empty())
        return;

    PathEntry *location = &path[start];
    int low(0);
    int high(0);

    for (int t = start; t < path.size() && t < start + length; t++)
    {
        if (location->location != path[t].location)
        {
            split(agent_id, location->location, low, high);
            low = t;
            location = &path[t];
        }
        else
            high = t;
    }
}

void SIPPIntervals::remove_path(int agent_id, vector<PathEntry> &path, int start = 0, int length = INFINITY)
{
    PathEntry *location = nullptr;
    // loop over each location in the path use binary search to find the interval and use merge to remove it.
    for (int t = start; t < path.size() && t < start + length; t++)
    {
        if (location->location != path[t].location)
        {
            location = &path[t];
            merge(location->location, t);
        }
    }
}

void SIPPIntervals::split(int agent_id, int location, int low, int high)
{
    int interval_index = SIPPIntervals::binary_search(location, low);

    // Proposed interval matches current interval
    if (intervals_[location][interval_index].high == high &&
        intervals_[location][interval_index].low == low)
    {
        intervals_[location][interval_index].agent_id = agent_id;
        return;
    }

    // Interval shares a high interval
    if (intervals_[location][interval_index].high == high)
    {
        intervals_[location][interval_index].high = low;
        intervals_[location].insert(intervals_[location].begin() + interval_index, SIPPInterval(low, high, agent_id));
        return;
    }

    // Interval shares a low interval
    if (intervals_[location][interval_index].low == low)
    {
        intervals_[location][interval_index].low = high;
        intervals_[location].insert(intervals_[location].begin() + interval_index - 1, SIPPInterval(low, high, agent_id));
        return;
    }

    // interval is contained within current interval
    int new_high = intervals_[location][interval_index].high;
    intervals_[location][interval_index].high = low;
    intervals_[location].insert(intervals_[location].begin() + interval_index, SIPPInterval(low, high, agent_id));
    intervals_[location].insert(intervals_[location].begin() + interval_index + 1, SIPPInterval(high, new_high, NO_AGENT));
}

void SIPPIntervals::merge(int location, int low)
{
    int index = SIPPIntervals::binary_search(location, low);

    // Two Neighbouring Safe Intervals
    if (index > 0 &&
        index < intervals_[location].size() - 1 &&
        intervals_[location][index - 1].agent_id == NO_AGENT &&
        intervals_[location][index + 1].agent_id == NO_AGENT)
    {
        intervals_[location][index - 1].high = intervals_[location][index + 1].high;
        intervals_[location].erase(intervals_[location].begin() + index, intervals_[location].begin() + index + 1);
        return;
    }

    // One Neighbouring Safe Interval

    // Early
    if (index > 0 &&
        intervals_[location][index - 1].agent_id == NO_AGENT)
        intervals_[location][index - 1].high = intervals_[location][index].high;

    // Late
    if (index < intervals_[location].size() - 1 &&
        intervals_[location][index + 1].agent_id == NO_AGENT)
        intervals_[location][index].low = low;

    intervals_[location].erase(intervals_[location].begin() + index);

    return;
}

inline int
SIPPIntervals::binary_search(int location, int low, int left) const
{
    int right(intervals_[location].size() - 1);
    int mid;

    while (left <= right)
    {
        mid = (left + right) / 2;
        if (intervals_[location][mid].high < low)
            left = mid + 1;
        else if (intervals_[location][mid].low > low)
            right = mid - 1;
        else
            return mid;
    }

    // cerr << "ERROR: binary_search failed to find interval" << endl;
    return -1;
}

vector<SIPPInterval *> SIPPIntervals::find_intervals(int from, SIPPInterval *current_interval, int to) const
{
    //  Otherwise search for interval between low and high
    int low_index = SIPPIntervals::binary_search(to, current_interval->low);
    int high_index = SIPPIntervals::binary_search(to, current_interval->high - 1, low_index);
    vector<SIPPInterval *> result(high_index - low_index + 1);

    for (int i = low_index; i <= high_index; i++)
    {
        // Vertex Conflict
        if (intervals_[to][i].agent_id != NO_AGENT)
            continue;

        // Edge Conflict
        if (i - 1 > 0 &&
            current_interval + 1 > &intervals_[from][0] &&
            intervals_[to][i - 1].agent_id == (current_interval + 1)->agent_id)
            continue;

        result.push_back(current_interval + 1);
    }
    return result;
}
