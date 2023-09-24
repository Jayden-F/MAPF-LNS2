#include "SIPPInterval.h"
#include "common.h"

#define NO_AGENT -1

const SIPPInterval *SIPPIntervals::get_first_interval(int agent_id, int location, int start_time)
{
    if (intervals_[location].empty())
    {
        this->init_location(location);
    }

    int index = this->binary_search(location, start_time);
    if (index == -1)
        return nullptr;

    if (intervals_[location][index].agent_id != NO_AGENT)
        return nullptr;

    return &intervals_[location][index];
}

vector<const SIPPInterval *> SIPPIntervals::get_intervals(int from, int to, int low, int high)
{
    if (intervals_[to].empty())
    {
        this->init_location(to);
    }

    return this->find_intervals(from, to, low, high);
}

void SIPPIntervals::insert_path(int agent_id, vector<PathEntry> &path, int start)
{
    if (path.empty())
        return;

    int location = path.front().location;
    int low(start);
    int high(start);

    for (int t = 0; t < path.size(); t++)
    {
        if (location != path[t].location)
        {
            // cout << agent_id << " splitting: " << location << " @ [" << low << "," << high << ")" << endl;
            this->split(agent_id, location, low, high);
            low = high;
            location = path[t].location;
        }
        high++;
    }
}

void SIPPIntervals::remove_horizon(int agent_id, vector<PathEntry> &path, int start, int period)
{
    int location = path[period].location;
    this->truncate_interval(agent_id, location, start + period);

    for (int t = period + 1; t < path.size(); t++)
    {
        if (location != path[t].location)
        {
            // cout << agent_id << " merging: " << path[t].location << " @ [" << start + t << "," << start + t + 1 << ")" << endl;
            this->merge(path[t].location, start + t);
            location = path[t].location;
        }
    }
}

void SIPPIntervals::remove_path(int agent_id, vector<PathEntry> &path, int start)
{
    int location = path[0].location;
    this->truncate_interval(agent_id, location, start);

    // loop over each location in the path use binary search to find the interval and use merge to remove it.
    for (int t = 1; t < path.size(); t++)
    {
        if (location != path[t].location)
        {
            // cout << agent_id << " merging: " << path[t].location << " @ [" << start + t << "," << start + t + 1 << ")" << endl;
            this->merge(path[t].location, start + t);
            location = path[t].location;
        }
    }
}

void SIPPIntervals::truncate_interval(int agent_id, int location, int timestep)
{

    // cout << agent_id << " truncating: " << location << " @ [" << timestep << "," << timestep + 1 << ")" << endl;
    // this->validate_intervals(location);

    int index = this->binary_search(location, timestep);

    int length = intervals_[location][index].high - intervals_[location][index].low;

    // Shorten Interval
    if (length > 1)
    {
        intervals_[location][index + 1].low = timestep;
        intervals_[location][index].high = timestep;

        // this->validate_intervals(location);
        return;
    }

    // Remove interval entirely
    if (index >= 1 &&
        intervals_[location][index - 1].agent_id == NO_AGENT &&
        intervals_[location][index + 1].agent_id == NO_AGENT)
    {
        intervals_[location][index - 1].high = intervals_[location][index + 1].high;
        intervals_[location].erase(intervals_[location].begin() + index, intervals_[location].begin() + index + 2);

        // this->validate_intervals(location);
        return;
    }

    if (index >= 1 &&
        intervals_[location][index - 1].agent_id == NO_AGENT)
    {
        intervals_[location][index - 1].high = intervals_[location][index].high;
        intervals_[location].erase(intervals_[location].begin() + index);
        // this->validate_intervals(location);
        return;
    }

    if (intervals_[location][index + 1].agent_id == NO_AGENT)
    // Absorb following interval if it is safe
    {
        intervals_[location][index].agent_id = NO_AGENT;
        intervals_[location][index].high = intervals_[location][index + 1].high;
        intervals_[location].erase(intervals_[location].begin() + index + 1);

        // this->validate_intervals(location);
        return;
    }

    intervals_[location][index].agent_id = NO_AGENT;
    // this->validate_intervals(location);
    return;
}

void SIPPIntervals::split(int agent_id, int location, int low, int high)
{

    if (intervals_[location].empty())
    {
        this->init_location(location);
    }

    // this->validate_intervals(location);

    int interval_index = this->binary_search(location, low);
    assert(intervals_[location][interval_index].agent_id == NO_AGENT);

    // Merge with previous interval removing current interval
    // [1246,1250): 83, [1250,1251): -1 , [1251,1252): 12
    // [1246,1251): 83, [1251,1252): 12
    if (interval_index > 0 &&
        intervals_[location][interval_index].low == low &&
        intervals_[location][interval_index - 1].agent_id == agent_id &&
        intervals_[location][interval_index].high - intervals_[location][interval_index].low == 1)
    {
        intervals_[location][interval_index - 1].high = high;
        intervals_[location].erase(intervals_[location].begin() + interval_index);
        // this->validate_intervals(location);
        return;
    }

    // Merge with previous interval truncating current interval.
    // [1246,1250): 83, [1250,1255): -1 , [1255,1252): 12
    // [1246,1251): 83, [1251,1255): -1 , [1255,1252): 12
    if (interval_index > 0 &&
        intervals_[location][interval_index].low == low &&
        intervals_[location][interval_index - 1].agent_id == agent_id)
    {
        intervals_[location][interval_index - 1].high = high;
        intervals_[location][interval_index].low = high;

        // this->validate_intervals(location);
        return;
    }

    // Proposed interval matches current interval
    if (intervals_[location][interval_index].high == high &&
        intervals_[location][interval_index].low == low)
    {
        intervals_[location][interval_index].agent_id = agent_id;

        // this->validate_intervals(location);
        return;
    }

    // Interval shares a high interval
    if (intervals_[location][interval_index].high == high)
    {
        intervals_[location][interval_index].high = low;
        intervals_[location].emplace(intervals_[location].begin() + interval_index + 1, low, high, agent_id);

        // this->validate_intervals(location);
        return;
    }

    // Interval shares a low interval
    if (intervals_[location][interval_index].low == low)
    {
        intervals_[location][interval_index].low = high;
        intervals_[location].emplace(intervals_[location].begin() + interval_index, low, high, agent_id);

        // this->validate_intervals(location);
        return;
    }

    // interval is contained within current interval
    int new_high = intervals_[location][interval_index].high;
    intervals_[location][interval_index].high = low;
    intervals_[location].emplace(intervals_[location].begin() + interval_index + 1, high, new_high, NO_AGENT);
    intervals_[location].emplace(intervals_[location].begin() + interval_index + 1, low, high, agent_id);

    // this->validate_intervals(location);
    return;
}

void SIPPIntervals::merge(int location, int low)
{
    // this->validate_intervals(location);

    assert(!intervals_[location].empty());
    int index = this->binary_search(location, low);
    if (intervals_[location][index].agent_id == NO_AGENT)
        return;

    // Two Neighbouring Safe Intervals
    if (index > 0 &&
        index < intervals_[location].size() - 1 &&
        intervals_[location][index - 1].agent_id == NO_AGENT &&
        intervals_[location][index + 1].agent_id == NO_AGENT)
    {
        intervals_[location][index - 1].high = intervals_[location][index + 1].high;
        intervals_[location].erase(intervals_[location].begin() + index, intervals_[location].begin() + index + 2);

        // this->validate_intervals(location);
        return;
    }

    // One Neighbouring Safe Interval

    // Early
    if (index > 0 &&
        intervals_[location][index - 1].agent_id == NO_AGENT)
    {
        intervals_[location][index - 1].high = intervals_[location][index].high;
        intervals_[location].erase(intervals_[location].begin() + index);
        // this->validate_intervals(location);
        return;
    }

    // Late
    if (index < intervals_[location].size() - 1 &&
        intervals_[location][index + 1].agent_id == NO_AGENT)
    {
        intervals_[location][index + 1].low = intervals_[location][index].low;
        intervals_[location].erase(intervals_[location].begin() + index);
        // this->validate_intervals(location);
        return;
    }

    // No Neighbouring Safe Intervals
    intervals_[location][index].agent_id = NO_AGENT;
    // this->validate_intervals(location);
    return;
}

inline int
SIPPIntervals::binary_search(int location, int low, int left) const
{
    int right(intervals_[location].size() - 1);
    int mid;

    // cout << "   binary_search: " << location << " @ [" << low << "," << low + 1 << ")" << endl;
    // this->validate_intervals(location);

    while (left <= right)
    {
        mid = (left + right) / 2;
        if (intervals_[location][mid].high <= low)
            left = mid + 1;
        else if (intervals_[location][mid].low > low)
            right = mid - 1;
        else
        {
            assert(intervals_[location][mid].low <= low && intervals_[location][mid].high > low);
            return mid;
        }
    }

    // cerr << "ERROR: binary_search failed to find interval" << endl;
    return -1;
}

vector<const SIPPInterval *> SIPPIntervals::find_intervals(int from, int to, int low, int high) const
{
    //  Otherwise search for interval between low and high
    int next_index = this->binary_search(from, low - 1) + 1;
    int low_index = this->binary_search(to, low);
    int high_index = this->binary_search(to, high - 1);
    vector<const SIPPInterval *> result;
    result.reserve(high_index - low_index + 1);

    for (int i = low_index; i <= high_index; i++)
    {
        // Vertex Conflict
        if (intervals_[to][i].agent_id != NO_AGENT)
            continue;

        // Edge Conflict
        if (i - 1 > 0 &&
            next_index < (int)intervals_[from].size() &&
            intervals_[to][i - 1].agent_id == intervals_[from][next_index].agent_id)
            continue;

        result.push_back(&intervals_[to][i]);
    }
    return result;
}

void SIPPIntervals::validate_intervals(int location) const
{
    cout << "   location: " << location << endl
         << "   ";

    if (intervals_[location][0].low != 0)
    {
        cerr << "ERROR: interval does not start at 0" << endl;
        exit(1);
    }

    for (int i = 0; i < intervals_[location].size() - 1; i++)
        cout << "[" << intervals_[location][i].low << "," << intervals_[location][i].high << "): " << intervals_[location][i].agent_id << " , ";
    cout << "[" << intervals_[location].back().low << "," << intervals_[location].back().high << "): " << intervals_[location].back().agent_id << " " << endl;

    for (int i = 0; i < intervals_[location].size() - 1; i++)
    {
        if (intervals_[location][i].agent_id == intervals_[location][i + 1].agent_id)
        {
            cerr << "ERROR: interval " << i << " and " << i + 1 << " have the same agent_id" << endl;
            exit(1);
        }
        if (intervals_[location][i].high > intervals_[location][i + 1].low)
        {
            cerr << "ERROR: interval " << i << " and " << i + 1 << " overlap" << endl;
            exit(1);
        }
    }

    if (intervals_[location].back().high != MAX_TIMESTEP)
    {
        cerr << "ERROR: interval does not end at MAX_TIMESTEP" << endl;
        exit(1);
    }
}
