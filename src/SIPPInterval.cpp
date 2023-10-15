#include "SIPPInterval.h"
#include "common.h"

// #define DEBUG_MODE

const int SIPPIntervals::get_first_interval(int location, int start_time)
{
    auto &location_intervals = intervals_[location];

    if (location_intervals.empty())
        this->init_location(location);

    int index(this->binary_search(location, start_time));

    if (location_intervals[index].agent_id != NO_AGENT)
        return -1;

    return index;
}

const vector<int> SIPPIntervals::get_intervals(int from, int interval, int timestep, int to)
{
    if (intervals_[to].empty())
        this->init_location(to);

    int low_index(this->binary_search(to, timestep));
    int high_index(this->binary_search(to, intervals_[from][interval].high - 1));

    this->clear_intervals_.clear();
    this->clear_intervals_.reserve(high_index - low_index + 1);

    for (int i = low_index; i <= high_index; i++)
    {
        // Vertex Conflict
        if (intervals_[to][i].agent_id != NO_AGENT)
        {
            continue;
        }
        // Edge Conflict
        if (
            i - 1 >= 0 &&
            interval + 1 < intervals_[from].size() &&
            intervals_[to][i - 1].agent_id != NO_AGENT &&
            intervals_[from][interval + 1].agent_id != NO_AGENT &&
            intervals_[to][i - 1].agent_id == intervals_[from][interval + 1].agent_id &&
            intervals_[from][interval].high == intervals_[to][i].low)
        {
            continue;
        }
        clear_intervals_.push_back(i);
    }
    return clear_intervals_;
}

void SIPPIntervals::insert_path(int agent_id, Path &path, int start, int horizon)
{
    if (path.empty())
        return;

    int location = path[0].location;
    int low(start);
    int high(start);

    int t_max = min((int)path.size() - 1, horizon);
    for (int t = 0; t <= t_max; t++)
    {
        if (location != path[t].location || t == t_max)
        {
#ifdef DEBUG_MODE
            cout << agent_id << " splitting: " << location << " @ [" << low << "," << high << ")" << endl;
#endif
            this->split(agent_id, location, low, high);
            low = high;
            location = path[t].location;
        }
        high++;
    }
}

void SIPPIntervals::remove_path(int agent_id, Path &path, int start, int period, int horizon)
{
    if (period >= path.size())
        return;
    // truncate first location as might be shared with previous window.
    this->truncate(agent_id, path[period].location, start + period);

    int location = path[period + 1].location;
    int low(start + period + 1);  // Low timestep of interval
    int high(start + period + 1); // High timestep of interval

    int t_max = min((int)path.size() - 1, horizon);
    for (int t = period + 1; t <= t_max; t++)
    {
        if (location != path[t].location || t == t_max)
        {
#ifdef DEBUG_MODE
            cout << agent_id << " merging: " << location << " @ [" << low << "," << high << ")" << endl;
#endif
            this->merge(agent_id, location, low, high);
            low = high;
            location = path[t].location;
        }
        high++;
    }
}

void SIPPIntervals::unreserve_goal(int agent_id, int location, int timestep)
{

    auto &location_intervals = intervals_[location];

#ifdef DEBUG_MODE
    this->validate(location);
#endif
    int index = this->binary_search(location, timestep);

    if (index > 0 &&
        location_intervals[index - 1].agent_id == NO_AGENT)
    {
        location_intervals[index - 1].high = MAX_TIMESTEP;
        location_intervals.erase(location_intervals.end() - 1);
#ifdef DEBUG_MODE
        cout << "agent: " << agent_id << " removing reservation on goal location: " << location << endl;
        this->validate(location);
#endif
        return;
    }

    location_intervals[index].agent_id = NO_AGENT;

#ifdef DEBUG_MODE
    cout << "agent: " << agent_id << " removing reservation on goal location: " << location << endl;
    this->validate(location);
#endif
}

void SIPPIntervals::reserve_goal(int agent_id, int location, int timestep)
{

#ifdef DEBUG_MODE
    cout << "agent: " << agent_id << " reservation on goal location: " << location << " at timestep:" << timestep << endl;
#endif
    auto &location_intervals = intervals_[location];

    if (location_intervals.empty())
    {
        this->init_location(location);
    }

    if (!this->is_location_clear(location, timestep))
    {
        this->validate(location);
        cout << "location cannot be reservered at timestep: " << timestep << " by agent: " << agent_id << endl;
        exit(-1);
    }

    int index = location_intervals.size() - 1;

    // location does not share timestep
    if (location_intervals[index].low < timestep)
    {
        location_intervals[index].high = timestep;
        location_intervals.emplace_back(timestep, MAX_TIMESTEP, agent_id);
#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // location is reserved by current agent extending reservation
    if (index > 0 &&
        location_intervals[index - 1].agent_id == agent_id &&
        location_intervals[index].low == timestep)
    {
        location_intervals[index - 1].high = MAX_TIMESTEP;
        location_intervals.erase(location_intervals.begin() + index);
#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // location is
    if (location_intervals[index].low == timestep)
    {
        location_intervals[index].agent_id = agent_id;

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    cout << "Error reserving goal" << endl;
    exit(-1);
}

void SIPPIntervals::truncate(int agent_id, int location, int timestep)
{
#ifdef DEBUG_MODE
    cout << agent_id << " truncating: " << location << " @ [" << timestep << "," << timestep + 1 << ")" << endl;
    this->validate(location);
#endif

    auto &location_intervals = intervals_[location];

    int index = this->binary_search(location, timestep);
    if (location_intervals[index].agent_id != agent_id)
    {
        return;
    }

    int length = location_intervals[index].high - location_intervals[index].low;

    // Interval early interval off at timestep
    if (length > 1 && location_intervals[index].low < timestep)
    {
        location_intervals[index + 1].low = timestep;
        location_intervals[index].high = timestep;

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Remove interval entirely
    if (index > 0 &&
        location_intervals[index - 1].agent_id == NO_AGENT &&
        location_intervals[index + 1].agent_id == NO_AGENT)
    {
        location_intervals[index - 1].high = location_intervals[index + 1].high;
        location_intervals.erase(location_intervals.begin() + index, location_intervals.begin() + index + 2);

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Merge with previous interval if it is safe
    if (index > 0 &&
        location_intervals[index - 1].agent_id == NO_AGENT)
    {
        location_intervals[index - 1].high = location_intervals[index].high;
        location_intervals.erase(location_intervals.begin() + index);

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Merge with next interval if it is safe
    if (location_intervals[index + 1].agent_id == NO_AGENT)
    {
        location_intervals[index].agent_id = NO_AGENT;
        location_intervals[index].high = location_intervals[index + 1].high;
        location_intervals.erase(location_intervals.begin() + index + 1);

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Allocate interval to No Agent
    location_intervals[index].agent_id = NO_AGENT;
#ifdef DEBUG_MODE
    this->validate(location);
#endif
    return;
}

void SIPPIntervals::split(int agent_id, int location, int low, int high)
{

    auto &location_intervals = intervals_[location];

    if (location_intervals.empty())
    {
        this->init_location(location);
    }

#ifdef DEBUG_MODE
    this->validate(location);
#endif

    int interval_index = this->binary_search(location, low);

    if (location_intervals[interval_index].agent_id != NO_AGENT)
    {
        cout << "ERROR: Agent " << agent_id << " interval already allocated to " << location_intervals[interval_index].agent_id << endl;
        exit(1);
    }

    // Merge with previous interval removing current interval
    // [1246,1250): 83, [1250,1251): -1 , [1251,1252): 12
    // [1246,1251): 83, [1251,1252): 12
    if (interval_index > 0 &&
        location_intervals[interval_index].low == low &&
        location_intervals[interval_index - 1].agent_id == agent_id &&
        location_intervals[interval_index].high - location_intervals[interval_index].low == 1)
    {
        location_intervals[interval_index - 1].high = high;
        location_intervals.erase(location_intervals.begin() + interval_index);

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Merge with previous interval truncating current interval.
    // [1246,1250): 83, [1250,1255): -1 , [1255,1252): 12
    // [1246,1251): 83, [1251,1255): -1 , [1255,1252): 12
    if (interval_index > 0 &&
        location_intervals[interval_index].low == low &&
        location_intervals[interval_index - 1].agent_id == agent_id)
    {
        location_intervals[interval_index - 1].high = high;
        location_intervals[interval_index].low = high;

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Proposed interval matches current interval
    if (location_intervals[interval_index].high == high &&
        location_intervals[interval_index].low == low)
    {
        location_intervals[interval_index].agent_id = agent_id;

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Interval shares a high interval
    if (location_intervals[interval_index].high == high)
    {
        location_intervals[interval_index].high = low;
        location_intervals.emplace(location_intervals.begin() + interval_index + 1, low, high, agent_id);

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Interval shares a low interval
    if (location_intervals[interval_index].low == low)
    {
        location_intervals[interval_index].low = high;
        location_intervals.emplace(location_intervals.begin() + interval_index, low, high, agent_id);

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // proposed interval is subset of found interval
    if (location_intervals[interval_index].low < low &&
        location_intervals[interval_index].high > high)
    {
        int new_high = location_intervals[interval_index].high;
        location_intervals[interval_index].high = low;
        location_intervals.emplace(location_intervals.begin() + interval_index + 1, high, new_high, NO_AGENT);
        location_intervals.emplace(location_intervals.begin() + interval_index + 1, low, high, agent_id);

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    cerr << "ERROR: split failed " << endl;
    cerr << "Proposed Interval: [" << low << "," << high << "):" << agent_id << endl;
    this->validate(location);
    exit(1);
}

void SIPPIntervals::merge(int agent_id, int location, int low, int high)
{
#ifdef DEBUG_MODE
    this->validate(location);
#endif

    auto &location_intervals = intervals_[location];

    assert(!location_intervals.empty());
    int index = this->binary_search(location, low);

    // Interval already removed by truncation
    if (location_intervals[index].agent_id != agent_id)
        return;

    // Two Neighbouring Safe Intervals
    if (index > 0 &&
        index < location_intervals.size() - 1 &&
        location_intervals[index - 1].agent_id == NO_AGENT &&
        location_intervals[index + 1].agent_id == NO_AGENT)
    {
        location_intervals[index - 1].high = location_intervals[index + 1].high;
        location_intervals.erase(location_intervals.begin() + index, location_intervals.begin() + index + 2);

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // One Neighbouring Safe Interval

    // Early
    if (index > 0 &&
        location_intervals[index - 1].agent_id == NO_AGENT)
    {
        location_intervals[index - 1].high = location_intervals[index].high;
        location_intervals.erase(location_intervals.begin() + index);

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Late
    if (index < location_intervals.size() - 1 &&
        location_intervals[index + 1].agent_id == NO_AGENT)
    {
        location_intervals[index + 1].low = location_intervals[index].low;
        location_intervals.erase(location_intervals.begin() + index);

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // No Neighbouring Safe Intervals
    location_intervals[index].agent_id = NO_AGENT;

#ifdef DEBUG_MODE
    this->validate(location);
#endif
    return;
}

void SIPPIntervals::validate(int location) const
{
    cout << "   location: " << location << endl
         << "   ";

    auto &location_intervals = intervals_[location];
    if (location_intervals[0].low != 0)
    {
        cerr << "ERROR: interval does not start at 0" << endl;
        exit(1);
    }

    for (int i = 0; i < location_intervals.size() - 1; i++)
        cout << "[" << location_intervals[i].low << "," << location_intervals[i].high << "): " << location_intervals[i].agent_id << " , ";
    cout << "[" << location_intervals.back().low << "," << location_intervals.back().high << "): " << location_intervals.back().agent_id << " " << endl;

    for (int i = 0; i < location_intervals.size() - 1; i++)
    {
        if (location_intervals[i].low >= location_intervals[i].high)
        {
            cerr << "ERROR: interval " << i << " has low >= high" << endl;
            exit(1);
        }
        if (location_intervals[i].agent_id == location_intervals[i + 1].agent_id)
        {
            cerr << "ERROR: interval " << i << " and " << i + 1 << " have the same agent_id" << endl;
            exit(1);
        }
        if (location_intervals[i].high > location_intervals[i + 1].low)
        {
            cerr << "ERROR: interval " << i << " and " << i + 1 << " overlap" << endl;
            exit(1);
        }
        if (location_intervals[i].high != location_intervals[i + 1].low)
        {
            cerr << "ERROR: interval " << i << " and " << i + 1 << " do not touch" << endl;
            exit(1);
        }
    }

    if (location_intervals.back().low >= location_intervals.back().high)
    {
        cerr << "ERROR: interval " << location_intervals.size() - 1 << " has low >= high" << endl;
        exit(1);
    }
    if (location_intervals.back().high != MAX_TIMESTEP)
    {
        cerr << "ERROR: interval does not end at MAX_TIMESTEP" << endl;
        exit(1);
    }
}
