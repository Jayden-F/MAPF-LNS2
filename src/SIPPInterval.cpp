#include "SIPPInterval.h"
#include "common.h"
#include <iterator>
#include <utility>

// #define DEBUG_MODE

bool SIPPIntervals::get_first_interval(int location, int start_time,
                                       iterator &return_interval) {
    auto &location_intervals = intervals_[location];

    if (location_intervals.empty())
        this->init_location(location);

    iterator interval_iterator = location_intervals.upper_bound(start_time);
    interval_iterator--;
    // int index(this->binary_search(location, start_time));

    if (interval_iterator->second.agent_id != NO_AGENT)
        return false;

    return_interval = interval_iterator;
    return true;
}

const vector<iterator>
SIPPIntervals::get_intervals(int from, iterator from_current_iterator,
                             int timestep, int to) {

    if (intervals_[to].empty())
        this->init_location(to);

    // cout << from_current_iterator->first << " "
    //      << from_current_iterator->second.low << " "
    //      << from_current_iterator->second.high << " "
    //      << from_current_iterator->second.agent_id << endl;
    // for (auto i = intervals_[to].begin(); i != intervals_[to].end(); i++) {
    //     cout << i->first << " " << i->second.low << " " << i->second.high <<
    //     " "
    //          << i->second.agent_id << endl;
    // }

    iterator low_iterator = this->intervals_[to].upper_bound(timestep);
    low_iterator--;
    iterator high_iterator =
        this->intervals_[to].upper_bound(from_current_iterator->second.high);

    this->clear_intervals_.clear();
    this->clear_intervals_.reserve(intervals_[to].size());

    iterator from_previous_iterator = from_current_iterator;
    from_previous_iterator--;
    iterator from_next_iterator = from_current_iterator;
    from_next_iterator++;

    for (iterator to_current_iterator = low_iterator;
         to_current_iterator != high_iterator; to_current_iterator++) {

        iterator to_previous_iterator = to_current_iterator;
        to_previous_iterator--;
        iterator to_next_iterator = to_current_iterator;
        to_next_iterator++;
        // Vertex Conflict
        if (to_current_iterator->second.agent_id != NO_AGENT) {
            continue;
        }
        // Edge Conflict
        if (to_current_iterator != intervals_[to].begin() &&
            from_next_iterator != intervals_[from].end() &&
            to_previous_iterator->second.agent_id != NO_AGENT &&
            from_next_iterator->second.agent_id != NO_AGENT &&
            to_previous_iterator->second.agent_id ==
                from_next_iterator->second.agent_id &&
            from_current_iterator->second.high ==
                to_current_iterator->second.low) {
            continue;
        }
        clear_intervals_.push_back(to_current_iterator);
    }
    return clear_intervals_;
}

void SIPPIntervals::insert_path(int agent_id, Path &path, int start,
                                int horizon) {
    if (path.empty())
        return;

    // int location = path[0].location;
    int low(start);
    // int high(start);

    int t_max = min((int)path.size(), horizon + 1);
    for (int t = 0; t < t_max; t++) {
        if (t + 1 >= t_max || path[t].location != path[t + 1].location) {
#ifdef DEBUG_MODE
            cout << agent_id << " splitting: " << path[t].location << " @ ["
                 << low << "," << start + t + 1 << ")" << endl;
#endif
            this->split(agent_id, path[t].location, low, start + t + 1);
            low = start + t + 1;
        }
    }
}

void SIPPIntervals::remove_path(int agent_id, Path &path, int start, int period,
                                int horizon) {
    if (period >= path.size())
        return;
    // truncate first location as might be shared with previous window.
    this->truncate(agent_id, path[period].location, start + period);

    // int location = path[period + 1].location;
    int low(start + period + 1); // Low timestep of interval
    // int high(start + period + 1); // High timestep of interval

    int t_max = min((int)path.size(), horizon + 1);
    for (int t = period + 1; t < t_max; t++) {
        if (t + 1 >= t_max || path[t].location != path[t + 1].location) {
#ifdef DEBUG_MODE
            cout << agent_id << " merging: " << path[t].location << " @ ["
                 << low << "," << start + t + 1 << ")" << endl;
#endif
            this->merge(agent_id, path[t].location, low, start + t + 1);
            low = start + t + 1;
        }
    }
}

void SIPPIntervals::unreserve_goal(int agent_id, int location, int timestep) {

    auto &location_intervals = intervals_[location];

#ifdef DEBUG_MODE
    this->validate(location);
#endif

    if (location_intervals.empty()) {
        return;
    }
    iterator current_iterator = location_intervals.end();
    current_iterator--;
    SIPPInterval &current_interval = current_iterator->second;
    iterator previous_iterator = current_iterator;
    previous_iterator--;
    SIPPInterval &previous_interval = previous_iterator->second;

    if (current_iterator != location_intervals.begin() &&
        previous_interval.agent_id == NO_AGENT) {
        previous_interval.high = MAX_TIMESTEP;
        location_intervals.erase(current_iterator);
#ifdef DEBUG_MODE
        cout << "agent: " << agent_id
             << " removing reservation on goal location: " << location << endl;
        this->validate(location);
#endif
        return;
    }

    current_interval.agent_id = NO_AGENT;

#ifdef DEBUG_MODE
    cout << "agent: " << agent_id
         << " removing reservation on goal location: " << location << endl;
    this->validate(location);
#endif
}

void SIPPIntervals::reserve_goal(int agent_id, int location, int timestep) {

#ifdef DEBUG_MODE
    cout << "agent: " << agent_id
         << " reservation on goal location: " << location
         << " at timestep:" << timestep << endl;
    this->validate(location);
#endif
    auto &location_intervals = intervals_[location];

    if (location_intervals.empty()) {
        this->init_location(location);
    }

    if (!this->is_location_clear(location, timestep)) {
        // this->validate(location);
        cout << "location cannot be reservered at timestep: " << timestep
             << " by agent: " << agent_id << endl;
        exit(-1);
    }

    iterator current_iterator = location_intervals.end();
    current_iterator--;
    SIPPInterval &current_interval = current_iterator->second;

    // location does not share timestep
    if (current_interval.low < timestep) {
        current_interval.high = timestep;
        location_intervals.insert(std::make_pair(
            timestep,
            std::move(SIPPInterval(timestep, MAX_TIMESTEP, agent_id))));

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    iterator previous_iterator = current_iterator;
    previous_iterator--;
    SIPPInterval &previous_interval = previous_iterator->second;

    // location is reserved by current agent extending reservation
    if (current_iterator != location_intervals.begin() &&
        previous_interval.agent_id == agent_id &&
        current_interval.low == timestep) {
        previous_interval.high = MAX_TIMESTEP;
        location_intervals.erase(current_iterator);
#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // location is
    if (current_interval.low == timestep) {
        current_interval.agent_id = agent_id;

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    cout << "Error reserving goal" << endl;
    exit(-1);
}

void SIPPIntervals::truncate(int agent_id, int location, int timestep) {
#ifdef DEBUG_MODE
    cout << agent_id << " truncating: " << location << " @ [" << timestep << ","
         << timestep + 1 << ")" << endl;
    this->validate(location);
#endif

    auto &location_intervals = intervals_[location];
    iterator current_iterator = location_intervals.upper_bound(timestep);
    current_iterator--;
    SIPPInterval &current_interval = current_iterator->second;

    if (current_interval.agent_id != agent_id) {
        return;
    }

    int length = current_interval.high - current_interval.low;

    iterator next_iterator = current_iterator;
    next_iterator++;
    SIPPInterval &next_interval = next_iterator->second;

    // Interval early interval off at timestep
    if (length > 1 && current_interval.low < timestep &&
        next_interval.agent_id == NO_AGENT) {
        int next_high = next_interval.high;
        current_interval.high = timestep;
        location_intervals.erase(next_iterator);
        location_intervals.insert(std::make_pair(
            timestep, std::move(SIPPInterval(timestep, next_high, NO_AGENT))));

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Interval early interval off at timestep
    if (length > 1 && current_interval.low < timestep &&
        next_interval.agent_id != NO_AGENT) {
        current_interval.high = timestep;
        location_intervals.insert(std::make_pair(
            timestep,
            std::move(SIPPInterval(timestep, next_interval.low, NO_AGENT))));

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    iterator previous_iterator = current_iterator;
    previous_iterator--;
    SIPPInterval &previous_interval = previous_iterator->second;

    // Remove interval entirely
    if (current_iterator != location_intervals.begin() &&
        previous_interval.agent_id == NO_AGENT &&
        next_interval.agent_id == NO_AGENT) {
        previous_interval.high = next_interval.high;
        next_iterator++;
        location_intervals.erase(current_iterator, next_iterator);

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Merge with previous interval if it is safe
    if (current_iterator != location_intervals.begin() &&
        previous_interval.agent_id == NO_AGENT) {
        previous_interval.high = current_interval.high;
        location_intervals.erase(current_iterator);

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Merge with next interval if it is safe
    if (next_interval.agent_id == NO_AGENT) {
        current_interval.agent_id = NO_AGENT;
        current_interval.high = next_interval.high;
        location_intervals.erase(next_iterator);

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Allocate interval to No Agent
    current_interval.agent_id = NO_AGENT;
#ifdef DEBUG_MODE
    this->validate(location);
#endif
    return;
}

void SIPPIntervals::split(int agent_id, int location, int low, int high) {

    auto &location_intervals = intervals_[location];

    if (location_intervals.empty()) {
        this->init_location(location);
    }

#ifdef DEBUG_MODE
    this->validate(location);
#endif

    iterator current_iterator = location_intervals.upper_bound(low);
    current_iterator--;
    SIPPInterval &current_interval = current_iterator->second;

    if (current_interval.agent_id != NO_AGENT) {
        cout << "ERROR: Agent " << agent_id << " interval already allocated to "
             << current_interval.agent_id << endl;
        exit(1);
    }

    // Merge with previous interval removing current interval
    // [1246,1250): 83, [1250,1251): -1 , [1251,1252): 12
    // [1246,1251): 83, [1251,1252): 12
    iterator previous_iterator = current_iterator;
    previous_iterator--;
    SIPPInterval &previous_interval = previous_iterator->second;

    if (current_iterator != location_intervals.begin() &&
        current_interval.low == low && previous_interval.agent_id == agent_id &&
        current_interval.high - current_interval.low == 1) {
        previous_interval.high = high;
        location_intervals.erase(current_iterator);

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Merge with previous interval truncating current interval.
    // [1246,1250): 83, [1250,1255): -1 , [1255,1252): 12
    // [1246,1251): 83, [1251,1255): -1 , [1255,1252): 12
    if (current_iterator != location_intervals.begin() &&
        current_interval.low == low && previous_interval.agent_id == agent_id) {
        previous_interval.high = high;
        SIPPInterval new_interval(high, current_interval.high, NO_AGENT);
        location_intervals.erase(current_iterator);
        location_intervals.insert(
            std::make_pair(high, std::move(new_interval)));

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Proposed interval matches current interval
    if (current_interval.high == high && current_interval.low == low) {
        current_interval.agent_id = agent_id;

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Interval shares a high interval
    if (current_interval.high == high) {
        current_interval.high = low;
        location_intervals.insert(
            std::make_pair(low, std::move(SIPPInterval(low, high, agent_id))));

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Interval shares a low interval
    if (current_interval.low == low) {
        int new_high = current_interval.high;
        current_interval.high = high;
        current_interval.agent_id = agent_id;
        location_intervals.insert(std::make_pair(
            high, std::move(SIPPInterval(high, new_high, NO_AGENT))));

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // proposed interval is subset of found interval
    if (current_interval.low < low && current_interval.high > high) {
        int new_high = current_interval.high;
        current_interval.high = low;
        location_intervals.insert(std::make_pair(
            high, std::move(SIPPInterval(high, new_high, NO_AGENT))));
        location_intervals.insert(
            std::make_pair(low, std::move(SIPPInterval(low, high, agent_id))));

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    cerr << "ERROR: split failed " << endl;
    cerr << "Proposed Interval: [" << low << "," << high << "):" << agent_id
         << endl;
    // this->validate(location);
    exit(1);
}

void SIPPIntervals::merge(int agent_id, int location, int low, int high) {
#ifdef DEBUG_MODE
    this->validate(location);
#endif

    auto &location_intervals = intervals_[location];

    assert(!location_intervals.empty());

    iterator current_iterator = location_intervals.upper_bound(low);
    current_iterator--;
    SIPPInterval &current_interval = current_iterator->second;

    // Interval already removed by truncation
    if (current_interval.agent_id != agent_id)
        return;

    iterator previous_iterator = current_iterator;
    previous_iterator--;
    SIPPInterval &previous_interval = previous_iterator->second;
    iterator next_iterator = current_iterator;
    next_iterator++;
    SIPPInterval &next_interval = next_iterator->second;

    // Two Neighbouring Safe Intervals
    if (current_iterator != location_intervals.begin() &&
        current_iterator != location_intervals.end() &&
        previous_interval.agent_id == NO_AGENT &&
        next_interval.agent_id == NO_AGENT) {
        previous_interval.high = next_interval.high;
        next_iterator++;
        location_intervals.erase(current_iterator, next_iterator);

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // One Neighbouring Safe Interval

    // Early
    if (current_iterator != location_intervals.begin() &&
        previous_interval.agent_id == NO_AGENT) {
        previous_interval.high = current_interval.high;
        location_intervals.erase(current_iterator);

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Late
    if (current_iterator != location_intervals.end() &&
        next_interval.agent_id == NO_AGENT) {
        // next_interval.low = current_interval.low;
        current_interval.high = next_interval.high;
        current_interval.agent_id = NO_AGENT;
        location_intervals.erase(next_iterator);

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // No Neighbouring Safe Intervals
    current_interval.agent_id = NO_AGENT;

#ifdef DEBUG_MODE
    this->validate(location);
#endif
    return;
}

void SIPPIntervals::validate(int location) const {

    auto &location_intervals = intervals_[location];

    if (location_intervals.empty()) {
        return;
    }

    if (location_intervals.begin()->second.low != 0) {
        cerr << "ERROR: interval does not start at 0" << endl;
        exit(1);
    }

    cout << "location: " << location << endl;
    for (auto i = location_intervals.begin(); i != location_intervals.end();
         i++) {
        cout << "[" << i->second.low << "," << i->second.high
             << "):" << i->second.agent_id << ", ";
    }
    cout << endl;

    for (auto i = location_intervals.begin();; i++) {
        auto next_iterator = i;
        next_iterator++;

        if (next_iterator == location_intervals.end()) {
            if (i->second.high != MAX_TIMESTEP) {
                cerr << "ERROR: interval does not end at MAX_TIMESTEP" << endl;
                exit(1);
            }
            break;
        }

        if (i->second.low >= i->second.high) {
            cerr << "ERROR: interval " << i.position << " has low >= high"
                 << endl;
            exit(1);
        }
        if (i->second.agent_id == next_iterator->second.agent_id) {
            cerr << "ERROR: interval " << i.position << " and "
                 << i.position + 1 << " have the same agent_id" << endl;
            exit(1);
        }
        if (i->second.high > next_iterator->second.low) {
            cerr << "ERROR: interval " << i.position << " and "
                 << i.position + 1 << " overlap" << endl;
            exit(1);
        }
        if (i->second.high != next_iterator->second.low) {
            cerr << "ERROR: interval " << i.position << " and "
                 << i.position + 1 << " do not touch" << endl;
            exit(1);
        }
    }

    // if (location_intervals.end()->second.low >=
    //     location_intervals.end()->second.high) {
    //     cerr << "ERROR: interval " << location_intervals.size() - 1
    //          << " has low >= high" << endl;
    //     exit(1);
    // }
    // if ((location_intervals.end()--)->second.high != MAX_TIMESTEP) {
    //     cerr << "ERROR: interval does not end at MAX_TIMESTEP" << endl;
    //     exit(1);
    // }
}
