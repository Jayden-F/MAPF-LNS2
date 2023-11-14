#include "SIPPInterval.h"
#include "common.h"
#include <utility>

// #define DEBUG_MODE

const int SIPPIntervals::get_first_interval(int location, int start_time) {
    auto &location_intervals = intervals_[location];

    if (location_intervals.empty())
        this->init_location(location);

    int index = location_intervals.lower_bound(start_time).position;
    // int index(this->binary_search(location, start_time));

    if (location_intervals[index].agent_id != NO_AGENT)
        return -1;

    return index;
}

const vector<int> SIPPIntervals::get_intervals(int from, int interval,
                                               int timestep, int to) {

    if (intervals_[to].empty())
        this->init_location(to);

    auto low_iterator = this->intervals_[to].lower_bound(timestep);

    auto high_iterator =
        this->intervals_[to].lower_bound(intervals_[from][interval].high);

    this->clear_intervals_.clear();
    this->clear_intervals_.reserve(intervals_[to].size());

    for (auto current_iterator = low_iterator;
         current_iterator != high_iterator; current_iterator++) {

        SIPPInterval &current_interval = current_iterator->second;
        auto previous_iterator = current_iterator--;
        SIPPInterval &previous_interval = previous_iterator->second;
        auto next_iterator = current_iterator++;
        SIPPInterval &next_interval = next_iterator->second;

        // Vertex Conflict
        if (next_interval.agent_id != NO_AGENT) {
            continue;
        }
        // Edge Conflict
        if (i != intervals_[to].begin() && i < intervals_[from].size() &&
            intervals_[to][i - 1].agent_id != NO_AGENT &&
            intervals_[from][interval + 1].agent_id != NO_AGENT &&
            intervals_[to][i - 1].agent_id ==
                intervals_[from][interval + 1].agent_id &&
            intervals_[from][interval].high == next_interval.low) {
            continue;
        }
        clear_intervals_.push_back(i);
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
    auto current_iterator = location_intervals.lower_bound(timestep);
    SIPPInterval &current_interval = current_iterator->second;
    auto previous_iterator = current_iterator;
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

    auto current_iterator = location_intervals.lower_bound(timestep);
    SIPPInterval &current_interval = current_iterator->second;

    // location does not share timestep
    if (current_interval.low < timestep) {
        current_interval.high = timestep;
        location_intervals.insert(std::make_pair(
            timestep, SIPPInterval(timestep, MAX_TIMESTEP, agent_id)));

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    auto previous_iterator = current_iterator;
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
    auto current_iterator = location_intervals.lower_bound(timestep);
    SIPPInterval &current_interval = current_iterator->second;

    if (current_interval.agent_id != agent_id) {
        return;
    }

    int length = current_interval.high - current_interval.low;

    auto next_iterator = current_iterator;
    next_iterator++;
    SIPPInterval &next_interval = next_iterator->second;

    // Interval early interval off at timestep
    if (length > 1 && current_interval.low < timestep) {
        next_interval.low = timestep;
        current_interval.high = timestep;

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    auto previous_iterator = current_iterator;
    previous_iterator--;
    SIPPInterval &previous_interval = previous_iterator->second;

    // Remove interval entirely
    if (current_iterator != location_intervals.begin() &&
        previous_interval.agent_id == NO_AGENT &&
        next_interval.agent_id == NO_AGENT) {
        previous_interval.high = next_interval.high;
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

    auto current_iterator = location_intervals.lower_bound(low);
    SIPPInterval &current_interval = current_iterator->second;

    if (current_interval.agent_id != NO_AGENT) {
        cout << "ERROR: Agent " << agent_id << " interval already allocated to "
             << current_interval.agent_id << endl;
        exit(1);
    }

    // Merge with previous interval removing current interval
    // [1246,1250): 83, [1250,1251): -1 , [1251,1252): 12
    // [1246,1251): 83, [1251,1252): 12
    auto previous_iterator = current_iterator;
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
        current_interval.low = high;

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
            std::make_pair(low, SIPPInterval(low, high, agent_id)));

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // Interval shares a low interval
    if (current_interval.low == low) {
        current_interval.low = high;
        location_intervals.insert(
            std::make_pair(low, SIPPInterval(low, high, agent_id)));

#ifdef DEBUG_MODE
        this->validate(location);
#endif
        return;
    }

    // proposed interval is subset of found interval
    if (current_interval.low < low && current_interval.high > high) {
        int new_high = current_interval.high;
        current_interval.high = low;
        location_intervals.insert(
            std::make_pair(high, SIPPInterval(high, new_high, NO_AGENT)));
        location_intervals.insert(
            std::make_pair(high, SIPPInterval(low, high, agent_id)));

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

    auto current_iterator = location_intervals.lower_bound(low);
    SIPPInterval &current_interval = current_iterator->second;

    // Interval already removed by truncation
    if (current_interval.agent_id != agent_id)
        return;

    auto previous_iterator = current_iterator;
    previous_iterator--;
    SIPPInterval &previous_interval = previous_iterator->second;
    auto next_iterator = current_iterator;
    next_iterator++;
    SIPPInterval &next_interval = next_iterator->second;

    // Two Neighbouring Safe Intervals
    if (current_iterator != location_intervals.begin() &&
        current_iterator != location_intervals.end() &&
        previous_interval.agent_id == NO_AGENT &&
        next_interval.agent_id == NO_AGENT) {
        previous_interval.high = next_interval.high;
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
        next_interval.low = current_interval.low;
        location_intervals.erase(current_iterator);

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

// void SIPPIntervals::validate(int location) const {
//     cout << "   location: " << location << endl << "   ";

//     auto &location_intervals = intervals_[location];
//     if (location_intervals[0].low != 0) {
//         cerr << "ERROR: interval does not start at 0" << endl;
//         exit(1);
//     }

//     for (int i = 0; i < location_intervals.size() - 1; i++)
//         cout << "[" << location_intervals[i].low << ","
//              << location_intervals[i].high
//              << "): " << location_intervals[i].agent_id << " , ";
//     cout << "[" << location_intervals.back().low << ","
//          << location_intervals.back().high
//          << "): " << location_intervals.back().agent_id << " " << endl;

//     for (int i = 0; i < location_intervals.size() - 1; i++) {
//         if (location_intervals[i].low >= location_intervals[i].high) {
//             cerr << "ERROR: interval " << i << " has low >= high" << endl;
//             exit(1);
//         }
//         if (location_intervals[i].agent_id ==
//             location_intervals[i + 1].agent_id) {
//             cerr << "ERROR: interval " << i << " and " << i + 1
//                  << " have the same agent_id" << endl;
//             exit(1);
//         }
//         if (location_intervals[i].high > location_intervals[i + 1].low) {
//             cerr << "ERROR: interval " << i << " and " << i + 1 << " overlap"
//                  << endl;
//             exit(1);
//         }
//         if (location_intervals[i].high != location_intervals[i + 1].low) {
//             cerr << "ERROR: interval " << i << " and " << i + 1
//                  << " do not touch" << endl;
//             exit(1);
//         }
//     }

//     if (location_intervals.back().low >= location_intervals.back().high) {
//         cerr << "ERROR: interval " << location_intervals.size() - 1
//              << " has low >= high" << endl;
//         exit(1);
//     }
//     if (location_intervals.back().high != MAX_TIMESTEP) {
//         cerr << "ERROR: interval does not end at MAX_TIMESTEP" << endl;
//         exit(1);
//     }
// }
