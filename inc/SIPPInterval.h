#pragma once
#include "btree/btree_container.h"
#include "btree/btree_map.h"
#include "common.h"
#include <cassert>
#include <utility>

#define NO_AGENT -1

struct SIPPInterval {
  int low;
  int high;
  int agent_id;

  SIPPInterval() : low(0), high(MAX_TIMESTEP), agent_id(-1) {}
  SIPPInterval(int low, int high) : low(low), high(high), agent_id(-1) {}
  SIPPInterval(int low, int high, int agent_id)
      : low(low), high(high), agent_id(agent_id) {}
};

typedef btree::btree_iterator<
    btree::btree_node<btree::btree_map_params<
        int, SIPPInterval, std::less<int>,
        std::allocator<std::pair<const int, SIPPInterval>>, 256>>,
    pair<const int, SIPPInterval> &, pair<const int, SIPPInterval> *>
    iterator;

class SIPPIntervals {

private:
  vector<btree::btree_map<int, SIPPInterval>> intervals_;
  vector<iterator> clear_intervals_;

public:
  // in the constructor initalise intervals_ with map_size
  SIPPIntervals(int map_size) : intervals_(map_size), clear_intervals_(0) {}
  bool get_first_interval(int location, int start_time,
                          iterator &return_interval);

  bool is_location_clear(int location, int timestep) {
    auto &location_intervals = intervals_[location];

    if (location_intervals.empty()) {
      init_location(location);
    }
    return (location_intervals.rbegin()->second.low <= timestep &&
            location_intervals.rbegin()->second.agent_id == NO_AGENT);
  }
  const vector<iterator> get_intervals(int from, iterator interval,
                                       int timestep, int to);
  // inline const SIPPInterval *get_interval(int location, int index) {
  //     return &intervals_[location][index];
  // }
  void insert_path(int agent_id, Path &path, int start = 0,
                   int horizon = MAX_TIMESTEP);
  void remove_path(int agent_id, Path &path, int start = 0, int period = 0,
                   int horizon = MAX_TIMESTEP);
  void unreserve_goal(int agent_id, int location, int timestep);
  void reserve_goal(int agent_id, int location, int timestep);
  // void cleared_intervals(int timestep) const {
  //     for (int i = 0; i < intervals_.size(); i++)
  //         for (int j = 0; j < intervals_[i].size(); j++)
  //             assert(intervals_[i][j].high <= timestep ||
  //                    intervals_[i][j].agent_id == NO_AGENT);
  // }
  void agent_removed(int agent_id) {
    for (auto location : intervals_) {
      for (auto it = location.begin(); it != location.end(); it++) {
        if (it->second.agent_id == agent_id) {
          cout << "agent " << agent_id << " is not removed from the location "
               << location << " [" << it->second.low << "," << it->second.high
               << endl;
          exit(1);
        }
      }
    }
  }
  void clear() {
    for (int i = 0; i < intervals_.size(); i++)
      intervals_[i].clear();
  }
  void validate(int location) const;

private:
  void init_location(int location) {

    assert(intervals_[location].empty());
#ifdef DEBUG_MODE
    this->validate(location);
#endif
    intervals_[location].insert(
        std::make_pair(0, std::move(SIPPInterval(0, MAX_TIMESTEP))));
#ifdef DEBUG_MODE
    this->validate(location);
#endif
  }
  void split(int agent_id, int location, int low, int high);
  void merge(int agent_id, int location, int low, int high);
  void truncate(int agent_id, int location, int timestep);

  //   inline int binary_search(int location, int timestep) const {
  //     int left(0);
  //     int right(intervals_[location].size() - 1);
  //     int mid;

  //     while (left <= right) {
  //       mid = (left + right) / 2;
  //       if (intervals_[location][mid].high <= timestep)
  //         left = mid + 1;
  //       else if (intervals_[location][mid].low > timestep)
  //         right = mid - 1;
  //       else
  //         return mid;
  //     }

  //     this->validate(location);
  //     cerr << "ERROR: binary_search failed to find interval " << timestep
  //     << endl;

  //     return -1;
  //   }
};