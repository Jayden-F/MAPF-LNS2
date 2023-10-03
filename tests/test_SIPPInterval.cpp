#include <cassert>
#include <SIPPInterval.h>

class TestSIPPInterval
{
public:
    SIPPIntervals sipp_intervals;

    TestSIPPInterval() : sipp_intervals(10) {}
    // [0, 10) -> [0, 10) [10, MAX_TIMESTEP)
    void test_split_early()
    {
        sipp_intervals.clear();
        int agent_id = 0;
        int location = 1;
        int low = 0;
        int high = 10;

        sipp_intervals.split(agent_id, location, low, high);
        sipp_intervals.validate(location);

        const SIPPInterval *first_interval = sipp_intervals.get_interval(location, 0);
        assert(first_interval->low == low);
        assert(first_interval->high == high);
        assert(first_interval->agent_id == agent_id);

        const SIPPInterval *second_interval = sipp_intervals.get_interval(location, 1);
        assert(second_interval->low == high);
        assert(second_interval->high == MAX_TIMESTEP);
        assert(second_interval->agent_id == NO_AGENT);
    }

    // [5, 10) -> [5, 10) [10, MAX_TIMESTEP)
    void test_split_mid()
    {
        sipp_intervals.clear();
        int agent_id = 0;
        int location = 1;
        int low = 5;
        int high = 10;

        sipp_intervals.split(agent_id, location, low, high);

        sipp_intervals.validate(location);

        const SIPPInterval *first_interval = sipp_intervals.get_interval(location, 0);
        assert(first_interval->low == 0);
        assert(first_interval->high == low);
        assert(first_interval->agent_id == NO_AGENT);

        const SIPPInterval *second_interval = sipp_intervals.get_interval(location, 1);
        assert(second_interval->low == low);
        assert(second_interval->high == high);
        assert(second_interval->agent_id == agent_id);

        const SIPPInterval *third_interval = sipp_intervals.get_interval(location, 2);
        assert(third_interval->low == high);
        assert(third_interval->high == MAX_TIMESTEP);
        assert(third_interval->agent_id == NO_AGENT);
    }

    void test_split_after_unsafe()
    {
        int first_agent_id = 0;
        int first_location = 1;
        int first_low = 0;
        int first_high = 10;

        sipp_intervals.split(first_agent_id, first_location, first_low, first_high);
        sipp_intervals.validate(first_location);

        int second_agent_id = 1;
        int second_location = 1;
        int second_low = 10;
        int second_high = 20;

        sipp_intervals.split(second_agent_id, second_location, second_low, second_high);
        sipp_intervals.validate(second_location);

        const SIPPInterval *first_interval = sipp_intervals.get_interval(first_location, 0);
        assert(first_interval->low == first_low);
        assert(first_interval->high == first_high);
        assert(first_interval->agent_id == first_agent_id);

        const SIPPInterval *second_interval = sipp_intervals.get_interval(first_location, 1);
        assert(second_interval->low == first_high);
        assert(second_interval->low == second_low);
        assert(second_interval->high == second_high);
        assert(second_interval->agent_id == second_agent_id);

        const SIPPInterval *third_interval = sipp_intervals.get_interval(first_location, 2);
        assert(third_interval->low == second_high);
        assert(third_interval->high == MAX_TIMESTEP);
        assert(third_interval->agent_id == NO_AGENT);
    }
};

int main()
{
    TestSIPPInterval test;
    test.test_split_early();
    test.test_split_mid();

    return 0;
}