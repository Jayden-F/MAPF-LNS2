add_executable(sipp_intervals test_SIPPInterval.cpp)

# Link the test to any necessary libraries here if needed
add_library(SIPPIntervals ../src/SIPPInterval.cpp)
target_link_libraries(sipp_intervals SIPPIntervals)
add_test(NAME SIPPIntervals COMMAND sipp_intervals)