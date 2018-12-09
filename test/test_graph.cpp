// Bring in my package's API, which is what I'm testing
#include "cspace_halton_graph.hpp"
// Bring in gtest
#include <gtest/gtest.h>

#define DIFF 0.0000001

// Declare a test
TEST(GraphTestSuite, creation)
{
    CSpaceHaltonGraph(20, 0.1);
}




// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
