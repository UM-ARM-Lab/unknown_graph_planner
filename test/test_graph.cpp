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

TEST(GraphTestSuite, saveAndLoad)
{
    auto g = CSpaceHaltonGraph(20, 0.1);
    std::string filepath = "/tmp/test_graph_storage.graph";
    g.saveToFile(filepath);
    auto g2 = CSpaceHaltonGraph(0, 0);
    g2.loadFromFile(filepath);
    EXPECT_EQ(g.GetNodesImmutable().size(), 20);
    EXPECT_EQ(g.GetNodesImmutable().size(), g2.GetNodesImmutable().size());

    EXPECT_EQ(g.r_disc, g2.r_disc);
}




// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
