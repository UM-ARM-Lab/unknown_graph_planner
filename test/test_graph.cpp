// Bring in my package's API, which is what I'm testing
#include "halton_graph.hpp"
// Bring in gtest
#include <gtest/gtest.h>

#define DIFF 0.0000001

// Declare a test
TEST(GraphTestSuite, construction)
{
    double max_dist = 0.4;
    auto g = HaltonGraph(1000, max_dist);
    for(auto &n: g.getNodes())
    {
        for(auto &e: n.getInEdges())
        {
            EXPECT_TRUE(e.getWeight() <= max_dist);
            EXPECT_TRUE(e.getWeight() > 0.00001);
            EXPECT_FALSE(e.getFromIndex() == e.getToIndex());
        }
    }

}

TEST(GraphTestSuite, graphComparison)
{
    HaltonGraph g1(1000, 0.4);
    HaltonGraph g2(g1);
    
    EXPECT_TRUE(arc_dijkstras::haveSameEdgeValidity(g1, g2));

    g2.getNode(100).getOutEdges()[0].setValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
    EXPECT_FALSE(arc_dijkstras::haveSameEdgeValidity(g1, g2));
    g1.getNode(100).getOutEdges()[0].setValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
    EXPECT_TRUE(arc_dijkstras::haveSameEdgeValidity(g1, g2));
}

TEST(GraphTestSuite, countEdges)
{
    auto g = HaltonGraph(10, 2);
    EXPECT_EQ(g.countEdges(), 10*9);
    g = HaltonGraph(10, 0);
    EXPECT_EQ(g.countEdges(), 0);
}

TEST(GraphTestSuite, saveAndLoad)
{
    auto g1 = HaltonGraph(1000, 0.4);
    std::string filepath = "/tmp/test_graph_storage.graph";
    g1.saveToFile(filepath);
    auto g2 = HaltonGraph(filepath);

    EXPECT_EQ(g1.getNodes().size(), g2.getNodes().size());

    EXPECT_EQ(g1.r_disc, g2.r_disc);

    for(int i=0; i<g1.getNodes().size(); i++)
    {
        auto &n1 = g1.getNodes()[i];
        auto &n2 = g2.getNodes()[i];

        // Check Node values (configuration) are equal
        auto &q1 = n1.getValue();
        auto &q2 = n2.getValue();
        EXPECT_EQ(q1.size(), q2.size());
        for(int j=0; j<q1.size(); j++)
        {
            EXPECT_EQ(q1[j], q2[j]);
        }


        //Check edges are equal
        auto &E1 = n1.getOutEdges();
        auto &E2 = n2.getOutEdges();

        EXPECT_EQ(E1.size(), E2.size());

        for(int j=0; j<E1.size(); j++)
        {
            EXPECT_TRUE(E1[j].getWeight() <= 0.4);
            EXPECT_TRUE(E1[j].getWeight() > 0.00001);

            EXPECT_EQ(E1[j].getValidity(), E2[j].getValidity());
            EXPECT_EQ(E1[j].getWeight(), E2[j].getWeight());
        }
    }
}




// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
