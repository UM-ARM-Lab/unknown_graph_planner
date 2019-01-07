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
    for(auto &n: g.GetNodesMutable())
    {
        for(auto &e: n.GetInEdgesMutable())
        {
            EXPECT_TRUE(e.GetWeight() <= max_dist);
            EXPECT_TRUE(e.GetWeight() > 0.00001);
            EXPECT_FALSE(e.GetFromIndex() == e.GetToIndex());
        }
    }

}

TEST(GraphTestSuite, graphComparison)
{
    HaltonGraph g1(1000, 0.4);
    HaltonGraph g2(g1);
    
    EXPECT_TRUE(arc_dijkstras::haveSameEdgeValidity(g1, g2));

    g2.GetNodeMutable(100).GetOutEdgesMutable()[0].SetValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
    EXPECT_FALSE(arc_dijkstras::haveSameEdgeValidity(g1, g2));
    g1.GetNodeMutable(100).GetOutEdgesMutable()[0].SetValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
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

    EXPECT_EQ(g1.GetNodesImmutable().size(), g2.GetNodesImmutable().size());

    EXPECT_EQ(g1.r_disc, g2.r_disc);

    for(int i=0; i<g1.GetNodesMutable().size(); i++)
    {
        auto &n1 = g1.GetNodesMutable()[i];
        auto &n2 = g2.GetNodesMutable()[i];

        // Check Node values (configuration) are equal
        auto &q1 = n1.GetValueMutable();
        auto &q2 = n2.GetValueMutable();
        EXPECT_EQ(q1.size(), q2.size());
        for(int j=0; j<q1.size(); j++)
        {
            EXPECT_EQ(q1[j], q2[j]);
        }


        //Check edges are equal
        auto &E1 = n1.GetOutEdgesMutable();
        auto &E2 = n2.GetOutEdgesMutable();

        EXPECT_EQ(E1.size(), E2.size());

        for(int j=0; j<E1.size(); j++)
        {
            EXPECT_TRUE(E1[j].GetWeight() <= 0.4);
            EXPECT_TRUE(E1[j].GetWeight() > 0.00001);

            EXPECT_EQ(E1[j].GetValidity(), E2[j].GetValidity());
            EXPECT_EQ(E1[j].GetWeight(), E2[j].GetWeight());
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
