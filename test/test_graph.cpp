// Bring in my package's API, which is what I'm testing
#include "halton_graph.hpp"
// Bring in gtest
#include <gtest/gtest.h>

#define DIFF 0.0000001


TEST(GraphTestSuite, rdisc_construction)
{
    RDiscGraph g(0.1);
    g.addVertexAndEdges(std::vector<double>{0,1});
}

TEST(GraphTestSuite, halton_construction)
{
    double max_dist = 0.4;
    auto g = HaltonGraph(1000, max_dist);
    for(auto &n: g.getNodes())
    {
        for(auto &e: n.getInEdges())
        {
            EXPECT_LE(e.getWeight(), max_dist);
            EXPECT_GT(e.getWeight(), 0.00001);
            EXPECT_FALSE(e.getFromIndex() == e.getToIndex());
        }
    }

}

TEST(GraphTestSuite, halton_graph_comparison)
{
    HaltonGraph g1(1000, 0.4);
    HaltonGraph g2(g1);
    
    EXPECT_TRUE(arc_dijkstras::haveSameEdgeValidity(g1, g2));

    g2.getNode(100).getOutEdges()[0].setValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
    EXPECT_FALSE(arc_dijkstras::haveSameEdgeValidity(g1, g2));
    g1.getNode(100).getOutEdges()[0].setValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
    EXPECT_TRUE(arc_dijkstras::haveSameEdgeValidity(g1, g2));
}


TEST(GraphTestSuite, rdisc_neighbors)
{
    RDiscGraph g(0.5);
    g.addVertexAndEdges(std::vector<double>{0, 0, 0});
    g.addVertexAndEdges(std::vector<double>{0, 0, 0.4});
    g.addVertexAndEdges(std::vector<double>{0.1, 0.1, 0.1});
    g.addVertexAndEdges(std::vector<double>{1, 1, 1});
    EXPECT_EQ(4, g.getNodes().size()) << "Graph has wrong number of nodes";
    EXPECT_EQ(6, g.countEdges()) << "Graph has wrong number of edges";

    auto within_disc = g.getVerticesWithinRadius(std::vector<double>{0.05, 0.05, 0.05}, 0.18);
    EXPECT_EQ(within_disc.size(), 2);
    for(int i:within_disc)
    {
        EXPECT_TRUE(i==0 || i==2);
    }



    //NOTE FLANNS KD tree is not exact
    // HaltonGraph g2(1000, 0.4);
    // std::vector<double> query{0.5, 0.5};
    // double radius = 0.1;

    // ASSERT_EQ(g2.index.size(), 1000);

    // std::vector<int> inds_in_radius = g2.getVerticesWithinRadius(query, radius);
    // for(int ind:inds_in_radius)
    // {
    //     double d = EigenHelpers::Distance(g2.getNode(ind).getValue(), query);
    //     ASSERT_LE(d, radius); 
    // }

    // for(int i=0; i<g2.index.size(); i++)
    // {
    //     if(std::count(inds_in_radius.begin(), inds_in_radius.end(), i))
    //     {
    //         continue;
    //     }
    //     double d = EigenHelpers::Distance(g2.getNode(i).getValue(), query);
    //     EXPECT_GE(d, radius); 
    // }
}

TEST(GraphTestSuite, nearest_neighbors)
{
    RDiscGraph g(0.5);
    g.addVertexAndEdges(std::vector<double>{0, 0, 0});
    g.addVertexAndEdges(std::vector<double>{0, 0, 0.4});
    g.addVertexAndEdges(std::vector<double>{0.1, 0.1, 0.1});
    g.addVertexAndEdges(std::vector<double>{1, 1, 1});

    EXPECT_EQ(g.getNearest(std::vector<double>{0.1, 0.1, 0.1}), 2);

    EXPECT_EQ(g.getNearest(std::vector<double>{0.9, 0.9, 0.4}), 3);
}

TEST(GraphTestSuite, copy_constructor)
{
    std::vector<double> query{0.5, 0.5};
    double radius = 0.2;

    HaltonGraph g1(1000, 0.4);
    auto rdisc_orig_first = g1.getVerticesWithinRadius(query, radius);
    // std::cout << rdisc_orig_first.size() << "\n";
    
    HaltonGraph g2(g1);
    // HaltonGraph g3;
    // g3 = g1;

    ASSERT_EQ(g1.getNodes().size(), g2.getNodes().size());
    // ASSERT_EQ(g1.getNodes().size(), g3.getNodes().size());

    for(int64_t i=0; i<g1.getNodes().size(); i++)
    {
        ASSERT_EQ(g1.getNode(i).getOutEdges().size(), g2.getNode(i).getOutEdges().size());
        // ASSERT_EQ(g1.getNode(i).getOutEdges().size(), g3.getNode(i).getOutEdges().size());

        for(int64_t j=0; j<g1.getNode(i).getValue().size(); j++)
        {
            EXPECT_EQ(g1.getNode(i).getValue()[j], g2.getNode(i).getValue()[j]) <<
                "Copy constructor changed node value";
            // EXPECT_EQ(g1.getNode(i).getValue()[j], g3.getNode(i).getValue()[j]) <<
            //     "Assignment changed node value";
        }
    }

    auto rdisc_orig = g1.getVerticesWithinRadius(query, radius);
    // std::cout << "rdisc_orig: " << rdisc_orig.size() << "\n";

    for(auto &n:g1.getNodes())
    {
        n.getValue() = std::vector<double>{-1,-1};
    }
    ASSERT_NE(rdisc_orig.size(), g1.getVerticesWithinRadius(query, radius).size()) <<
        "Test invalid. g1 not properly mangled for test to be valid";
    for(int64_t i=0; i<g1.getNodes().size(); i++)
    {
        ASSERT_EQ(g1.getNode(i).getOutEdges().size(), g2.getNode(i).getOutEdges().size());
        // ASSERT_EQ(g1.getNode(i).getOutEdges().size(), g3.getNode(i).getOutEdges().size());

        for(int64_t j=0; j<g1.getNode(i).getValue().size(); j++)
        {
            ASSERT_NE(g1.getNode(i).getValue()[j], g2.getNode(i).getValue()[j]) <<
                "Mangling of original changed copy";
            // EXPECT_EQ(g1.getNode(i).getValue()[j], g3.getNode(i).getValue()[j]) <<
            //     "Assignment changed node value";
        }
    }



    ASSERT_EQ(g1.index.size(), 1000);
    ASSERT_EQ(g2.index.size(), 1000);
    
    // std::cout << "rdisc_orig: " << rdisc_orig.size() << "\n";
    
    auto rdisc_copy = g2.getVerticesWithinRadius(query, radius);
    // auto rdisc_asgn = g3.getVerticesWithinRadius(query, radius);
    // std::cout << "rdisc_copy: " << rdisc_copy.size() << "\n";

    ASSERT_GT(rdisc_orig.size(), 0) << "Test not valid, no elements in rdisc";
    ASSERT_GT(rdisc_copy.size(), rdisc_orig.size() - 10) << "Copy changes rdisc";
    ASSERT_LT(rdisc_copy.size(), rdisc_orig.size() + 10) << "Copy changes rdisc";
    // ASSERT_EQ(rdisc_orig.size(), rdisc_asgn.size()) << "Assignment changes rdisc";
    
}

TEST(GraphTestSuite, halton_count_edges)
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
            EXPECT_LE(E1[j].getWeight(), 0.4);
            EXPECT_GT(E1[j].getWeight(), 0.00001);

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
