#include "halton_graph.hpp"
#include "increasing_density_search.hpp"
#include "increasing_density_planning.hpp"
#include <gtest/gtest.h>


TEST(Grid, Create)
{
    for(int i=0; i<4; i++)
    {
        IncreasingDensityGrid g(i);
        int expected_nodes_per_side = 1 + pow(2, i);
        EXPECT_EQ(g.getNodes().size(), std::pow(expected_nodes_per_side, 2)) << "Wrong number of nodes";
    }


    for(int i=0; i<4; i++)
    {
        IncreasingDensityGrid g(i);
        int expected_edges = 2*(i+1);
        EXPECT_EQ(g.getNode(0).getOutEdges().size(), expected_edges) << "Wrong number of edges";
        EXPECT_EQ(g.getNode(0).getInEdges().size(), expected_edges) << "Wrong number of edges";
    }
}




int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}

