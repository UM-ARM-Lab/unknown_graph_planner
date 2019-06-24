#include "halton_graph.hpp"
#include "increasing_density_grid.hpp"
#include "increasing_density_planning.hpp"
#include <gtest/gtest.h>


TEST(Grid, Create)
{
    int num_nodes_expected = 0;
    for(int i=0; i<4; i++)
    {
        DoublingSDG g(i);
        int expected_nodes_per_side = 1 + pow(2, i);
        num_nodes_expected += std::pow(expected_nodes_per_side, 2);
        EXPECT_EQ(g.getNodes().size(), num_nodes_expected) << "Wrong number of nodes";
    }


    DoublingSDG g(4);
    for(int depth =0; depth<4; depth++)
    {
        int expected_edges = 3 + (depth != 0);
        const auto n = g.getNode(g.getNodeAt(depth, std::vector<double>{0,0}));
        EXPECT_EQ(n.getOutEdges().size(), expected_edges) << "Wrong number of edges";
        EXPECT_EQ(n.getInEdges().size(), expected_edges) << "Wrong number of edges";
    }
}




int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}

