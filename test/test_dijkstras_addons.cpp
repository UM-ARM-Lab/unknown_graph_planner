// Bring in my package's API, which is what I'm testing
#include "lpa_pqueue.hpp"
#include "dijkstras_addons.hpp"
#include "halton_graph.hpp"
// Bring in gtest
#include <gtest/gtest.h>
#include <arc_utilities/pretty_print.hpp>

#define DIFF 0.0000001


TEST(DIJKSTRAS_ADDONS, queue)
{
    arc_dijkstras::pqueue<double, int64_t> queue;

    queue.insert({1.123, 2});

    EXPECT_FALSE(queue.isEmpty());
    EXPECT_EQ(queue.top().first, 1.123);
    EXPECT_EQ(queue.top().first, 1.123);
    EXPECT_EQ(queue.top().second, 2);
    EXPECT_EQ(queue.top().second, 2);

    queue.insert({1.101, 1});
    EXPECT_EQ(queue.top().first, 1.101);
    EXPECT_EQ(queue.top().second, 1);

    size_t num_erased = queue.remove(1);
    EXPECT_EQ(num_erased, 1);
    EXPECT_EQ(queue.top().first, 1.123);
    EXPECT_EQ(queue.top().second, 2);

    num_erased = queue.remove(1);
    EXPECT_EQ(num_erased, 0);
    EXPECT_EQ(queue.top().first, 1.123);
    EXPECT_EQ(queue.top().second, 2);



    queue.insert({1.345, 3});
    EXPECT_EQ(queue.top().second, 2);

    queue.insert({1.00, 3});
    EXPECT_EQ(queue.top().second, 3);

    num_erased = queue.remove(3);
    EXPECT_EQ(num_erased, 2);
    EXPECT_EQ(queue.top().second, 2);

    num_erased = queue.remove(2);
    EXPECT_EQ(num_erased, 1);
    EXPECT_TRUE(queue.isEmpty());
}

GraphD makeSimpleGraph()
{
    GraphD g;
    int64_t n1 = g.addNode(std::vector<double>{0,0});
    int64_t n2 = g.addNode(std::vector<double>{0.0, 1});
    int64_t n3 = g.addNode(std::vector<double>{1, 1});

    g.addEdgesBetweenNodes(n1, n2, 1);
    g.addEdgesBetweenNodes(n2, n3, 1);
    return g;
}

GraphD makeGrid()
{
    RDiscGraph g(1.1);
    for(int i=0; i<10; i++)
    {
        for(int j=0; j<10; j++)
        {
            g.addVertexAndEdges(std::vector<double>{(double)i, (double)j});
        }
    }
    return g;
}

TEST(DIJKSTRAS_ADDONS, LPA_trivial)
{
    using namespace arc_dijkstras;
    GraphD graph = makeSimpleGraph();

    const auto& edge_validity_fn = [] (const GraphD& g, const GraphEdge& e)
        {
            return true;
        };
    const auto& distance_fn = [] (const GraphD& g, const GraphEdge &e)
        {
            return e.getWeight();
        };
    const auto& heuristic_fn = [] (const std::vector<double>& q1, const std::vector<double>& q2)
        {
            return EigenHelpers::Distance(q1, q2);
        };
    
    LPAstar<std::vector<double>> lpa(graph, 0, 2, edge_validity_fn, distance_fn, heuristic_fn);

    auto result = lpa.computeShortestPath();
    EXPECT_EQ(result.second, 2);
    EXPECT_EQ(result.first.size(), 3);
    EXPECT_EQ(result.first[0], 0);
    EXPECT_EQ(result.first[1], 1);
    EXPECT_EQ(result.first[2], 2);
}


TEST(DIJKSTRAS_ADDONS, LPA_grid)
{
    using namespace arc_dijkstras;
    GraphD graph = makeGrid();
    EvaluatedEdges evaluated_edges;

    const auto& edge_validity_fn = [&] (const GraphD& g, const GraphEdge& e)
        {
            if(evaluated_edges.count(getHashable(e)) &&
               evaluated_edges[getHashable(e)] >= std::numeric_limits<double>::max())
            {
                return false;
            }
            return e.getValidity() != EDGE_VALIDITY::INVALID;
        };
    const auto& distance_fn = [&] (const GraphD& g, const GraphEdge &e)
        {
            if(evaluated_edges.count(getHashable(e)) &&
               evaluated_edges[getHashable(e)] >= std::numeric_limits<double>::max())
            {
                return std::numeric_limits<double>::infinity();
            }

            return e.getWeight();
        };
    const auto& heuristic_fn = [] (const std::vector<double>& q1, const std::vector<double>& q2)
        {
            return EigenHelpers::Distance(q1, q2);
        };
    LPAstar<std::vector<double>> lpa(graph, 0, 9, edge_validity_fn, distance_fn, heuristic_fn);

    auto result = lpa.computeShortestPath();
    EXPECT_EQ(result.second, 9);
    ASSERT_EQ(result.first.size(), 10);
    for(int i=0; i<10; i++)
    {
        EXPECT_EQ(result.first[i], i);
    }



    evaluated_edges[getHashable(graph.getEdge(4,5))] = std::numeric_limits<double>::infinity();
    graph.getEdge(4,5).setValidity(EDGE_VALIDITY::INVALID);
    graph.getEdge(5,4).setValidity(EDGE_VALIDITY::INVALID);
    lpa.updateEdgeCost(graph.getEdge(4,5));
    lpa.updateEdgeCost(graph.getEdge(5,4));

    result = lpa.computeShortestPath();
    EXPECT_EQ(result.second, 11);
    ASSERT_EQ(result.first.size(), 12);
    EXPECT_EQ(result.first[0], 0);
    EXPECT_EQ(result.first[11], 9);
    std::cout << PrettyPrint::PrettyPrint(result.first) << "\n";
    for(int i=0; i<11; i++)
    {
        EXPECT_NE(graph.getEdge(result.first[i], result.first[i+1]).getValidity(), EDGE_VALIDITY::INVALID);
    }
    
}


TEST(DIJKSTRAS_ADDONS, LPA_grid_inadmissible)
{
    using namespace arc_dijkstras;
    GraphD graph = makeGrid();
    EvaluatedEdges evaluated_edges;

    const auto& edge_validity_fn = [&] (const GraphD& g, const GraphEdge& e)
        {
            if(evaluated_edges.count(getHashable(e)) &&
               evaluated_edges[getHashable(e)] >= std::numeric_limits<double>::max())
            {
                return false;
            }
            return e.getValidity() != EDGE_VALIDITY::INVALID;
        };
    const auto& distance_fn = [&] (const GraphD& g, const GraphEdge &e)
        {
            if(evaluated_edges.count(getHashable(e)) &&
               evaluated_edges[getHashable(e)] >= std::numeric_limits<double>::max())
            {
                return std::numeric_limits<double>::infinity();
            }

            return e.getWeight();
        };
    const auto& heuristic_fn = [] (const std::vector<double>& q1, const std::vector<double>& q2)
        {
            return 1.2*EigenHelpers::Distance(q1, q2);
        };

    const auto& heuristic_cons_fn = [] (const std::vector<double>& q1, const std::vector<double>& q2)
        {
            return EigenHelpers::Distance(q1, q2);
        };
    LPAstar<std::vector<double>> lpa(graph, 0, 9, edge_validity_fn, distance_fn, heuristic_fn,
                                     heuristic_cons_fn);

    auto result = lpa.computeShortestPath();
    EXPECT_EQ(result.second, 9);
    ASSERT_EQ(result.first.size(), 10);
    for(int i=0; i<10; i++)
    {
        EXPECT_EQ(result.first[i], i);
    }



    evaluated_edges[getHashable(graph.getEdge(4,5))] = std::numeric_limits<double>::infinity();
    graph.getEdge(4,5).setValidity(EDGE_VALIDITY::INVALID);
    graph.getEdge(5,4).setValidity(EDGE_VALIDITY::INVALID);
    lpa.updateEdgeCost(graph.getEdge(4,5));
    lpa.updateEdgeCost(graph.getEdge(5,4));

    result = lpa.computeShortestPath();
    EXPECT_EQ(result.second, 11);
    ASSERT_EQ(result.first.size(), 12);
    EXPECT_EQ(result.first[0], 0);
    EXPECT_EQ(result.first[11], 9);
    std::cout << PrettyPrint::PrettyPrint(result.first) << "\n";
    for(int i=0; i<11; i++)
    {
        EXPECT_NE(graph.getEdge(result.first[i], result.first[i+1]).getValidity(), EDGE_VALIDITY::INVALID);
    }
    
}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
