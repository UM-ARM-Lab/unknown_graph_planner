#include "halton_graph.hpp"
#include "increasing_density_grid.hpp"
#include "dijkstras_addons.hpp"
#include "lpastar.hpp"




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


int main()
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
    
    const auto& heuristic_cons_fn = [] (const std::vector<double>& q1, const std::vector<double>& q2)
        {
            return EigenHelpers::Distance(q1, q2);
        };
    // double inflation = 1.2;
    LPAstar<std::vector<double>> lpa(graph, 0, 9, edge_validity_fn, distance_fn, heuristic_fn,
                                     heuristic_cons_fn);

    auto result = lpa.computeShortestPath();
    // EXPECT_EQ(result.second, 9);
    // ASSERT_EQ(result.first.size(), 10);
    // for(int i=0; i<10; i++)
    // {
    //     EXPECT_EQ(result.first[i], i);
    // }
    std::cout << "First path with no invalid edges\n";
    std::cout << PrettyPrint::PrettyPrint(result.first) << "\n";


    evaluated_edges[getHashable(graph.getEdge(4,5))] = std::numeric_limits<double>::infinity();
    graph.getEdge(4,5).setValidity(EDGE_VALIDITY::INVALID);
    graph.getEdge(5,4).setValidity(EDGE_VALIDITY::INVALID);
    lpa.updateEdgeCost(graph.getEdge(4,5));
    lpa.updateEdgeCost(graph.getEdge(5,4));

    result = lpa.computeShortestPath();
    // EXPECT_EQ(result.second, 11);
    // ASSERT_EQ(result.first.size(), 12);
    // EXPECT_EQ(result.first[0], 0);
    // EXPECT_EQ(result.first[11], 9);
    std::cout << "Updated path with edge invalidated\n";
    std::cout << PrettyPrint::PrettyPrint(result.first) << "\n";
    // for(int i=0; i<11; i++)
    // {
    //     EXPECT_NE(graph.getEdge(result.first[i], result.first[i+1]).getValidity(), EDGE_VALIDITY::INVALID);
    // }
}
