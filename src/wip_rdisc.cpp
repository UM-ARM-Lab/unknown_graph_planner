#include "halton_graph.hpp"
#include "increasing_density_grid.hpp"



int main()
{
    RDiscGraph g(0.1);
    g.addVertexAndEdges(std::vector<double>{1,2});

    double* out_point = g.index.getPoint(0);
    std::cout << "Element 0 has value (" <<
        out_point[0] << ", " <<
        out_point[1] << ")\n";

    
    g.getVerticesWithinRadius(std::vector<double>{1,2.0}, 0.1);

    
    g.addVertexAndEdges(std::vector<double>{1,3});
    g.addVertexAndEdges(std::vector<double>{2,4});

    HaltonGraph g2(1000000, 0.2, 7);
    std::cout << " graph has " << g2.getNodes().size() << " nodes and " << g2.countEdges() << " edges\n";

    // IncreasingDensityGrid g(2);

    // for(auto i=0; i<g.getNodes().size(); i++)
    // {
    //     DepthNode dn = g.getNodeValue(i);
    //     std::cout << "<" << PrettyPrint::PrettyPrint(dn.q) << "> at depth " << dn.depth << "\n";
    // }
    
    return 0;
}
