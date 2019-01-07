#ifndef CSPACE_HALTON_GRAPH_HPP
#define CSPACE_HALTON_GRAPH_HPP

#include "halton.hpp"
#include <arc_utilities/dijkstras.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include <arc_utilities/serialization.hpp>
// #include "dijkstras_addons.hpp"
#include <vector>
#include <cmath>




double distanceHeuristic(std::vector<double> q1, std::vector<double> q2)
{
    return EigenHelpers::Distance(q1, q2);
}



typedef arc_dijkstras::Graph<std::vector<double>> GraphD;

class HaltonGraph : public GraphD
{
public:
    double r_disc;

    int64_t addVertexAndEdges(std::vector<double> q)
    {
        int64_t new_node_ind = AddNode(q);
        for(int64_t node_ind = 0; node_ind < nodes_.size()-1; node_ind++)
        {
            double d = EigenHelpers::Distance(nodes_[node_ind].GetValueImmutable(), q);
            if(d < r_disc)
            {
                AddEdgesBetweenNodes(node_ind, new_node_ind, d);
            }
        }
        return new_node_ind;
    }
    
    HaltonGraph(int num_vert, double max_edge_dist, int dim=2)
    {
        r_disc = max_edge_dist;
        auto qs = halton::haltonPoints(num_vert, dim);
        for(auto q: qs)
        {
            addVertexAndEdges(q);
        }
    }

    HaltonGraph(const std::string& filepath)
    {
        HaltonGraph(0,0);
        loadFromFile(filepath);
    }

    uint64_t SerializeSelf(std::vector<uint8_t>& buffer,
                           const std::function<uint64_t(const std::vector<double>&, std::vector<uint8_t>&)>& value_serializer) const
    {
        uint64_t bytes_written = arc_dijkstras::Graph<std::vector<double>>::SerializeSelf(buffer, value_serializer);
        bytes_written += arc_utilities::SerializeFixedSizePOD<double>(r_disc, buffer);
        return bytes_written;
    }

    uint64_t DeserializeSeflf(
            const std::vector<uint8_t>& buffer,
            const uint64_t current,
            const std::function<std::pair<std::vector<double>, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
    {
        double c = arc_dijkstras::Graph<std::vector<double>>::DeserializeSelf(buffer, current, value_deserializer);
        auto v = arc_utilities::DeserializeFixedSizePOD<double>(buffer, c);
        r_disc = v.first;
        return v.second;
    }

    size_t countEdges()
    {
        size_t count = 0;
        for(auto &n:nodes_)
        {
            count += n.GetOutEdgesImmutable().size();
        }
        return count;
    }


    void saveToFile(const std::string& filepath)
    {
        std::vector<uint8_t> buffer;

        const auto value_serializer_fn = [] (const std::vector<double>& value,
                                             std::vector<uint8_t>& buffer)
        {
            return arc_utilities::SerializeVector<double>(value, buffer,
                                                          arc_utilities::SerializeFixedSizePOD<double>);
        };
        
        SerializeSelf(buffer, value_serializer_fn);

        ZlibHelpers::CompressAndWriteToFile(buffer, filepath);
    }

    void loadFromFile(const std::string& filepath)
    {
        const auto value_deserializer_fn = [] (const std::vector<uint8_t>& buffer,
                                               const uint64_t current)
                                             
        {
            return arc_utilities::DeserializeVector<double>(buffer, current,
                                                            arc_utilities::DeserializeFixedSizePOD<double>);
        };
        
        std::vector<uint8_t> buffer = ZlibHelpers::LoadFromFileAndDecompress(filepath);
        DeserializeSelf(buffer, (uint64_t)0, value_deserializer_fn);
    }

};

#endif
