#ifndef CSPACE_HALTON_GRAPH_HPP
#define CSPACE_HALTON_GRAPH_HPP

#include "halton.hpp"
#include <arc_utilities/dijkstras.hpp>
#include <arc_utilities/math_helpers.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include <arc_utilities/serialization.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <flann/flann.hpp>
#include "dijkstras_addons.hpp"
#include <vector>
#include <cmath>




inline double distanceHeuristic(std::vector<double> q1, std::vector<double> q2)
{
    return EigenHelpers::Distance(q1, q2);
}



typedef arc_dijkstras::Graph<std::vector<double>> GraphD;



namespace flann
{
    template<class T>
    struct L2_weighted
    {
        typedef bool is_kdtree_distance;

        typedef T ElementType;
        typedef T ResultType;
        typedef Eigen::Matrix<ElementType, Eigen::Dynamic, 1> VectorX;

        L2_weighted(const Eigen::VectorXd& dof_weights)
        {
            dof_weights_.resizeLike(dof_weights);
            dof_weights2_.resizeLike(dof_weights);
            for (int i = 0; i < dof_weights.rows(); ++i)
            {
                dof_weights_(i) = (ElementType)dof_weights(i);
                dof_weights2_(i) = (ElementType)(dof_weights(i) * dof_weights(i));
            }
        }

        /**
         *  Compute the squared Euclidean distance between two vectors.
         *
         *	The computation of squared root at the end is omitted for
         *	efficiency.
         */
        template <typename Iterator1, typename Iterator2>
        ResultType operator()(Iterator1 a, Iterator2 b, size_t size, ResultType worst_dist = -1) const
        {
            (void)worst_dist;
            const Eigen::Map<const VectorX> a_vec(a, size);
            const Eigen::Map<const VectorX> b_vec(b, size);
            auto delta = (a_vec - b_vec).cwiseProduct(dof_weights_);
            return delta.squaredNorm();
        }

        /**
         *	Partial euclidean distance, using just one dimension. This is used by the
         *	kd-tree when computing partial distances while traversing the tree.
         *
         *	Squared root is omitted for efficiency.
         */
        template <typename U, typename V>
        inline ResultType accum_dist(const U& a, const V& b, int ind) const
        {
            return (a-b) * (a-b) * dof_weights2_(ind);
        }

    private:
        VectorX dof_weights_;
        VectorX dof_weights2_;
    };
}



class RDiscGraph : public GraphD
{
public:
    double r_disc;
    flann::Index<flann::L2<double>> index;
    
    RDiscGraph(double r_disc) : r_disc(r_disc),
        index(flann::Matrix<double>(), flann::KDTreeIndexParams(1))
    {
    }

    RDiscGraph(const std::string& filepath):
        index(flann::Matrix<double>(), flann::KDTreeIndexParams(1))
    {
        loadFromFile(filepath);
        for(auto n:nodes_)
        {
            addToKDTree(n.getValue());
        }
    }

    RDiscGraph(const RDiscGraph &other) : GraphD(other),
                                          r_disc(other.r_disc),
                                          index(flann::Matrix<double>(), flann::KDTreeIndexParams(1))
    {
        rebuildKDTree();
    }

    
    void rebuildKDTree()
    {
        std::cout << "Rebuilding KD tree...";
        for(auto &n: getNodes())
        {
            addToKDTree(n.getValue());
        }
        std::cout << "finished\n";
    }


    void addToKDTree(std::vector<double> &point)
    {
        if(index.size() == 0)
        {
            index = flann::Index<flann::L2<double>>(flann::Matrix<double>(point.data(), 1, point.size()),
                                                    flann::KDTreeIndexParams(1));
            index.buildIndex();
            return;
        }

        index.addPoints(flann::Matrix<double>(point.data(), 1, point.size()));
    }

    int64_t addNode(const std::vector<double>& new_value)
    {
        int64_t graph_node_ind = GraphD::addNode(new_value);
        addToKDTree(getNode(graph_node_ind).getValue());
        
        assert(getNodes().size() == index.size() && "KD tree and graph had different sizes");
        return graph_node_ind;
    }

    /*
     *  NOTE: This method return nodes approximately in radius
     *   Under the hood, this uses FLANN which does not compute the exact elements in radius
     */
    std::vector<int> getVerticesWithinRadius(std::vector<double> query_point, double r) const
    {
        flann::Matrix<double> query(query_point.data(), 1, query_point.size());
        std::vector<std::vector<int>> indices(query.rows, std::vector<int>(1));
        std::vector<std::vector<double>> dists(query.rows, std::vector<double>(1));

        flann::SearchParams params(flann::flann_checks_t::FLANN_CHECKS_UNLIMITED, 0);

        index.radiusSearch(query, indices, dists, r*r, params);

        return indices[0];
    }

    int64_t getNearest(std::vector<double> query_point) const
    {
        flann::Matrix<double> query(query_point.data(), 1, query_point.size());
        std::vector<std::vector<int>> indices(query.rows, std::vector<int>(1));
        std::vector<std::vector<double>> dists(query.rows, std::vector<double>(1));

        flann::SearchParams params(flann::flann_checks_t::FLANN_CHECKS_UNLIMITED, 0);

        index.knnSearch(query, indices, dists, 1, params);
        return indices[0][0];
    }

    int64_t addVertexAndEdges(const std::vector<double> &q)
    {
        return addVertexAndEdges(q, r_disc);
    }
    
    int64_t addVertexAndEdges(const std::vector<double> &q, double radius)
    {
        int64_t new_node_ind = addNode(q);

        for(int64_t near_ind: getVerticesWithinRadius(q, radius))
        {
            if(near_ind == new_node_ind)
            {
                continue;
            }
            double d = EigenHelpers::Distance(nodes_[near_ind].getValue(), q);
            assert( d <= radius && "near point is further than allowed radius");
            addEdgesBetweenNodes(new_node_ind, near_ind, d);
        }
        return new_node_ind;
    }


    uint64_t serializeSelf(std::vector<uint8_t>& buffer,
                           const std::function<uint64_t(const std::vector<double>&,
                                                        std::vector<uint8_t>&)>& value_serializer) const
    {
        uint64_t bytes_written = arc_dijkstras::Graph<std::vector<double>>::serializeSelf(buffer,
                                                                                          value_serializer);
        bytes_written += arc_utilities::SerializeFixedSizePOD<double>(r_disc, buffer);
        return bytes_written;
    }

    uint64_t deserializeSelf(
        const std::vector<uint8_t>& buffer,
        const uint64_t current,
        const std::function<std::pair<std::vector<double>, uint64_t>(const std::vector<uint8_t>&,
                                                                     const uint64_t)>& value_deserializer)
    {
        double c = arc_dijkstras::Graph<std::vector<double>>::deserializeSelf(buffer, current,
                                                                              value_deserializer);
        auto v = arc_utilities::DeserializeFixedSizePOD<double>(buffer, c);
        r_disc = v.first;
        return v.second;
    }


    void saveToFile(const std::string& filepath)
    {
        std::vector<uint8_t> buffer;
        const auto value_serializer_fn = [] (const std::vector<double>& value,
                                             std::vector<uint8_t>& buffer)
            {
                using namespace arc_utilities;
                return SerializeVector<double>(value, buffer,
                                               SerializeFixedSizePOD<double>);
            };
        serializeSelf(buffer, value_serializer_fn);
        ZlibHelpers::CompressAndWriteToFile(buffer, filepath);
    }

    void loadFromFile(const std::string& filepath)
    {
        const auto value_deserializer_fn = [] (const std::vector<uint8_t>& buffer,
                                               const uint64_t current)
            {
                using namespace arc_utilities;
                return DeserializeVector<double>(buffer, current,
                                                 DeserializeFixedSizePOD<double>);
            };
        std::vector<uint8_t> buffer = ZlibHelpers::LoadFromFileAndDecompress(filepath);
        deserializeSelf(buffer, (uint64_t)0, value_deserializer_fn);
        rebuildKDTree();
    }

    size_t countEdges() const
    {
        size_t count = 0;
        for(auto &n:nodes_)
        {
            count += n.getOutEdges().size();
        }
        return count;
    }

};


/**
 *   Creates an 8-connected grid graph
 */
class Grid : public RDiscGraph
{
public:
    Grid(int rows):
        RDiscGraph(1.0/((double)rows - 1.0000001) * 1.4143)
    {
        for(int i=0; i<rows; i++)
        {
            for(int j=0; j<rows; j++)
            {
                std::vector<double> q{(double)i / ((double)rows - 1.0),
                        (double)j / ((double)rows - 1.0)};
                addVertexAndEdges(q);
            }
        }
    }
};




class HaltonGraph : public RDiscGraph
{
public:
    
    HaltonGraph(int num_vert, double max_edge_dist, int dim=2):
        RDiscGraph(max_edge_dist)
    {
        auto qs = halton::haltonPoints(num_vert, dim);
        for(auto q: qs)
        {
            addVertexAndEdges(q);
        }
    }


    HaltonGraph(const std::string& filepath):
        RDiscGraph(filepath)
    {};
        

    HaltonGraph() : RDiscGraph(0)
    {
    }
};

#endif
