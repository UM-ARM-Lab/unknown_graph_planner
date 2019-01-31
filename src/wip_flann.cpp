#include <flann/flann.hpp>


#include <stdio.h>


int main()
{
    std::vector<double> point{
        0, 1, 2, 3,
            0, 1, 2, 4};
    flann::Matrix<double> mat(point.data(), 2, 4);
    
    flann::Index<flann::L2<double>> index(mat, flann::KDTreeIndexParams(1));
    index.buildIndex();

    std::vector<double> query_vec{0,1,2,3.4};
    flann::Matrix<double> query(query_vec.data(), 1, 4);

    std::vector<std::vector<int>> indices(query.rows, std::vector<int>(1));
    std::vector<std::vector<double>> dists(query.rows, std::vector<double>(1));

    flann::SearchParams params(flann::flann_checks_t::FLANN_CHECKS_UNLIMITED, 0);

    std::cout << "Doing radius search\n";
    double radius = 0.61;
    index.radiusSearch(query, indices, dists, radius*radius, params);
    // index.knnSearch(query, indices, dists, 1, params);


    std::cout << "Indices within radius: ";
    for(size_t i=0; i<indices[0].size(); i++)
    {
        std::cout << "(" << indices[0][i] << ": " << std::sqrt(dists[0][i]) <<"), ";
    }
    std::cout << "\n";

    double* out_point = index.getPoint(0);
    std::cout << "Element has value (" <<
        out_point[0] << ", " <<
        out_point[1] << ", " <<
        out_point[2] << ", " <<
        out_point[3] << ")\n";
    return 0;
}
