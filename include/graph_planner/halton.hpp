#ifndef HALTON_HPP
#define HALTON_HPP

#include <vector>
#include <array>
#include <cassert>
#include <iostream>
#include <random>

namespace halton
{
    const std::vector<int> primes{2, 3, 5, 7, 11, 13, 17, 19, 23, 29,
            31, 37, 41, 43, 47, 53, 59, 61};

    const std::vector<int> hardcoded_offsets{100, 120, 234, 182, 102,
            192, 476, 203, 120, 203, 203, 403, 203, 203, 120, 1045, 302, 102};
    
    inline double haltonElement(int index, int base)
    {
        double f = 1, r = 0;
        while(index > 0){
            f = f/base;
            r = r + f* (index% base);
            index = index/base;
        }
        return r;
    };

    inline std::vector<double> haltonSeq(int base, int length, int offset)
    {
        std::vector<double> seq;
        seq.reserve(length);
        for(int ind=offset; ind < length + offset; ind++)
        {
            seq.push_back(haltonElement(ind, base));
        }
        return seq;
    };

    inline std::vector<std::vector<double> > haltonPoints(std::vector<int> bases,
                                                          int length,
                                                          std::vector<int> offsets)
    {
        assert(bases.size() == offsets.size());
        std::vector<std::vector<double> > seqs(bases.size());
        for(int i=0; i<bases.size(); i++)
        {
            seqs[i] = haltonSeq(bases[i], length, offsets[i]);
        }

        std::vector<std::vector<double> > points(length, std::vector<double>(bases.size()));
        for(int i=0; i<length; i++)
        {
            for(int j=0; j<bases.size(); j++)
            {
                points[i][j] = seqs[j][i];
            }
        }
        return points;
    };

    inline std::vector<std::vector<double> > haltonPoints(int length, int dim)
    {
        std::vector<int> bases(dim), offsets(dim);
        for(int i=0; i<dim; i++)
        {
            bases[i] = primes[i];
            offsets[i] = hardcoded_offsets[i];
        }
        return haltonPoints(bases, length, offsets);
    };
    
}





#endif
