#ifndef HALTON_HPP
#define HALTON_HPP

#include <vector>
#include <array>
#include <cassert>
#include <iostream>

double haltonElement(int index, int base){
  double f = 1, r = 0;
  while(index > 0){
    f = f/base;
    r = r + f* (index% base);
    index = index/base;
  }
  return r;
};

std::vector<double> haltonSeq(int base, int length, int offset)
{
    std::vector<double> seq;
    seq.reserve(length);
    for(int ind=offset; ind < length + offset; ind++)
    {
        seq.push_back(haltonElement(ind, base));
    }
    return seq;
};

std::vector<std::vector<double> > haltonPoints(std::vector<int> bases,
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




#endif
