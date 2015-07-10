#ifndef MITRO_UTIL_UNION_FIND_H
#define MITRO_UTIL_UNION_FIND_H

#include <vector>

class UnionFind
{
public:
    UnionFind(int n);
    int root(int idx);
    void join(int a, int b);
    bool same(int a, int b);
private:
    std::vector<int> id;
    std::vector<int> weight;
};

#endif // MITRO_UTIL_UNION_FIND_H