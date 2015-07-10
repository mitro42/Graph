#include "UnionFind.h"

UnionFind::UnionFind(int n) : id(n), weight(n, 1)
{
    for (int i = 0; i < n; ++i)
        id[i] = i;
}

int UnionFind::root(int idx)
{
    while (idx != id[idx])
    {
        id[idx] = id[id[idx]];
        idx = id[idx];
    }
    return idx;
}

void UnionFind::join(int a, int b)
{
    int p = root(a);
    int q = root(b);

    if (p != q)
    {
        if (weight[p] > weight[q]) std::swap(p, q);

        id[p] = id[q];
        weight[q] += weight[p];
    }
}

bool UnionFind::same(int a, int b)
{
    return root(a) == root(b);
}
