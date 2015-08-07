#include "Graph.h"
#include "UnionFind.h"

#include <algorithm>

/////////////////////////////////////////////////////
//  KRUSKAL'S MINIMUM SPANNING TREE 
/////////////////////////////////////////////////////

std::vector<GraphEdge> mstKruskal(Graph &g)
{
    std::vector<GraphEdge> mst;
    if (g.getNodeCount() == 0)
        return mst;

    UnionFind uf(int(g.getNodeCount()));
    std::vector<GraphEdge> edges;

    for (int idx = 0; idx < g.getNodeCount(); ++idx)
    {
        auto &node = g.getNode(idx);
        for (auto &edgePtr: node)
        {
            edges.emplace_back(*edgePtr);
        }
    }
    std::sort(begin(edges), end(edges), [](const GraphEdge &l, const GraphEdge &r) { return l.weight < r.weight; });
    
    for (size_t i = 0; i < edges.size(); ++i)
    {
        GraphEdge &nextEdge = edges[i];
        if (uf.same(nextEdge.from, nextEdge.to))
            continue;

        uf.join(nextEdge.from, nextEdge.to);
        mst.push_back(nextEdge);
    }

    return mst;
}


namespace graph_algorithm_capture
{

std::vector<MstKruskalState> mstKruskalCaptureStates(Graph &g)
{
    std::vector<MstKruskalState> states;
    const GraphEdge noEdge(0.0, -1, -1);

    std::vector<GraphEdge> mst;
    if (g.getNodeCount() == 0)
        return states;

    UnionFind uf(int(g.getNodeCount()));
    std::vector<GraphEdge> edges;
    states.emplace_back(mst, edges, uf, noEdge);

    for (int idx = 0; idx < g.getNodeCount(); ++idx)
    {
        auto &node = g.getNode(idx);
        for (auto &edgePtr : node)
        {
            edges.emplace_back(*edgePtr);
        }
    }
    std::sort(begin(edges), end(edges), [](const GraphEdge &l, const GraphEdge &r) { return l.weight < r.weight; });
    
    states.emplace_back(mst, edges, uf, noEdge);
    while (!edges.empty())
    {        
        GraphEdge &nextEdge = edges[0];
        edges.erase(edges.begin());
        states.emplace_back(mst, edges, uf, nextEdge);
        if (uf.same(nextEdge.from, nextEdge.to))
            continue;

        uf.join(nextEdge.from, nextEdge.to);
        mst.push_back(nextEdge);
    }
    states.emplace_back(mst, edges, uf, noEdge);

    return states;
}

} //namespace graph_algorithm_capture

