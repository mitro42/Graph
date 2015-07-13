#include "Graph.h"
#include "UnionFind.h"


/////////////////////////////////////////////////////
//  KRUSKAL'S MINIMUM SPANNING TREE 
/////////////////////////////////////////////////////

std::vector<GraphEdge> mstKruskal(const Graph &g)
{
    std::vector<GraphEdge> mst;
    if (g.getNodeCount() == 0)
        return mst;

    UnionFind uf(int(g.getNodeCount()));
    std::set<GraphEdge> edges;

    for (size_t idx = 0; idx < g.getNodeCount(); ++idx)
    {
        auto &node = g.getNode(idx);
        for (size_t neighborIdx = 0; neighborIdx < node.getNeighborCount(); ++neighborIdx)
        {
            edges.emplace(node.getEdgeWeight(neighborIdx), int(idx), node.getNeighbor(neighborIdx));
        }
    }

    while (!edges.empty())
    {
        GraphEdge nextEdge = *edges.begin();
        edges.erase(edges.begin());
        if (uf.same(nextEdge.from, nextEdge.to))
            continue;

        uf.join(nextEdge.from, nextEdge.to);
        mst.push_back(nextEdge);
    }

    return mst;
}


namespace graph_algorithm_capture
{

std::vector<GraphEdge> mstKruskalCaptureStates(const Graph &g)
{
    std::vector<GraphEdge> mst;
    if (g.getNodeCount() == 0)
        return mst;

    UnionFind uf(int(g.getNodeCount()));
    std::set<GraphEdge> edges;

    for (size_t idx = 0; idx < g.getNodeCount(); ++idx)
    {
        auto &node = g.getNode(idx);
        for (size_t neighborIdx = 0; neighborIdx < node.getNeighborCount(); ++neighborIdx)
        {
            edges.emplace(node.getEdgeWeight(neighborIdx), int(idx), node.getNeighbor(neighborIdx));
        }
    }

    while (!edges.empty())
    {
        GraphEdge nextEdge = *edges.begin();
        edges.erase(edges.begin());
        if (uf.same(nextEdge.from, nextEdge.to))
            continue;

        uf.join(nextEdge.from, nextEdge.to);
        mst.push_back(nextEdge);
    }

    return mst;
}

} //namespace graph_algorithm_capture

