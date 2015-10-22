#include "mstKruskal.h"
#include "UnionFind.h"

#include <algorithm>

/////////////////////////////////////////////////////
//  KRUSKAL'S MINIMUM SPANNING TREE 
/////////////////////////////////////////////////////

std::vector<gsl::not_null<GraphEdge*>> mstKruskal(Graph &g)
{
    std::vector<gsl::not_null<GraphEdge*>> mst;
    if (g.getNodeCount() == 0)
        return mst;

    UnionFind uf(int(g.getNodeCount()));
    std::vector<gsl::not_null<GraphEdge*>> edges;

    for (auto &node: g)
    {
        for (auto &edgePtr: node)
        {
            edges.push_back(&edgePtr);
        }
    }
    std::sort(begin(edges), end(edges), [](gsl::not_null<GraphEdge*> l, gsl::not_null<GraphEdge*> r) { return l->weight < r->weight; });
    
    for (size_t i = 0; i < edges.size(); ++i)
    {
        gsl::not_null<GraphEdge*> nextEdge = edges[i];
        if (uf.same(nextEdge->from, nextEdge->to))
            continue;

        uf.join(nextEdge->from, nextEdge->to);
        mst.push_back(nextEdge);
    }

    return mst;
}


namespace graph_algorithm_capture
{

std::vector<MstKruskalState> mstKruskalCaptureStates(Graph &g)
{
    std::vector<MstKruskalState> states;

    EdgePtrVector mst;
    if (g.getNodeCount() == 0)
        return states;

    UnionFind uf(int(g.getNodeCount()));
    EdgePtrSet notProcessed;
    EdgePtrVector notMst;

    std::for_each(g.edges_begin(), g.edges_end(), [&](GraphEdge &e){ notProcessed.insert(&e); });

    states.emplace_back(MstKruskalState{ mst, notMst, notProcessed, uf, nullptr, "Start" });
    while (!notProcessed.empty())
    {
        if (uf.getWeight(0) == g.getNodeCount())
        {
            states.emplace_back(MstKruskalState{ mst, notMst, notProcessed, uf, nullptr, "All nodes are connected" });
            std::copy(begin(notProcessed), end(notProcessed), std::back_inserter(notMst));
            notProcessed.clear();
            break;
        }

        GraphEdge *nextEdge = *notProcessed.begin();
        notProcessed.erase(notProcessed.begin());
        std::string stepDesc = "Check edge between node " + std::to_string(nextEdge->from + 1) + " and node " + std::to_string(nextEdge->to + 1);
        states.emplace_back(MstKruskalState{ mst, notMst, notProcessed, uf, nextEdge, stepDesc });
        if (uf.same(nextEdge->from, nextEdge->to))
        {
            states.emplace_back(MstKruskalState{ mst, notMst, notProcessed, uf, nextEdge, "The end points are already connected" });
            notMst.push_back(nextEdge);
            states.emplace_back(MstKruskalState{ mst, notMst, notProcessed, uf, nullptr, "The edge is not in the MST" });
            continue;
        }
        uf.join(nextEdge->from, nextEdge->to);
        states.emplace_back(MstKruskalState{ mst, notMst, notProcessed, uf, nextEdge, "Join two trees" });
        mst.push_back(nextEdge);
        states.emplace_back(MstKruskalState{ mst, notMst, notProcessed, uf, nextEdge, "Add edge to MST" });
    }
    states.emplace_back(MstKruskalState{ mst, notMst, notProcessed, uf, nullptr, "Done" });
    return states;
}

} //namespace graph_algorithm_capture

