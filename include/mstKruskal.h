#ifndef MITRO_GRAPH_MST_KRUSKAL_H
#define  MITRO_GRAPH_MST_KRUSKAL_H

#include "Graph.h"
#include "UnionFind.h"

/////////////////////////////////////////////////////
//  KRUSKAL'S MINIMUM SPANNING TREE 
/////////////////////////////////////////////////////

std::vector<GraphEdge> mstKruskal(Graph &g);

namespace graph_algorithm_capture
{
    struct MstKruskalState
    {
        std::vector<GraphEdge> mst;
        std::vector<GraphEdge> edges;
        UnionFind uf;
        GraphEdge inspectedEdge;
        MstKruskalState(const std::vector<GraphEdge> mst, const std::vector<GraphEdge> &edges, const UnionFind &uf, const GraphEdge &inspectedEdge) :
            mst(mst),
            edges(edges),
            uf(uf),
            inspectedEdge(inspectedEdge)
        {}
    };

    std::vector<MstKruskalState> mstKruskalCaptureStates(Graph &g);
} //namespace graph_algorithm_capture

#endif // MITRO_GRAPH_MST_KRUSKAL_H
