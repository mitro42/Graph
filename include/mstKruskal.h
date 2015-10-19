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
        EdgePtrVector mst;
        EdgePtrVector notMst;
        EdgePtrSet notProcessed;
        UnionFind uf;
        const GraphEdge *inspectedEdge;
        std::string description;
    };

    std::vector<MstKruskalState> mstKruskalCaptureStates(Graph &g);
} //namespace graph_algorithm_capture

#endif // MITRO_GRAPH_MST_KRUSKAL_H
