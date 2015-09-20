#ifndef MITRO_GRAPH_MST_PRIM_H
#define  MITRO_GRAPH_MST_PRIM_H
#include "Graph.h"

/////////////////////////////////////////////////////
//  PRIM'S MINIMUM SPANNING TREE
/////////////////////////////////////////////////////

std::vector<GraphEdge> mstPrim(Graph &g, int startNode);


namespace graph_algorithm_capture
{
    struct MstPrimState
    {
        std::vector<GraphEdge> mst;
        std::vector<bool> visited;
        std::vector<GraphEdge> inspectedEdges;
        std::set<GraphEdge> edges;
        MstPrimState(const std::vector<GraphEdge> &mst, const std::vector<bool> &visited, const std::vector<GraphEdge> &inspectedEdges, const std::set<GraphEdge> &edges) :
            mst(mst),
            visited(visited),
            inspectedEdges(inspectedEdges),
            edges(edges)
        {}
    };

    std::vector<MstPrimState> mstPrimCaptureStates(Graph &g, int startNode);
} //namespace graph_algorithm_capture

#endif // MITRO_GRAPH_MST_PRIM_H
