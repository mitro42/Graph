#ifndef MITRO_GRAPH_MST_PRIM_H
#define  MITRO_GRAPH_MST_PRIM_H
#include "Graph.h"
#include "gsl.h"

/////////////////////////////////////////////////////
//  PRIM'S MINIMUM SPANNING TREE
/////////////////////////////////////////////////////

std::vector<GraphEdge> mstPrim(Graph &g, int startNode);


namespace graph_algorithm_capture
{
    struct MstPrimState
    {
        EdgePtrVector mst;
        EdgePtrVector nonMst;
        std::vector<bool> visited;
        EdgePtrVector inspectedEdges;
        EdgePtrSet edges;
        std::string description;

        MstPrimState(const EdgePtrVector &mst,
            const EdgePtrVector &nonMst,
            const std::vector<bool> &visited, 
            const EdgePtrVector &inspectedEdges,
            const EdgePtrSet &edges,
            const std::string &description) :
            mst(mst),
            nonMst(nonMst),
            visited(visited),
            inspectedEdges(inspectedEdges),
            edges(edges),
            description(description)
        {}
    };

    std::vector<MstPrimState> mstPrimCaptureStates(Graph &g, int startNode);
} //namespace graph_algorithm_capture

#endif // MITRO_GRAPH_MST_PRIM_H
