#ifndef MITRO_GRAPH_SHORTEST_PATH_DIJKSTRA_H
#define MITRO_GRAPH_SHORTEST_PATH_DIJKSTRA_H
#include "Graph.h"
#include <gsl.h>
/////////////////////////////////////////////////////
//  DIJKSTRA'S SHORTEST PATH
/////////////////////////////////////////////////////

std::vector<std::pair<double, const GraphEdge*>> nodeWeightDijkstra(Graph &g, int startNode, int endNode);
std::vector<std::pair<double, const GraphEdge*>> edgeWeightDijkstra(Graph &g, int startNode, int endNode);

namespace graph_algorithm_capture
{
    struct ShortestPathEdgeWeightDijkstraState
    {
        std::vector<std::pair<double, const GraphEdge*>> path;
        std::set<std::pair<double, int>> openNodes; // discovered but not fully processed
        std::set<std::pair<double, int>> closedNodes; //  fully processed
        std::vector<gsl::not_null<const GraphEdge*>> processedEdges; // processed, not part of any of the minimal paths
        int inspectedNode;
        const GraphEdge * inspectedEdge;
        std::string description;

        ShortestPathEdgeWeightDijkstraState(const std::vector<std::pair<double, const GraphEdge*>> &path,
            const std::set<std::pair<double, int>> &openNodes,
            const std::set<std::pair<double, int>> &closedNodes,
            const std::vector<gsl::not_null<const GraphEdge*>> &processedEdges,
            int inspectedNode,
            const GraphEdge* inspectedEdge,
            const std::string &description) :
            path(path),
            openNodes(openNodes),
            closedNodes(closedNodes),
            processedEdges(processedEdges),
            inspectedNode(inspectedNode),
            inspectedEdge(inspectedEdge),
            description(description)
        {}
    };

    std::vector<ShortestPathEdgeWeightDijkstraState>
    edgeWeightDijkstraCaptureStates(Graph &g, int startNode, int endNode);
} // namespace graph_algorithm_capture

#endif // MITRO_GRAPH_SHORTEST_PATH_DIJKSTRA_H


