#include "Graph.h"

/////////////////////////////////////////////////////
//  PRIM'S MINIMUM SPANNING TREE
/////////////////////////////////////////////////////

std::vector<GraphEdge> mstPrim(const Graph &g, int startNode)
{
    std::vector<GraphEdge> mst;
    if (g.getNodeCount() == 0)
        return mst;
    if (startNode < 0)
        startNode = 0;

    std::vector<bool> visited(g.getNodeCount(), false);
    std::set<GraphEdge> edges;

    visited[startNode] = true;
    const auto &node = g.getNode(startNode);
    for (int neighborIdx = 0; neighborIdx < node.getNeighborCount(); ++neighborIdx)
    {
        edges.emplace(node.getEdgeWeight(neighborIdx), startNode, node.getNeighbor(neighborIdx));
    }
    int nodeCount = 1;

    while (nodeCount != g.getNodeCount() && !edges.empty())
    {
        GraphEdge nextEdge = *edges.begin();
        edges.erase(edges.begin());
        if (visited[nextEdge.from] && visited[nextEdge.to])
            continue;
        if (!visited[nextEdge.from] && !visited[nextEdge.to])
            throw("Edge is not connected to MST");

        int newNode = nextEdge.from;
        if (visited[newNode])
            newNode = nextEdge.to;
        mst.push_back(nextEdge);
        visited[newNode] = true;
        const auto &node = g.getNode(newNode);
        for (int neighborIdx = 0; neighborIdx < node.getNeighborCount(); ++neighborIdx)
        {
            int otherEnd = node.getNeighbor(neighborIdx);
            if (visited[otherEnd])
                continue;
            edges.emplace(node.getEdgeWeight(neighborIdx), newNode, otherEnd);
        }
        nodeCount++;
    }

    return mst;
}


namespace graph_algorithm_capture
{

std::vector<MstPrimState> mstPrimCaptureStates(const Graph &g, int startNode)
{
    std::vector<MstPrimState> states;

    std::vector<GraphEdge> mst;
    if (g.getNodeCount() == 0)
        return states;
    std::vector<bool> visited(g.getNodeCount(), false);
    std::set<GraphEdge> edges;
    const GraphEdge noEdge(0.0, -1, -1);

    states.emplace_back(mst, visited, noEdge, edges);

    visited[startNode] = true;
    const auto &node = g.getNode(startNode);
    for (int neighborIdx = 0; neighborIdx < node.getNeighborCount(); ++neighborIdx)
    {
        GraphEdge currentEdge(node.getEdgeWeight(neighborIdx), startNode, node.getNeighbor(neighborIdx));
        edges.emplace(currentEdge);
        states.emplace_back(mst, visited, currentEdge, edges);
        states.emplace_back(mst, visited, noEdge, edges);
    }
    int nodeCount = 1;

    while (nodeCount != g.getNodeCount() && !edges.empty())
    {
        GraphEdge nextEdge = *edges.begin();
        edges.erase(edges.begin());
        states.emplace_back(mst, visited, nextEdge, edges);
        states.emplace_back(mst, visited, noEdge, edges);
        if (visited[nextEdge.from] && visited[nextEdge.to])
            continue;
        if (!visited[nextEdge.from] && !visited[nextEdge.to])
            throw("Edge is not connected to MST");

        int newNode = nextEdge.from;
        if (visited[newNode])
            newNode = nextEdge.to;
        mst.push_back(nextEdge);
        states.pop_back();  // remove the state when the edge is already removed from edges and not yet added to mst
        states.emplace_back(mst, visited, noEdge, edges);
        visited[newNode] = true;
        const auto &node = g.getNode(newNode);
        for (int neighborIdx = 0; neighborIdx < node.getNeighborCount(); ++neighborIdx)
        {            
            int otherEnd = node.getNeighbor(neighborIdx);
            if (visited[otherEnd])
                continue;
            GraphEdge currentEdge(node.getEdgeWeight(neighborIdx), newNode, otherEnd);
            edges.emplace(currentEdge);
            states.emplace_back(mst, visited, currentEdge, edges);
            states.emplace_back(mst, visited, noEdge, edges);
        }
        nodeCount++;
    }
    states.emplace_back(mst, visited, GraphEdge(0.0, -1, -1), std::set<GraphEdge>());
    return states;
}

} //namespace graph_algorithm_capture
