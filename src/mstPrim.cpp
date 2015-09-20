#include "mstPrim.h"

/////////////////////////////////////////////////////
//  PRIM'S MINIMUM SPANNING TREE
/////////////////////////////////////////////////////

std::vector<GraphEdge> mstPrim(Graph &g, int startNode)
{
    std::vector<GraphEdge> mst;
    if (g.getNodeCount() == 0)
        return mst;
    if (startNode < 0)
        startNode = 0;

    std::vector<bool> visited(g.getNodeCount(), false);
    std::set<GraphEdge> edges;

    visited[startNode] = true;
    auto &node = g.getNode(startNode);
    for (auto &edgePtr: node)
    {
        edges.emplace(*edgePtr);
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
        auto &node = g.getNode(newNode);
        for (auto &edgePtr : node)
        {
            if (visited[edgePtr->otherEnd(newNode)])
                continue;
            edges.emplace(*edgePtr);
        }
        nodeCount++;
    }

    return mst;
}


namespace graph_algorithm_capture
{

std::vector<MstPrimState> mstPrimCaptureStates(Graph &g, int startNode)
{
    std::vector<MstPrimState> states;

    std::vector<GraphEdge> mst;
    if (g.getNodeCount() == 0)
        return states;
    std::vector<bool> visited(g.getNodeCount(), false);
    std::set<GraphEdge> edges;
    const std::vector<GraphEdge> noEdges;

    states.emplace_back(mst, visited, noEdges, edges);

    visited[startNode] = true;
    auto &node = g.getNode(startNode);
    std::vector<GraphEdge> addedEdges;
    for (auto &edgePtr : node)
    {
        edges.emplace(*edgePtr);
        addedEdges.push_back(*edgePtr);
    }
    states.emplace_back(mst, visited, addedEdges, edges);
    states.emplace_back(mst, visited, noEdges, edges);
    int nodeCount = 1;

    while (nodeCount != g.getNodeCount() && !edges.empty())
    {
        GraphEdge nextEdge = *edges.begin();
        edges.erase(edges.begin());
        states.emplace_back(mst, visited, std::vector<GraphEdge>{nextEdge}, edges);
        states.emplace_back(mst, visited, noEdges, edges);
        if (visited[nextEdge.from] && visited[nextEdge.to])
            continue;
        if (!visited[nextEdge.from] && !visited[nextEdge.to])
            throw("Edge is not connected to MST");

        int newNode = nextEdge.from;
        if (visited[newNode])
            newNode = nextEdge.to;
        mst.push_back(nextEdge);
        states.pop_back();  // remove the state when the edge is already removed from edges and not yet added to mst
        states.emplace_back(mst, visited, noEdges, edges);
        visited[newNode] = true;
        auto &node = g.getNode(newNode);
        addedEdges.clear();

        for (auto &edgePtr : node)
        {
            if (visited[edgePtr->otherEnd(newNode)])
                continue;
            edges.emplace(*edgePtr);
            addedEdges.push_back(*edgePtr);
        }
        states.emplace_back(mst, visited, addedEdges, edges);
        states.emplace_back(mst, visited, noEdges, edges);
        nodeCount++;
    }
    states.emplace_back(mst, visited, noEdges, std::set<GraphEdge>());
    return states;
}

} //namespace graph_algorithm_capture
