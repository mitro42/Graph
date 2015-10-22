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
    for (auto &edge: node)
    {
        edges.insert(edge);
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
        for (auto &edge : node)
        {
            if (visited[edge.otherEnd(newNode)])
                continue;
            edges.insert(edge);
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

    if (g.getNodeCount() == 0)
        return states;

    EdgePtrVector mst;
    EdgePtrVector nonMst;
    std::vector<bool> visited(g.getNodeCount(), false);
    EdgePtrSet edges;
    const EdgePtrVector noEdges;

    states.emplace_back(mst, nonMst, visited, noEdges, edges, "Start");

    visited[startNode] = true;
    auto &node = g.getNode(startNode);
    EdgePtrVector addedEdges;
    for (auto &edge : node)
    {
        edges.insert(&edge);
        addedEdges.push_back(&edge);
    }
    states.emplace_back(mst, nonMst, visited, addedEdges, edges, "Remember edges of node " + std::to_string(startNode+1));
    states.emplace_back(mst, nonMst, visited, noEdges, edges, "");
    int nodeCount = 1;
    std::string stepDesc;
    while (nodeCount != g.getNodeCount() && !edges.empty())
    {
        GraphEdge *nextEdge = *(edges.begin());
        edges.erase(edges.begin());
        stepDesc = "Check edge between node " + std::to_string(nextEdge->from + 1) + " and node " + std::to_string(nextEdge->to + 1);
        states.emplace_back(mst, nonMst, visited, std::vector<GraphEdge*>{nextEdge}, edges, stepDesc);
        if (visited[nextEdge->from] && visited[nextEdge->to])
        {
            nonMst.push_back(nextEdge);
            states.emplace_back(mst, nonMst, visited, std::vector<GraphEdge*>{nextEdge}, edges, "Both nodes are already in the MST");
            states.emplace_back(mst, nonMst, visited, noEdges, edges, "Edge is not part of the MST");
            continue;
        }
        if (!visited[nextEdge->from] && !visited[nextEdge->to])
            throw("Edge is not connected to MST");

        int newNode = nextEdge->from;
        if (visited[newNode])
            newNode = nextEdge->to;
        mst.push_back(nextEdge);
        //states.pop_back();  // remove the state when the edge is already removed from edges and not yet added to mst
        states.emplace_back(mst, nonMst, visited, noEdges, edges, "Add edge to MST");
        visited[newNode] = true;
        auto &node = g.getNode(newNode);
        addedEdges.clear();

        for (auto &edge : node)
        {
            if (visited[edge.otherEnd(newNode)])
                continue;
            edges.insert(&edge);
            addedEdges.push_back(&edge);
        }
        states.emplace_back(mst, nonMst, visited, addedEdges, edges, "Remember edges of node " + std::to_string(newNode + 1));
        states.emplace_back(mst, nonMst, visited, noEdges, edges, "");
        nodeCount++;
    }
    
    stepDesc = (nodeCount == g.getNodeCount()) ? "Reached all nodes" : "Processed all edges";
    states.emplace_back(mst, nonMst, visited, noEdges, edges, stepDesc);

    for (const auto &edge : edges)
    {
        nonMst.push_back(edge);
    }
    states.emplace_back(mst, nonMst, visited, noEdges, EdgePtrSet(), "Done");
    return states;
}

} //namespace graph_algorithm_capture
