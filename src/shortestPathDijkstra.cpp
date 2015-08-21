#include "Graph.h"


/////////////////////////////////////////////////////
//  DIJKSTRA'S SHORTEST PATH
/////////////////////////////////////////////////////

std::vector<std::pair<double, int>> nodeWeightDijkstra(Graph &g, int startNode, int endNode)
{
    std::vector<std::pair<double, int>> shortestRoute(g.getNodeCount(), std::make_pair(std::numeric_limits<double>::max(), -1));
    if (g.getNodeCount() == 0)
        return shortestRoute;

    shortestRoute[startNode] = std::make_pair(g.getNode(startNode).getWeight(), 0);

    std::set<std::pair<double, int>> q;

    q.insert(std::make_pair(g.getNode(startNode).getWeight(), startNode));

    int u = startNode;
    while (!q.empty())
    {
        u = begin(q)->second;

        if (endNode != -1 && u == endNode)
            return shortestRoute;

        for (auto &edgePtr : g.getNode(u))
        {
            double newValue = shortestRoute[u].first + edgePtr->weight;
            int neighbor = edgePtr->otherEnd(u);
            if (newValue < shortestRoute[neighbor].first)
            {
                q.erase(std::make_pair(shortestRoute[neighbor].first, neighbor));
                q.insert(std::make_pair(newValue, neighbor));
                shortestRoute[neighbor].first = newValue;
                shortestRoute[neighbor].second = u;
            }
        }
        q.erase(begin(q));
    }
    return shortestRoute;
}



std::vector<std::pair<double, int>> edgeWeightDijkstra(Graph &g, int startNode, int endNode)
{
    std::vector<std::pair<double, int>> shortestRoute(g.getNodeCount(), std::make_pair(std::numeric_limits<double>::max(), -1));
    if (g.getNodeCount() == 0)
        return shortestRoute;

    shortestRoute[startNode] = std::make_pair(0, 0);

    std::set<std::pair<double, int>> q;

    q.insert(std::make_pair(0, startNode));

    int u = startNode;
    while (!q.empty())
    {
        u = begin(q)->second;
        auto &node = g.getNode(u);
        if (endNode != -1 && u == endNode)
            return shortestRoute;

        for (auto &edgePtr : node)
        {            
            int neighbor = edgePtr->otherEnd(u);
            double newValue = shortestRoute[u].first + edgePtr->weight;
            if (newValue < shortestRoute[neighbor].first)
            {
                q.erase(std::make_pair(shortestRoute[neighbor].first, neighbor));
                q.insert(std::make_pair(newValue, neighbor));
                shortestRoute[neighbor].first = newValue;
                shortestRoute[neighbor].second = u;
            }
        }
        q.erase(begin(q));
    }
    return shortestRoute;
}

namespace graph_algorithm_capture
{
std::vector<ShortestPathEdgeWeightDijkstraState>
    edgeWeightDijkstraCaptureStates(Graph &g, int startNode, int endNode)
{
        std::vector<ShortestPathEdgeWeightDijkstraState> states;
        std::set<std::pair<double, int>> closedNodes;
        std::vector<GraphEdge> processedEdges;
        const GraphEdge noEdge(0.0, -1, -1);
        std::vector<std::pair<double, int>> shortestRoute(g.getNodeCount(), std::make_pair(std::numeric_limits<double>::max(), -1));
        if (g.getNodeCount() == 0)
            return states;

        shortestRoute[startNode] = std::make_pair(0, 0);

        std::set<std::pair<double, int>> q;

        q.insert(std::make_pair(0, startNode));
        //states.emplace_back(shortestRoute, q, startNode, noEdge);
        int u = startNode;
        std::string stepDesc;        
        while (!q.empty())
        {
            u = begin(q)->second;
            std::string uName = std::to_string(u + 1);
            states.emplace_back(shortestRoute, q, closedNodes, processedEdges, u, noEdge, "Starting processing " + uName);
            auto &node = g.getNode(u);
            if (endNode != -1 && u == endNode)
                return states;
            bool firstEdge = true;
            for (auto &edgePtr : node)
            {
                int neighbor = edgePtr->otherEnd(u);
                double weight = edgePtr->weight;
                double newValue = shortestRoute[u].first + weight;
                if (!firstEdge)
                {                    
                    states.emplace_back(shortestRoute, q, closedNodes, processedEdges, u, noEdge, stepDesc);
                }
                stepDesc = "Checking new edge: " + uName + " - " + std::to_string(neighbor + 1);
                states.emplace_back(shortestRoute, q, closedNodes, processedEdges, u, GraphEdge(NAN, u, neighbor), stepDesc);
                stepDesc = "Edge doesn't improve the path";
                if (newValue < shortestRoute[neighbor].first)
                {   
                    stepDesc = ((shortestRoute[neighbor].second == -1) ? "Found new node: " : "Found better path to ");
                    stepDesc += std::to_string(neighbor + 1);

                    q.erase(std::make_pair(shortestRoute[neighbor].first, neighbor));
                    q.insert(std::make_pair(newValue, neighbor));
                    shortestRoute[neighbor].first = newValue;
                    shortestRoute[neighbor].second = u;  
                }
                processedEdges.push_back(*edgePtr);
                states.emplace_back(shortestRoute, q, closedNodes, processedEdges, u, GraphEdge(NAN, u, neighbor), stepDesc);
                firstEdge = false;
            }
            states.emplace_back(shortestRoute, q, closedNodes, processedEdges, -1, noEdge, "Finshed processing " + uName);
            closedNodes.emplace(*begin(q));
            q.erase(begin(q));
            states.emplace_back(shortestRoute, q, closedNodes, processedEdges, -1, noEdge, "No more edges, " + uName + " is now closed");
        }
        //states.emplace_back(shortestRoute, q, closedNodes, - 1, noEdge);
        return states;
    }

} // namespace graph_algorithm_capture



