#include "Graph.h"

/////////////////////////////////////////////////////
//  DIJKSTRA'S SHORTEST PATH
/////////////////////////////////////////////////////

std::vector<std::pair<double, int>> nodeWeightDijkstra(const Graph &g, int startNode, int endNode)
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

        for (auto &neighbor : g.getNode(u))
        {
            double newValue = shortestRoute[u].first + g.getNode(neighbor).getWeight();
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



std::vector<std::pair<double, int>> edgeWeightDijkstra(const Graph &g, int startNode, int endNode)
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

        for (int neighborIdx = 0; neighborIdx < node.getNeighborCount(); ++neighborIdx)
        {
            auto neighbor = node.getNeighbor(neighborIdx);
            double newValue = shortestRoute[u].first + node.getEdgeWeight(neighborIdx);
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
    edgeWeightDijkstraCaptureStates(const Graph &g, int startNode, int endNode)
{
        std::vector<ShortestPathEdgeWeightDijkstraState> states;
        const GraphEdge noEdge(0.0, -1, -1);
        std::vector<std::pair<double, int>> shortestRoute(g.getNodeCount(), std::make_pair(std::numeric_limits<double>::max(), -1));
        if (g.getNodeCount() == 0)
            return states;

        shortestRoute[startNode] = std::make_pair(0, 0);

        std::set<std::pair<double, int>> q;

        q.insert(std::make_pair(0, startNode));
        states.emplace_back(shortestRoute, q, startNode, noEdge);
        int u = startNode;
        while (!q.empty())
        {
            u = begin(q)->second;
            auto &node = g.getNode(u);
            if (endNode != -1 && u == endNode)
                return states;

            for (int neighborIdx = 0; neighborIdx < node.getNeighborCount(); ++neighborIdx)
            {
                auto neighbor = node.getNeighbor(neighborIdx);
                double weight = node.getEdgeWeight(neighborIdx);
                double newValue = shortestRoute[u].first + weight;
                if (newValue < shortestRoute[neighbor].first)
                {   
                    states.emplace_back(shortestRoute, q, u, GraphEdge(NAN, shortestRoute[neighbor].second, neighbor));
                    q.erase(std::make_pair(shortestRoute[neighbor].first, neighbor));
                    q.insert(std::make_pair(newValue, neighbor));
                    shortestRoute[neighbor].first = newValue;
                    shortestRoute[neighbor].second = u;
                    states.emplace_back(shortestRoute, q, u, GraphEdge(NAN, u, neighbor));
                }
            }            
            q.erase(begin(q));
        }
        states.emplace_back(shortestRoute, q, -1, noEdge);
        return states;
    }

} // namespace graph_algorithm_capture



