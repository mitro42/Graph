#include "Graph.h"
#include "UnionFind.h"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <set>
#include <vector>
#include <map>
#include <functional>

/////////////////////////////////////////////////////
//  Node of Graph
/////////////////////////////////////////////////////


void GraphNode::removeNeighbor(int to)
{
    auto neighborIt = std::find(neighbors.begin(), neighbors.end(), to);
    auto edgeWeightIt = edgeWeights.begin() + (neighborIt - neighbors.begin());
    std::swap(*neighborIt, *neighbors.rbegin());
    std::swap(*edgeWeightIt, *edgeWeights.rbegin());
    neighbors.erase(neighbors.end() - 1, neighbors.end());
    edgeWeights.erase(edgeWeights.end() - 1, edgeWeights.end());
}

void GraphNode::addNeighbor(int to, double weight)
{
    auto it = std::lower_bound(neighbors.begin(), neighbors.end(), to);
    if (it != neighbors.end() && *it == to)
        return;
    edgeWeights.insert(edgeWeights.begin() + (it - neighbors.begin()), weight);
    neighbors.insert(it, to);
}

//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//   Graph
/////////////////////////////////////////////////////

void Graph::resize(size_t newNodes)
{
    if (newNodes < nodes.size())
        throw("Graph::resize - cannot decrease size");

    if (newNodes == nodes.size())
        return;

    nodes.resize(newNodes);
}


int Graph::addNode(double weight)
{
    nodes.emplace_back(weight);
    return int(nodes.size()) - 1;
}

void Graph::addEdge(int from, int to, double weight)
{
    nodes[from].addNeighbor(to, weight);
    if (!directed)
    {
        nodes[to].addNeighbor(from, weight);

    }
}


void Graph::removeEdge(int from, int to)
{
    nodes[from].removeNeighbor(to);
    if (!directed)
    {
        nodes[to].removeNeighbor(from);
    }
}

void Graph::clear()
{
    nodes.clear();
}


std::vector<GraphEdge> Graph::getEdges() const
{
    std::vector<GraphEdge> ret;
    for (size_t nodeIdx = 0; nodeIdx < nodes.size(); ++nodeIdx)
    {
        auto &node = nodes[nodeIdx];
        for (int neighborIdx = 0; neighborIdx < node.getNeighborCount(); ++neighborIdx)
        {
            if (directed || node.getNeighbor(neighborIdx) >= nodeIdx)
            {
                ret.emplace_back(node.getEdgeWeight(neighborIdx), nodeIdx, node.getNeighbor(neighborIdx));
            }
        }
    }
    return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Graph algorithms
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////
//  DIJKSTRA
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
            std::vector<std::pair<int64_t, int>> removedElements;
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
            std::vector<std::pair<int64_t, int>> removedElements;
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


std::vector<std::pair<std::vector<std::pair<double, int>>, std::set<std::pair<double, int>>>> 
edgeWeightDijkstraCaptureStates(const Graph &g, int startNode, int endNode)
{
    std::vector<std::pair<std::vector<std::pair<double, int>>, std::set<std::pair<double, int>>>> states;
    std::vector<std::pair<double, int>> shortestRoute(g.getNodeCount(), std::make_pair(std::numeric_limits<double>::max(), -1));
    if (g.getNodeCount() == 0)
        return states;

    shortestRoute[startNode] = std::make_pair(0, 0);

    std::set<std::pair<double, int>> q;

    q.insert(std::make_pair(0, startNode));

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
            std::vector<std::pair<int64_t, int>> removedElements;
            double newValue = shortestRoute[u].first + node.getEdgeWeight(neighborIdx);
            if (newValue < shortestRoute[neighbor].first)
            {
                states.emplace_back(shortestRoute, q);
                q.erase(std::make_pair(shortestRoute[neighbor].first, neighbor));
                q.insert(std::make_pair(newValue, neighbor));
                shortestRoute[neighbor].first = newValue;
                shortestRoute[neighbor].second = u;
            }
        }
        states.emplace_back(shortestRoute, q);
        q.erase(begin(q));
    }
    return states;
}



/////////////////////////////////////////////////////
//  PRIM
/////////////////////////////////////////////////////



std::vector<GraphEdge> mstPrim(const Graph &g, int startNode)
{
    std::vector<GraphEdge> mst;
    if (g.getNodeCount() == 0)
        return mst;

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




/////////////////////////////////////////////////////
//  KRUSKAL
/////////////////////////////////////////////////////

std::vector<GraphEdge> mstKruskal(const Graph &g)
{
    std::vector<GraphEdge> mst;
    if (g.getNodeCount() == 0)
        return mst;

    UnionFind uf(int(g.getNodeCount()));
    std::set<GraphEdge> edges;

    for (int idx = 0; idx < g.getNodeCount(); ++idx)
    {
        auto &node = g.getNode(idx);
        for (int neighborIdx = 0; neighborIdx < node.getNeighborCount(); ++neighborIdx)
        {
            edges.emplace(node.getEdgeWeight(neighborIdx), idx, node.getNeighbor(neighborIdx));
        }
    }

    while (!edges.empty())
    {
        GraphEdge nextEdge = *edges.begin();
        edges.erase(edges.begin());
        if (uf.same(nextEdge.from, nextEdge.to))
            continue;

        uf.join(nextEdge.from, nextEdge.to);
        mst.push_back(nextEdge);
    }

    return mst;
}


/////////////////////////////////////////////////////
//  Input / Output
/////////////////////////////////////////////////////
//
// Format:
// NumberOfNodes NumberOfEdges 
// DirectedFlag WeightedNodesFlag WeightedEdgesFlag
// If WeightedNodesFlag == 1, the node weights follow, NumberOfNodes numbers
//    else this section is omitted
// NumberOfEdges edges follow:
// From To [Weight]
// Wight is omitted if NumberOfEdges == 0

// Flags are 0 or 1
// All weights are floating point numbers
// Everything is whitespace separated
// Indexes are 1 based.

// e.g
// 3 2 
// 1 1 0
// 5.4
// 0.6
// 1
// 2 3
// 3 1

//  2  ----> 3 ----> 1
// 0.6       1      5.4

std::istream &operator>>(std::istream &is, Graph &g)
{
    g.clear();

    int n, m;
    int directedFlag, weightedNodesFlag, weightedEdgesFlag;

    is >> n >> m;
    is >> directedFlag >> weightedNodesFlag >> weightedEdgesFlag;

    g.resize(n);
    g.setDirected(directedFlag != 0);
    g.setWeightedNodes(weightedNodesFlag != 0);
    g.setWeightedEdges(weightedEdgesFlag != 0);

    if (g.hasWeightedNodes())
    {
        for (auto &node : g)
        {
            double w;
            is >> w;
            node.setWeight(w);
        }
    }

    for (int i = 0; i < m; ++i)
    {
        int from, to;
        is >> from >> to;
        double weight = 0.0;
        if (g.hasWeightedEdges())
        {
            is >> weight;
        }
        g.addEdge(from - 1, to - 1, weight);
    }

    return is;
}

std::ostream &operator<<(std::ostream &os, const Graph &g)
{
    os << g.getNodeCount() << " ";

    size_t edgesCount = 0;

    for (auto &node : g)
    {
        edgesCount += node.getNeighborCount();
    }

    if (g.isDirected())
    {
        os << edgesCount << "\n";
    }
    else
    {
        os << edgesCount / 2 << "\n";
    }

    os << g.isDirected() << " " << g.hasWeightedNodes() << " " << g.hasWeightedEdges() << "\n";

    if (g.hasWeightedNodes())
    {
        for (auto &node : g)
        {
            os << node.getWeight() << "\n";
        }
    }

    for (int nodeIdx = 0; nodeIdx < g.getNodeCount(); ++nodeIdx)
    {
        auto &node = g.getNode(nodeIdx);
        for (size_t i = 0; i < node.getNeighborCount(); ++i)
        {
            if (g.isDirected() || nodeIdx < node.getNeighbor(int(i)))
            {
                os << nodeIdx + 1 << " " << node.getNeighbor(int(i)) + 1;
            }
            if (g.hasWeightedEdges())
            {
                os << " " << node.getEdgeWeight(int(i));
            }
            os << "\n";
        }
    }

    os << std::flush;

    return os;
}



