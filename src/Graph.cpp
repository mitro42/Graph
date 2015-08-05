#include "Graph.h"

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

void Graph::resize(int newNodes)
{
    if (newNodes < int(nodes.size()))
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

void Graph::clear(bool newDirected)
{
    directed = newDirected;
    nodes.clear();
}


std::vector<GraphEdge> Graph::getEdges() const
{
    std::vector<GraphEdge> ret;
    for (size_t nodeIdx = 0; nodeIdx < nodes.size(); ++nodeIdx)
    {
        auto &node = nodes[nodeIdx];
        for (size_t neighborIdx = 0; neighborIdx < node.neighbors.size(); ++neighborIdx)
        {
            if (directed || node.neighbors[neighborIdx] >= int(nodeIdx))
            {
                ret.emplace_back(node.edgeWeights[neighborIdx], int(nodeIdx), node.neighbors[neighborIdx]);
            }
        }
    }
    return ret;
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
    

    int n, m;
    int directedFlag, weightedNodesFlag, weightedEdgesFlag;

    is >> n >> m;
    is >> directedFlag >> weightedNodesFlag >> weightedEdgesFlag;

    g.clear(directedFlag != 0);
    g.resize(n);
    
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
        for (int i = 0; i < node.getNeighborCount(); ++i)
        {
            if (g.isDirected() || nodeIdx < node.getNeighbor(i))
            {
                os << nodeIdx + 1 << " " << node.getNeighbor(i) + 1;
                if (g.hasWeightedEdges())
                {
                    os << " " << node.getEdgeWeight(int(i));
                }
                os << "\n";
            }
        }
    }

    os << std::flush;

    return os;
}



