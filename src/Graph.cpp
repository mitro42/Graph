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

/*
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
*/

//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//   Graph
/////////////////////////////////////////////////////

void Graph::resize(int newNodes)
{
    if (newNodes < int(nodes.size()))
        throw("Graph::resize - cannot decrease size");

    if (newNodes == nodes.size())
        return;

    nodes.reserve(newNodes);
    while (nodes.size() < newNodes)
    {
        nodes.push_back(std::make_unique<GraphNode>());
    }
}


int Graph::addNode(double weight)
{
    nodes.emplace_back(std::make_unique<GraphNode>(weight));
    return int(nodes.size()) - 1;
}

void Graph::addEdge(int from, int to, double weight)
{
    edges.emplace_back(std::make_unique<GraphEdge>(weight, from, to));

    nodes[from]->addEdge(edges.rbegin()->get());
    if (!directed)
    {
        nodes[to]->addEdge(edges.rbegin()->get());

    }
}


void Graph::removeEdge(int from, int to)
{
    auto firstEdge = std::remove_if(edges.begin(), edges.end(),
        [&from, &to](const std::unique_ptr<GraphEdge> &e) { return e->from == from && e->to == to; });

    for (auto edge = firstEdge; edge != edges.end(); ++edge)
    {
        gsl::not_null<GraphEdge*> edgePtr = edge->get();
        nodes[from]->removeEdge(edgePtr);
        if (!directed)
        {
            nodes[to]->removeEdge(edgePtr);
        }
    }
    edges.erase(firstEdge, edges.end());
}

void Graph::clear(bool newDirected)
{
    directed = newDirected;
    nodes.clear();
    edges.clear();
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

    os << g.edges.size() << "\n";
    os << g.isDirected() << " " << g.hasWeightedNodes() << " " << g.hasWeightedEdges() << "\n";

    if (g.hasWeightedNodes())
    {
        for (const auto &node : g.nodes)
        {
            os << node->getWeight() << "\n";
        }
    }

    for (const auto &edge : g.edges)
    {
        os << edge->from + 1 << " " << edge->to + 1;
        if (g.hasWeightedEdges())
        {
            os << " " << edge->weight;
        }
        os << "\n";
    }

    os << std::flush;

    return os;
}



