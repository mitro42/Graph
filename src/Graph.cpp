#include "Graph.h"
#include <algorithm>
#include <numeric>
#include <set>
#include <vector>

/////////////////////////////////////////////////////
//  Node of Graph
/////////////////////////////////////////////////////


void GraphNode::removeNeighbor(int to)
{
	neighbors.erase(std::find(neighbors.begin(), neighbors.end(), to), neighbors.end());
}

void GraphNode::addNeighbor(int to)
{
	auto it = std::lower_bound(neighbors.begin(), neighbors.end(), to);
	if (it != neighbors.end() && *it == to)
		return;
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


int Graph::addNode()
{
	nodes.emplace_back();
	return nodes.size() - 1;
}

void Graph::addEdge(int from, int to)
{
	nodes[from].addNeighbor(to);
	if (!directed)
	{
		nodes[to].addNeighbor(from);
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


/////////////////////////////////////////////////////
//  Graph algorithms
/////////////////////////////////////////////////////


std::vector<std::pair<int64_t, int>> findMinimalPathDijkstra(const Graph &g, int startNode, int endNode)
{
	std::vector<std::pair<int64_t, int>> shortestRoute(g.getNodeCount(), std::make_pair(std::numeric_limits<int64_t>::max(), -1));
	shortestRoute[startNode] = std::make_pair(g.getNode(startNode).getValue(), 0);

	std::set<std::pair<int64_t, int>> q;

	q.insert(std::make_pair(g.getNode(startNode).getValue(), startNode));

	int u = startNode;
	while (!q.empty())
	{
		u = begin(q)->second;

		if (endNode != -1 && u == endNode)
			return shortestRoute;

		for (auto &neighbor : g.getNode(u))
		{
			std::vector<std::pair<int64_t, int>> removedElements;
			int64_t newValue = shortestRoute[u].first + g.getNode(neighbor).getValue();
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
