#ifndef MITRO_UTIL_GRAPH_H
#define MITRO_UTIL_GRAPH_H

#include <cstdint>
#include <vector>

class Graph;

class GraphNode //: public GraphNode
{
public:
	friend class Graph;

	GraphNode() = default;
	~GraphNode() = default;
	//explicit GraphNode( int l ): GraphNode( l ) {};    

	GraphNode &operator=(const GraphNode &other) = delete;
	GraphNode(const GraphNode &other) = default;

	virtual void setValue(int64_t v) { value = v; }
	virtual int64_t getValue() const { return value; }

	std::vector<int>::const_iterator begin() const { return neighbors.begin(); }
	std::vector<int>::const_iterator end() const { return neighbors.end(); }

	std::vector<int>::iterator begin() { return neighbors.begin(); }
	std::vector<int>::iterator end() { return neighbors.end(); }
private:
	void removeNeighbor(int to);
	void addNeighbor(int to);

	std::vector<int> neighbors;
	int64_t value;
};



class Graph
{
public:
	explicit Graph(bool dir) : directed(dir) {}
	virtual ~Graph() = default;

	Graph &operator=(const Graph &other) = delete;
	Graph(const Graph &other) = delete;

	int addNode();
	//virtual void removeNode( int toRemove );

	virtual void addEdge(int from, int to);
	virtual void removeEdge(int from, int to);

	std::vector<GraphNode>::const_iterator begin() const { return nodes.begin(); }
	std::vector<GraphNode>::const_iterator end() const { return nodes.end(); }

	void resize(size_t newNodes);
	GraphNode &getNode(int idx) { return nodes[idx]; }
	const GraphNode &getNode(int idx) const { return nodes[idx]; }
	int getNodeCount() const { return nodes.size(); }
private:
	bool directed;

	std::vector<GraphNode> nodes;
};


std::vector<std::pair<int64_t, int>> findMinimalPathDijkstra(const Graph &g, int startNode, int endNode);

#endif // MITRO_UTIL_GRAPH_H
