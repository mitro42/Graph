#ifndef MITRO_UTIL_GRAPH_H
#define MITRO_UTIL_GRAPH_H
#include "UnionFind.h"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

class Graph;

struct GraphEdge
{
    double weight;
    int from;
    int to;
    GraphEdge(double weight, int from, int to) : weight(weight), from(from), to(to) {}
    bool operator<(const GraphEdge& other) const { return std::make_tuple(weight, from, to) < std::make_tuple(other.weight, other.from, other.to); }
    int otherEnd(int oneEnd) const
    { 
        if (oneEnd != from && oneEnd != to)
            throw "Invalid end";
        return (oneEnd == from) ? to : from;
    }
};


class GraphNode //: public GraphNode
{
public:
	friend class Graph;

    GraphNode(double weight = 0.0): weight(weight) {};
	~GraphNode() = default;
	//explicit GraphNode( int l ): GraphNode( l ) {};    

	GraphNode &operator=(const GraphNode &other) = delete;
	GraphNode(const GraphNode &other) = default;

    void setWeight(double w) { weight = w; }
    double getWeight() const { return weight; }

    //std::vector<std::shared_ptr<GraphEdge>>::const_iterator begin() const { return edges.begin(); }
    //std::vector<std::shared_ptr<GraphEdge>>::const_iterator end() const { return edges.end(); }

    std::vector<std::shared_ptr<GraphEdge>>::iterator begin() { return edges.begin(); }
    std::vector<std::shared_ptr<GraphEdge>>::iterator end() { return edges.end(); }

    //int getNeighbor(int idx) const { return neighbors[idx]; }

    //double getEdgeWeight(int idx) const { return edgeWeights[idx]; }
   // double getEdgeWeight(const std::vector<int>::iterator &it) const { return *(edgeWeights.begin() + (it - neighbors.begin())); }

    //void setEdgeWeight(int idx, double w) { edgeWeights[idx] = w; }
    //void setEdgeWeight(const std::vector<int>::iterator &it, double w) { (*(edgeWeights.begin() + (it - neighbors.begin()))) = w; }

    int getNeighborCount() const { return int(edges.size()); }
private:
    void addEdge(std::shared_ptr<GraphEdge> edge) { edges.push_back(edge); }
    void removeEdge(std::shared_ptr<GraphEdge> edge) { edges.erase(std::remove(edges.begin(), edges.end(), edge), edges.end()); }
	//void removeNeighbor(int to);
	//void addNeighbor(int to, double weight = 0.0);

    std::vector<std::shared_ptr<GraphEdge>> edges;
	double weight;
};



class Graph
{
public:
    friend class GraphNode;
    friend std::istream &operator>>(std::istream &is, Graph &g);
    friend std::ostream &operator<<(std::ostream &os, const Graph &g);
	explicit Graph(bool dir) : directed(dir) {}
	~Graph() = default;

	Graph &operator=(const Graph &other) = delete;
	Graph(const Graph &other) = delete;

	int addNode(double weight = 0.0);
	//virtual void removeNode( int toRemove );

	void addEdge(int from, int to, double weight = 0.0);
	void removeEdge(int from, int to);
    bool hasEdge(int from, int to);
    std::vector<std::shared_ptr<GraphEdge>> getEdge(int from, int to);

	//std::vector<std::shared_ptr<GraphNode>>::const_iterator begin() const { return nodes.begin(); }
	//std::vector<std::shared_ptr<GraphNode>>::const_iterator end() const { return nodes.end(); }

    std::vector<std::shared_ptr<GraphNode>>::iterator begin() { return nodes.begin(); }
    std::vector<std::shared_ptr<GraphNode>>::iterator end() { return nodes.end(); }

	void resize(int newNodes);
	GraphNode &getNode(int idx) { return *nodes[idx]; }
	const GraphNode &getNode(int idx) const { return *nodes[idx]; }
	int getNodeCount() const { return int(nodes.size()); }

    bool isDirected() const { return directed; }

    bool hasWeightedNodes() const { return weightedNodes; }
    void setWeightedNodes(bool weighted) { weightedNodes = weighted; }

    bool hasWeightedEdges() const { return weightedEdges; }
    void setWeightedEdges(bool weighted) { weightedEdges = weighted; }

    void clear(bool newDirected);
private:
    bool directed;
    bool weightedNodes = false;
    bool weightedEdges = true;

    std::vector<std::shared_ptr<GraphNode>> nodes;
    std::vector<std::shared_ptr<GraphEdge>> edges;
};


std::istream &operator>>(std::istream &is, Graph &g);
std::ostream &operator<<(std::ostream &os, const Graph &g);


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Graph algorithms
//////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::pair<double, std::shared_ptr<GraphEdge>>> nodeWeightDijkstra(Graph &g, int startNode, int endNode);
std::vector<std::pair<double, std::shared_ptr<GraphEdge>>> edgeWeightDijkstra(Graph &g, int startNode, int endNode);

std::vector<GraphEdge> mstPrim(Graph &g, int startNode);
std::vector<GraphEdge> mstKruskal(Graph &g);

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Algorithms with all intermediate states captured
//////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace graph_algorithm_capture
{

struct ShortestPathEdgeWeightDijkstraState
{
    std::vector<std::pair<double, std::shared_ptr<GraphEdge>>> path;
    std::set<std::pair<double, int>> openNodes; // discovered but not fully processed
    std::set<std::pair<double, int>> closedNodes; //  fully processed
    std::vector<std::shared_ptr<GraphEdge>> processedEdges; // processed, not part of any of the minimal paths
    int inspectedNode;
    std::shared_ptr<GraphEdge> inspectedEdge;
    std::string description;

    ShortestPathEdgeWeightDijkstraState(const std::vector<std::pair<double, std::shared_ptr<GraphEdge>>> &path,
        const std::set<std::pair<double, int>> &openNodes,
        const std::set<std::pair<double, int>> &closedNodes,
        const std::vector<std::shared_ptr<GraphEdge>> &processedEdges,
        int inspectedNode,
        std::shared_ptr<GraphEdge> inspectedEdge,
        const std::string &description) :
        path(path),
        openNodes(openNodes),
        closedNodes(closedNodes),
        processedEdges(processedEdges),
        inspectedNode(inspectedNode),
        inspectedEdge(inspectedEdge),
        description(description)
    {}
};

std::vector<ShortestPathEdgeWeightDijkstraState>
edgeWeightDijkstraCaptureStates(Graph &g, int startNode, int endNode);



struct MstKruskalState
{
    std::vector<GraphEdge> mst;
    std::vector<GraphEdge> edges;
    UnionFind uf;
    GraphEdge inspectedEdge;
    MstKruskalState(const std::vector<GraphEdge> mst, const std::vector<GraphEdge> &edges, const UnionFind &uf, const GraphEdge &inspectedEdge) :
        mst(mst),
        edges(edges),
        uf(uf),
        inspectedEdge(inspectedEdge)
    {}
};

std::vector<MstKruskalState> mstKruskalCaptureStates(Graph &g);



struct MstPrimState
{
    std::vector<GraphEdge> mst;
    std::vector<bool> visited;
    std::vector<GraphEdge> inspectedEdges;
    std::set<GraphEdge> edges;
    MstPrimState(const std::vector<GraphEdge> &mst, const std::vector<bool> &visited, const std::vector<GraphEdge> &inspectedEdges, const std::set<GraphEdge> &edges) :
        mst(mst), 
        visited(visited), 
        inspectedEdges(inspectedEdges),
        edges(edges) 
    {}
};

std::vector<MstPrimState> mstPrimCaptureStates(Graph &g, int startNode);
} // graph_algorithm_capture

#endif // MITRO_UTIL_GRAPH_H
