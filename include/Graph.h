#ifndef MITRO_GRAPH_H
#define MITRO_GRAPH_H

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <gsl.h>

class Graph;

struct GraphEdge
{
    double weight = 0;
    int from = -1;
    int to = -1;
    GraphEdge(double weight, int from, int to) : weight(weight), from(from), to(to) {}
    bool operator<(const GraphEdge& other) const { return std::make_tuple(weight, from, to) < std::make_tuple(other.weight, other.from, other.to); }
    int otherEnd(int oneEnd) const
    { 
        if (oneEnd != from && oneEnd != to)
            throw "Invalid end";
        return (oneEnd == from) ? to : from;
    }
};

struct EdgePtrCompare
{
    bool operator() (const GraphEdge* lhs, const GraphEdge* rhs)
    {
        return *lhs < *rhs;
    }
};

typedef std::set<gsl::not_null<GraphEdge*>, EdgePtrCompare> EdgePtrSet;
typedef std::vector<GraphEdge*> EdgePtrVector;



template <class ValueType, class WrappedValueType, bool is_const_iterator = false>
class basic_graph_iterator
{
    using vector_in_creator = std::vector<WrappedValueType>;
    using iterator_in_creator = typename std::conditional<is_const_iterator,
        typename vector_in_creator::const_iterator,
        typename vector_in_creator::iterator
    >::type;
public:
    using iterator_category = std::random_access_iterator_tag;
    using difference_type = typename iterator_in_creator::difference_type;
    using value_type = ValueType;
    using pointer = typename std::conditional<is_const_iterator, const value_type*, value_type*>::type;
    using reference = typename std::conditional<is_const_iterator, const value_type&, value_type&>::type;

    basic_graph_iterator(const basic_graph_iterator& other) = default;
    basic_graph_iterator& operator=(const basic_graph_iterator& other) = default;
    explicit basic_graph_iterator(const iterator_in_creator &it) : it(it) {}

    friend bool operator==(const basic_graph_iterator& l, const basic_graph_iterator& r)  { return l.it == r.it; }
    friend bool operator!=(const basic_graph_iterator& l, const basic_graph_iterator& r)  { return l.it != r.it; }
    friend bool operator<(const basic_graph_iterator& l, const basic_graph_iterator& r)   { return l.it < r.it; }

    reference operator*() { return **it; }
    pointer  operator->() { return *it; }

    basic_graph_iterator& operator++()
    {
        ++it;
        return *this;
    }

    basic_graph_iterator operator++(int)
    {
        basic_graph_iterator tmp(*this);
        ++it;
        return tmp;
    }

private:
    iterator_in_creator it;
};



class GraphNode 
{
public:
	friend class Graph;
    using node_iterator = basic_graph_iterator<GraphEdge, gsl::not_null<GraphEdge*>, false>;
    using const_node_iterator = basic_graph_iterator<GraphEdge, gsl::not_null<GraphEdge*>, true>;

    GraphNode(double weight = 0.0): weight(weight) {};
	~GraphNode() = default;

	GraphNode &operator=(const GraphNode &other) = delete;
	GraphNode(const GraphNode &other) = default;

    void setWeight(double w) { weight = w; }
    double getWeight() const { return weight; }

    const_node_iterator begin() const { return const_node_iterator{edges.begin()}; }
    const_node_iterator end() const { return const_node_iterator{edges.end()}; }

    node_iterator begin() { return node_iterator{ edges.begin() }; }
    node_iterator end() { return node_iterator{ edges.end() }; }

    int getNeighborCount() const { return int(edges.size()); }
private:
    void addEdge(gsl::not_null<GraphEdge*> edge) { edges.push_back(edge); }
    void removeEdge(gsl::not_null<GraphEdge*> edge) { edges.erase(std::remove(edges.begin(), edges.end(), edge), edges.end()); }

    std::vector<gsl::not_null<GraphEdge*>> edges;
	double weight;
};




class Graph
{
public:
    friend class GraphNode;
    friend std::istream &operator>>(std::istream &is, Graph &g);
    friend std::ostream &operator<<(std::ostream &os, const Graph &g);

    using node_iterator = basic_graph_iterator<GraphNode, std::unique_ptr<GraphNode>, false>;
    using const_node_iterator = basic_graph_iterator<GraphNode, std::unique_ptr<GraphNode>, true>;
    using edge_iterator = basic_graph_iterator<GraphEdge, std::unique_ptr<GraphEdge>, false>;
    using const_edge_iterator = basic_graph_iterator<GraphEdge, std::unique_ptr<GraphEdge>, true>;

    explicit Graph(bool dir) : directed(dir) {}
	~Graph() = default;

	Graph &operator=(const Graph &other) = delete;
	Graph(const Graph &other) = delete;

	int addNode(double weight = 0.0);

	void addEdge(int from, int to, double weight = 0.0);
	void removeEdge(int from, int to);
    bool hasEdge(int from, int to);

    const_node_iterator begin() const { return const_node_iterator{nodes.begin()}; }
    const_node_iterator end() const { return const_node_iterator{nodes.end()}; }

    node_iterator begin() { return node_iterator{nodes.begin()}; }
    node_iterator end() { return node_iterator{nodes.end()}; }

    const_edge_iterator edges_begin() const { return const_edge_iterator{edges.begin()}; }
    const_edge_iterator edges_end() const { return const_edge_iterator{edges.end()}; }

    edge_iterator edges_begin() { return edge_iterator{edges.begin()}; }
    edge_iterator edges_end() { return edge_iterator{edges.end()}; }

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
    using node_vector = std::vector<std::unique_ptr<GraphNode>>;
    using edge_vector = std::vector<std::unique_ptr<GraphEdge>>;
    node_vector nodes;
    edge_vector edges;
};


std::istream &operator>>(std::istream &is, Graph &g);
std::ostream &operator<<(std::ostream &os, const Graph &g);

#endif // MITRO_UTIL_GRAPH_H
