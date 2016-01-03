#include "depthFirstSearch.h"

namespace graph_algorithm_capture
{

	namespace detail
	{
		std::vector<gsl::not_null<const GraphEdge*>> 
			convertPath(
				const Graph &g, 
				const std::vector<std::pair<int, int>> &path)
		{
			std::vector<gsl::not_null<const GraphEdge*>> ret;
			for (const auto &p : path)
			{
				if (p.second != -1)
					ret.push_back(&g.getNode(p.first).getEdge(p.second));
			}
			return ret;
		}
	}

std::vector<DepthFirstSearchState> depthFirstSearchCaptureStates(Graph & g, int startNode)
{
	std::vector<DepthFirstSearchState> states;	
	std::vector<int> finished;
	std::vector<gsl::not_null<const GraphEdge*>> capturedPath;
	if (g.getNodeCount() == 0)
		return states;

	std::vector<std::pair<int, int>> path;
		
	path.push_back(std::make_pair(startNode, -1));
	while (!path.empty())
	{
		auto state = *path.rbegin();
		int nodeIdx = state.first;
		int edgeIdx = state.second;

		states.push_back({ finished, detail::convertPath(g, path) });
		path.pop_back();
		
		if (edgeIdx == -1)
		{
			finished.push_back(nodeIdx);
		}
		const auto &node = g.getNode(nodeIdx);
		while (edgeIdx < node.getEdgeCount() - 1)
		{
			edgeIdx++;
			int childIdx = node.getEdge(edgeIdx).otherEnd(nodeIdx);
			if (std::find(begin(finished), end(finished), childIdx) == end(finished))
			{
				path.push_back(std::make_pair(nodeIdx, edgeIdx));
				path.push_back(std::make_pair(childIdx, -1));
				break;
			}
		}
	}

	states.push_back({ finished, detail::convertPath(g, path) });
	return states;
}

}