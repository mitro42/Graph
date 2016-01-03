#ifndef MITRO_GRAPH_DEPTH_FIRST_SEARCH_H
#define  MITRO_GRAPH_DEPTH_FIRST_SEARCH_H

#include "Graph.h"
#include "gsl.h"

#include <vector>

namespace graph_algorithm_capture
{
	struct DepthFirstSearchState
	{		
		std::vector<int> finishedNodes;
		std::vector<gsl::not_null<const GraphEdge*>> currentPath;
	};

	std::vector<DepthFirstSearchState> depthFirstSearchCaptureStates(Graph &g, int startNode);
} //namespace graph_algorithm_capture

#endif // MITRO_GRAPH_DEPTH_FIRST_SEARCH_H
