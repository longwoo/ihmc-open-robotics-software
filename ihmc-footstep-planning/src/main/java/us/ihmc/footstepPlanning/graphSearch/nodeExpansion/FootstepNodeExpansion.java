package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstanceNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

import java.util.HashSet;

public interface FootstepNodeExpansion
{
   HashSet<FootstanceNode> expandNode(FootstanceNode node);
}
