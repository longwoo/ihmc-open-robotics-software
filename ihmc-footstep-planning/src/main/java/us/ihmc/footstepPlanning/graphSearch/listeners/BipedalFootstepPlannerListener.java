package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstanceNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;

import java.util.List;

public interface BipedalFootstepPlannerListener
{
   void addNode(FootstanceNode node, FootstanceNode previousNode);

   void rejectNode(FootstanceNode rejectedNode, BipedalFootstepPlannerNodeRejectionReason reason);

   void plannerFinished(List<FootstanceNode> plan);

   void reportLowestCostNodeList(List<FootstanceNode> plan);

   void tickAndUpdate();
}
