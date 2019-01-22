package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstanceNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class EuclideanDistanceHeuristics extends CostToGoHeuristics
{
   public EuclideanDistanceHeuristics(DoubleProvider weight)
   {
      super(weight);
   }

   @Override
   protected double computeHeuristics(FootstanceNode node, FootstanceNode goalNode)
   {
      return node.getMidStancePose().getPositionDistance(goalNode.getMidStancePose());
   }
}
