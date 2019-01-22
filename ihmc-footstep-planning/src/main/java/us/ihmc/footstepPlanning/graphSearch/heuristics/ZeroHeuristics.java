package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstanceNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.yoVariables.variable.YoDouble;

public class ZeroHeuristics extends CostToGoHeuristics
{
   public ZeroHeuristics(YoDouble weight)
   {
      super(weight);
   }

   @Override
   protected double computeHeuristics(FootstanceNode node, FootstanceNode goalNode)
   {
      return 0.0;
   }
}
