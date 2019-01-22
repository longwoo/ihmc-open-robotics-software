package us.ihmc.footstepPlanning.graphSearch.heuristics;

import java.util.Comparator;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstanceNode;
import us.ihmc.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.robotSide.SideDependentList;

public class NodeComparator implements Comparator<FootstanceNode>
{
   private final FootstepGraph graph;
   private final FootstanceNode goalNode;
   private final CostToGoHeuristics heuristics;

   public NodeComparator(FootstepGraph graph, FootstanceNode goalNode, CostToGoHeuristics heuristics)
   {
      this.graph = graph;
      this.goalNode = goalNode;
      this.heuristics = heuristics;
   }

   @Override
   public int compare(FootstanceNode o1, FootstanceNode o2)
   {
      double cost1 = graph.getCostFromStart(o1) + heuristics.compute(o1, goalNode);
      double cost2 = graph.getCostFromStart(o2) + heuristics.compute(o2, goalNode);
      if (cost1 == cost2) return 0;
      return cost1 < cost2 ? -1 : 1;
   }
}
