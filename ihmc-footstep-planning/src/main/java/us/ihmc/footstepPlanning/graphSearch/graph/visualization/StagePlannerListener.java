package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepPlannerCellMessage;
import controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;

import java.util.HashSet;
import java.util.List;

public class StagePlannerListener implements BipedalFootstepPlannerListener
{
   private final HashSet<PlannerCell> exploredCells = new HashSet<>();

   @Override
   public void addNode(FootstepNode node, FootstepNode previousNode)
   {
      synchronized (this)
      {
         if (previousNode == null)
            exploredCells.clear();
         exploredCells.add(new PlannerCell(node.getXIndex(), node.getYIndex()));
      }
   }

   @Override
   public void reportLowestCostNodeList(List<FootstepNode> plan)
   {
   }

   @Override
   public void rejectNode(FootstepNode rejectedNode, FootstepNode parentNode, BipedalFootstepPlannerNodeRejectionReason reason)
   {
   }

   @Override
   public void tickAndUpdate()
   {
   }

   @Override
   public void plannerFinished(List<FootstepNode> plan)
   {
   }

   void packOccupancyMapMessage(FootstepPlannerOccupancyMapMessage message)
   {
      synchronized (this)
      {
         for (PlannerCell plannerCell : exploredCells)
         {
            FootstepPlannerCellMessage cell = message.getOccupiedCells().add();
            cell.setXIndex(plannerCell.xIndex);
            cell.setYIndex(plannerCell.yIndex);
         }

         exploredCells.clear();
      }
   }
}
