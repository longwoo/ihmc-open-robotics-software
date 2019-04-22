package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepNodeDataListMessage;
import controller_msgs.msg.dds.FootstepNodeDataMessage;
import controller_msgs.msg.dds.FootstepPlannerCellMessage;
import controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.log.LogTools;

import java.util.HashSet;
import java.util.List;

public class StagePlannerListener implements BipedalFootstepPlannerListener
{
   private final FootstepNodeSnapperReadOnly snapper;
   private final HashSet<PlannerCell> exploredCells = new HashSet<>();
   private final FootstepNodeDataListMessage intermediatePlanMessage = new FootstepNodeDataListMessage();

   public StagePlannerListener(FootstepNodeSnapperReadOnly snapper)
   {
      this.snapper = snapper;
   }

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
      synchronized (this)
      {
         Object<FootstepNodeDataMessage> nodeDataList = intermediatePlanMessage.getNodeData();
         nodeDataList.clear();

         for (int i = 0; i < plan.size(); i++)
         {
            FootstepNode node = plan.get(i);
            FootstepNodeDataMessage nodeData = nodeDataList.add();

            nodeData.setRobotSide(node.getRobotSide().toByte());
            nodeData.setXIndex(node.getXIndex());
            nodeData.setYIndex(node.getYIndex());

            FootstepNodeSnapData snapData = snapper.getSnapData(node);
            if(snapData == null || !snapData.getSnapTransform().containsNaN())
            {
               LogTools.error("Snapper returned invalid data");
               nodeDataList.clear();
               return;
            }

            nodeData.getSnapTranslation().set(snapData.getSnapTransform().getTranslationVector());
            nodeData.getSnapRotation().set(snapData.getSnapTransform().getRotationMatrix());
         }
      }
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

   void packFootstepNodeListMessage(FootstepNodeDataListMessage intermediatePlanMessage)
   {
      synchronized (this)
      {
         Object<FootstepNodeDataMessage> completeIntermediatePlan = intermediatePlanMessage.getNodeData();
         Object<FootstepNodeDataMessage> partialIntermediatePlan = this.intermediatePlanMessage.getNodeData();

         for (int i = 0; i < partialIntermediatePlan.size(); i++)
         {
            completeIntermediatePlan.add().set(partialIntermediatePlan.get(i));
         }
      }
   }
}
