package us.ihmc.footstepPlanning.graphSearch.graph.visualization;

import controller_msgs.msg.dds.FootstepPlannerCellMessage;
import controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.idl.IDLSequence.Object;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

public class MultiStagePlannerListener implements BipedalFootstepPlannerListener
{
   private static final long broadcastDtMillis = 1000;
   private final StatusMessageOutputManager statusMessageOutputManager;

   private final AtomicInteger planId = new AtomicInteger();
   private final FootstepPlannerOccupancyMapMessage occupancyMapMessage = new FootstepPlannerOccupancyMapMessage();
   private long lastBroadcastTime = -1;

   private final List<StagePlannerListener> listeners = new ArrayList<>();

   public MultiStagePlannerListener(StatusMessageOutputManager statusMessageOutputManager)
   {
      this.statusMessageOutputManager = statusMessageOutputManager;
   }

   public void addStagePlannerListener(StagePlannerListener listener)
   {
      listeners.add(listener);
   }

   public void setPlanId(int planId)
   {
      this.planId.set(planId);
   }

   @Override
   public void addNode(FootstepNode node, FootstepNode previousNode)
   {
      if(previousNode == null)
         lastBroadcastTime = System.currentTimeMillis();
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
      long currentTime = System.currentTimeMillis();

      if (lastBroadcastTime == -1)
         lastBroadcastTime = currentTime;

      if (currentTime - lastBroadcastTime < broadcastDtMillis)
         return;

      broadcastOccupancyMessage();
      lastBroadcastTime = currentTime;
   }

   @Override
   public void plannerFinished(List<FootstepNode> plan)
   {
      broadcastOccupancyMessage();
   }

   private void broadcastOccupancyMessage()
   {
      Object<FootstepPlannerCellMessage> occupiedCells = occupancyMapMessage.getOccupiedCells();
      occupiedCells.clear();

      for (StagePlannerListener listener : listeners)
         listener.packOccupancyMapMessage(occupancyMapMessage);

      if (!occupancyMapMessage.getOccupiedCells().isEmpty())
      {
         occupancyMapMessage.setSequenceId(planId.get());
         statusMessageOutputManager.reportStatusMessage(occupancyMapMessage);
      }
   }
}
