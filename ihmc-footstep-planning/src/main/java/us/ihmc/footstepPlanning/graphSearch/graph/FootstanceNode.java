package us.ihmc.footstepPlanning.graphSearch.graph;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootstanceNode
{
   private final SideDependentList<FootstepNode> footstepNodes = new SideDependentList<>();
   private final RobotSide upcomingSwingSide;
   private final Pose2D midStancePose = new Pose2D();

   private final int hashCode;

   public FootstanceNode(FootstepNode upcomingStanceNode, FootstepNode upcomingSwingNode)
   {
      upcomingSwingSide = upcomingSwingNode.getRobotSide();
      footstepNodes.put(upcomingSwingSide, upcomingSwingNode);
      footstepNodes.put(upcomingSwingSide.getOppositeSide(), upcomingStanceNode);

      midStancePose.setX(0.5 * (upcomingStanceNode.getX() + upcomingStanceNode.getX()));
      midStancePose.setY(0.5 * (upcomingStanceNode.getY() + upcomingSwingNode.getY()));
      midStancePose.setYaw(AngleTools.computeAngleAverage(upcomingStanceNode.getYaw(), upcomingSwingNode.getYaw()));

      this.hashCode = computeHashCode();
   }

   public FootstepNode getStanceNode()
   {
      return footstepNodes.get(upcomingSwingSide.getOppositeSide());
   }

   public FootstepNode getSwingNode()
   {
      return footstepNodes.get(upcomingSwingSide);
   }

   public RobotSide getUpcomingSwingSide()
   {
      return upcomingSwingSide;
   }

   public Pose2D getMidStancePose()
   {
      return midStancePose;
   }

   private int computeHashCode()
   {
      int prime = 31;
      int result = 1;
      result += prime * getStanceNode().hashCode();
      result += prime * getSwingNode().hashCode();
      return result;
   }

   @Override
   public int hashCode()
   {
      return hashCode;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      FootstanceNode other = (FootstanceNode) obj;
      if(!other.getStanceNode().equals(this.getStanceNode()))
         return false;
      if(!other.getSwingNode().equals(this.getSwingNode()))
         return false;
      return true;
   }

   public FootstanceNode createChild(FootstepNode nextStep)
   {
      return new FootstanceNode(nextStep, getStanceNode());
   }
}
