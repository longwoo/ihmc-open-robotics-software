package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstanceNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.HashSet;

public class ParameterBasedNodeExpansion implements FootstepNodeExpansion
{
   private final FootstepPlannerParameters parameters;

   public ParameterBasedNodeExpansion(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
   }

   @Override
   public HashSet<FootstanceNode> expandNode(FootstanceNode node)
   {
      HashSet<FootstanceNode> expansion = new HashSet<>();

      FootstepNode upcomingStanceNode = node.getStanceNode();
      FootstepNode upcomingSwingNode = node.getSwingNode();
      RobotSide nextSide = upcomingSwingNode.getRobotSide();

      for (double x = parameters.getMinimumStepLength(); x < parameters.getMaximumStepReach(); x += FootstepNode.gridSizeXY)
      {
         for (double y = parameters.getMinimumStepWidth(); y < parameters.getMaximumStepWidth(); y += FootstepNode.gridSizeXY)
         {
            if (Math.abs(x) <= parameters.getMinXClearanceFromStance() && Math.abs(y) <= parameters.getMinYClearanceFromStance())
            {
               continue;
            }

            for (double yaw = parameters.getMinimumStepYaw(); yaw < parameters.getMaximumStepYaw(); yaw += FootstepNode.gridSizeYaw)
            {
               FootstepNode touchdownNode = constructNodeInPreviousNodeFrame(x, nextSide.negateIfRightSide(y), nextSide.negateIfRightSide(yaw), upcomingStanceNode);
               expansion.add(new FootstanceNode(touchdownNode, upcomingStanceNode));
            }
         }
      }

      return expansion;
   }

   private static FootstepNode constructNodeInPreviousNodeFrame(double stepLength, double stepWidth, double stepYaw, FootstepNode node)
   {
      Vector2D footstep = new Vector2D(stepLength, stepWidth);
      AxisAngle rotation = new AxisAngle(node.getYaw(), 0.0, 0.0);
      rotation.transform(footstep);

      return new FootstepNode(node.getX() + footstep.getX(), node.getY() + footstep.getY(), stepYaw + node.getYaw(), node.getRobotSide().getOppositeSide());
   }
}
