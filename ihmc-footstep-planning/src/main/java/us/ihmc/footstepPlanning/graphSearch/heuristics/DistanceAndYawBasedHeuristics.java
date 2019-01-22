package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstanceNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class DistanceAndYawBasedHeuristics extends CostToGoHeuristics
{
   private final FootstepPlannerParameters parameters;
   private final FootstepPlannerCostParameters costParameters;

   public DistanceAndYawBasedHeuristics(DoubleProvider weight, FootstepPlannerParameters parameters)
   {
      super(weight);
      this.parameters = parameters;
      this.costParameters = parameters.getCostParameters();
   }

   @Override
   protected double computeHeuristics(FootstanceNode node, FootstanceNode goalNode)
   {
      Point2D goalPoint = goalNode.getStanceNode().getOrComputeMidFootPoint(parameters.getIdealFootstepWidth());
      Point2DBasics nodeMidFootPoint = node.getMidStancePose().getPosition();
      double euclideanDistance = nodeMidFootPoint.distance(goalPoint);
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(node.getMidStancePose().getYaw(), goalNode.getMidStancePose().getYaw());
      double minSteps = euclideanDistance / parameters.getMaximumStepReach();

      return euclideanDistance + costParameters.getYawWeight() * Math.abs(yaw) + costParameters.getCostPerStep() * minSteps;
   }
}
