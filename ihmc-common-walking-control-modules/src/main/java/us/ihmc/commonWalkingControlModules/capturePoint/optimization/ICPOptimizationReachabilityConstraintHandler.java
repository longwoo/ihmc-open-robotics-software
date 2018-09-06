package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.algorithms.FrameConvexPolygonWithLineIntersector2d;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFrameLineSegment2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoInteger;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class ICPOptimizationReachabilityConstraintHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final int numberOfVertices = 5;

   private final List<Footstep> upcomingFootsteps;

   private final SideDependentList<List<YoFramePoint2D>> reachabilityVertices = new SideDependentList<>();
   private final SideDependentList<YoFrameConvexPolygon2D> reachabilityPolygons = new SideDependentList<>();

   private final SideDependentList<List<YoFramePoint2D>> adjustmentVertices = new SideDependentList<>();
   private final SideDependentList<PoseReferenceFrame> stepFrames = new SideDependentList<>();
   private final SideDependentList<YoFrameConvexPolygon2D> adjustmentPolygons = new SideDependentList<>();

   private final FixedFrameConvexPolygon2DBasics adjustmentPolygon = new FrameConvexPolygon2D(worldFrame);
   private final FixedFrameConvexPolygon2DBasics reachabilityPolygon = new FrameConvexPolygon2D(worldFrame);

   private final YoFrameConvexPolygon2D contractedReachabilityPolygon;
   private final YoFrameLineSegment2D motionLimitLine;
   private final YoFrameLineSegment2D adjustmentLineSegment;

   private final DoubleProvider lengthLimit;
   private final DoubleProvider lengthBackLimit;
   private final DoubleProvider innerLimit;
   private final DoubleProvider outerLimit;

   private final DoubleProvider forwardAdjustmentLimit;
   private final DoubleProvider backwardAdjustmentLimit;
   private final DoubleProvider innerAdjustmentLimit;
   private final DoubleProvider outerAdjustmentLimit;

   private final ConvexPolygonTools polygonTools = new ConvexPolygonTools();

   public ICPOptimizationReachabilityConstraintHandler(BipedSupportPolygons bipedSupportPolygons, SteppingParameters steppingParameters, String yoNamePrefix,
                                                       boolean visualize, List<Footstep> upcomingFootsteps, YoVariableRegistry registry,
                                                       YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.upcomingFootsteps = upcomingFootsteps;

      lengthLimit = new DoubleParameter(yoNamePrefix + "MaxReachabilityLength", registry, steppingParameters.getMaxStepLength());
      lengthBackLimit = new DoubleParameter(yoNamePrefix + "MaxReachabilityBackwardLength", registry, steppingParameters.getMaxBackwardStepLength());
      innerLimit = new DoubleParameter(yoNamePrefix + "MaxReachabilityWidth", registry, steppingParameters.getMinStepWidth());
      outerLimit = new DoubleParameter(yoNamePrefix + "MinReachabilityWidth", registry, steppingParameters.getMaxStepWidth());

      forwardAdjustmentLimit = new DoubleParameter(yoNamePrefix + "ForwardAdjustmentLimit", registry, 0.1);
      backwardAdjustmentLimit = new DoubleParameter(yoNamePrefix + "BackwardAdjustmentLimit", registry, 0.1);
      innerAdjustmentLimit = new DoubleParameter(yoNamePrefix + "InnerAdjustmentLimit", registry, 0.1);
      outerAdjustmentLimit = new DoubleParameter(yoNamePrefix + "OuterAdjustmentLimit", registry, 0.1);

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame supportSoleFrame = bipedSupportPolygons.getSoleZUpFrames().get(robotSide);

         YoInteger yoNumberOfReachabilityVertices = new YoInteger(robotSide.getLowerCaseName() + "NumberOfReachabilityVertices", registry);
         yoNumberOfReachabilityVertices.set(numberOfVertices);

         String prefix = yoNamePrefix + robotSide.getSideNameFirstLetter();

         List<YoFramePoint2D> reachabilityVertices = new ArrayList<>();
         for (int i = 0; i < yoNumberOfReachabilityVertices.getValue(); i++)
         {
            YoFramePoint2D vertex = new YoFramePoint2D(prefix + "ReachabilityVertex" + i, supportSoleFrame, registry);
            reachabilityVertices.add(vertex);
         }
         YoFrameConvexPolygon2D reachabilityPolygon = new YoFrameConvexPolygon2D(reachabilityVertices, yoNumberOfReachabilityVertices, supportSoleFrame);

         this.reachabilityVertices.put(robotSide, reachabilityVertices);
         this.reachabilityPolygons.put(robotSide, reachabilityPolygon);

         ReferenceFrame adjustmentFrame = new PoseReferenceFrame(prefix + "AdjustmentFrame", worldFrame);

         YoInteger yoNumberOfAdjustmentVertices = new YoInteger(robotSide.getLowerCaseName() + "NumberOfAdjustmentVertices", registry);
         yoNumberOfAdjustmentVertices.set(4);

         List<YoFramePoint2D> adjustmentVertices = new ArrayList<>();
         for (int i = 0; i < 4; i++)
         {
            YoFramePoint2D adjustmentVertex = new YoFramePoint2D(prefix + "AdjustmentVertex" + i, adjustmentFrame, registry);
            adjustmentVertices.add(adjustmentVertex);
         }

         YoFrameConvexPolygon2D adjustmentPolygon = new YoFrameConvexPolygon2D(adjustmentVertices, yoNumberOfAdjustmentVertices, adjustmentFrame);

         this.adjustmentVertices.put(robotSide, adjustmentVertices);
         this.adjustmentPolygons.put(robotSide, adjustmentPolygon);

      }

      contractedReachabilityPolygon = new YoFrameConvexPolygon2D(yoNamePrefix + "ReachabilityRegion", "", worldFrame, 12, registry);
      motionLimitLine = new YoFrameLineSegment2D(yoNamePrefix + "AdjustmentThresholdSegment", "", worldFrame, registry);
      adjustmentLineSegment = new YoFrameLineSegment2D(yoNamePrefix + "AdjustmentLineSegment", "", worldFrame, registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon reachabilityGraphic = new YoArtifactPolygon("ReachabilityRegionViz", contractedReachabilityPolygon, Color.BLUE, false);
         YoArtifactLineSegment2d adjustmentGraphic = new YoArtifactLineSegment2d("AdjustmentViz", adjustmentLineSegment, Color.GREEN);
         YoArtifactLineSegment2d adjustmentClippingGraphic = new YoArtifactLineSegment2d("AdjustmentClippingViz", motionLimitLine, Color.RED);

         reachabilityGraphic.setVisible(visualize);
         adjustmentGraphic.setVisible(visualize);
         adjustmentClippingGraphic.setVisible(visualize);

         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), reachabilityGraphic);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), adjustmentGraphic);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), adjustmentClippingGraphic);
      }
   }

   /**
    * Initializes the reachability constraint for the double support state.
    * This is the constraint that determines where the robot can step to kinematically.
    * It is a simple convex rectangle determined by the stepping parameters.
    */
   public FrameConvexPolygon2DReadOnly initializeReachabilityConstraintForDoubleSupport()
   {
      contractedReachabilityPolygon.clear();
      motionLimitLine.setToNaN();
      adjustmentLineSegment.setToNaN();

      return null;
   }

   /**
    * Initializes the reachability constraint for the single support state.
    * This is the constraint that determines where the robot can step to kinematically.
    * It is a simple convex rectangle determined by the stepping parameters.
    *
    * @param  supportSide the current support side of the robot
    */
   public FrameConvexPolygon2DReadOnly initializeReachabilityConstraintForSingleSupport(RobotSide supportSide)
   {
      FrameConvexPolygon2DReadOnly reachabilityPolygon = getReachabilityPolygon(supportSide);
      FrameConvexPolygon2DReadOnly adjustmentPolygon = getAdjustmentPolygon(supportSide.getOppositeSide());

      contractedReachabilityPolygon.checkReferenceFrameMatch(reachabilityPolygon);
      contractedReachabilityPolygon.checkReferenceFrameMatch(adjustmentPolygon);

      polygonTools.computeIntersectionOfPolygons(reachabilityPolygon, adjustmentPolygon, contractedReachabilityPolygon);

      return contractedReachabilityPolygon;
   }

   private FrameConvexPolygon2DReadOnly getReachabilityPolygon(RobotSide supportSide)
   {
      List<YoFramePoint2D> vertices = reachabilityVertices.get(supportSide);
      YoFrameConvexPolygon2D polygon = reachabilityPolygons.get(supportSide);

      // create an ellipsoid around the center of the forward and backward reachable limits
      double xRadius = 0.5 * (lengthLimit.getValue() + lengthBackLimit.getValue());
      double yRadius = outerLimit.getValue() - innerLimit.getValue();
      double centerX = lengthLimit.getValue() - xRadius;
      double centerY = innerLimit.getValue();

      // compute the vertices on the edge of the ellipsoid
      for (int vertexIdx = 0; vertexIdx < vertices.size(); vertexIdx++)
      {
         double angle = Math.PI * vertexIdx / (vertices.size() - 1);
         double x = centerX + xRadius * Math.cos(angle);
         double y = centerY + yRadius * Math.sin(angle);
         vertices.get(vertexIdx).set(x, supportSide.negateIfLeftSide(y));
      }

      polygon.notifyVerticesChanged();
      polygon.update();

      reachabilityPolygon.setMatchingFrame(polygon, false);
      return reachabilityPolygon;
   }

   private final FramePose3D footstepPose = new FramePose3D();

   private FrameConvexPolygon2DReadOnly getAdjustmentPolygon(RobotSide swingSide)
   {
      Footstep footstep = upcomingFootsteps.get(0);
      if (!footstep.getRobotSide().equals(swingSide))
         throw new RuntimeException("Somehow initializing the wrong side!");

      footstep.getPose(footstepPose);
      footstepPose.changeFrame(worldFrame);
      stepFrames.get(swingSide).setPoseAndUpdate(footstepPose);

      List<YoFramePoint2D> vertices = adjustmentVertices.get(swingSide);
      YoFrameConvexPolygon2D polygon = adjustmentPolygons.get(swingSide);

      vertices.get(0).set(forwardAdjustmentLimit.getValue(), swingSide.negateIfRightSide(outerAdjustmentLimit.getValue()));
      vertices.get(1).set(forwardAdjustmentLimit.getValue(), swingSide.negateIfLeftSide(innerAdjustmentLimit.getValue()));
      vertices.get(2).set(-backwardAdjustmentLimit.getValue(), swingSide.negateIfRightSide(outerAdjustmentLimit.getValue()));
      vertices.get(3).set(-backwardAdjustmentLimit.getValue(), swingSide.negateIfLeftSide(innerAdjustmentLimit.getValue()));

      polygon.notifyVerticesChanged();
      polygon.update();

      adjustmentPolygon.setMatchingFrame(polygon, false);
      return adjustmentPolygon;
   }

   private final FramePoint2D adjustedLocation = new FramePoint2D();
   private final FramePoint2D referenceLocation = new FramePoint2D();
   private final FrameVector2D adjustmentDirection = new FrameVector2D();
   private final FrameLine2D motionLine = new FrameLine2D();

   private final FrameConvexPolygonWithLineIntersector2d lineIntersector2d = new FrameConvexPolygonWithLineIntersector2d();

   public void updateReachabilityBasedOnAdjustment(Footstep upcomingFootstep, FixedFramePoint2DBasics footstepSolution, boolean wasAdjusted)
   {
      if (!wasAdjusted)
         return;

      upcomingFootstep.getPosition2d(referenceLocation);
      adjustedLocation.setIncludingFrame(footstepSolution);
      referenceLocation.changeFrame(worldFrame);
      adjustedLocation.changeFrame(worldFrame);

      adjustmentDirection.sub(adjustedLocation, referenceLocation);
      EuclidGeometryTools.perpendicularVector2D(adjustmentDirection, adjustmentDirection);

      motionLine.setPoint(adjustedLocation);
      motionLine.setDirection(adjustmentDirection);

      contractedReachabilityPolygon.update();
      ConvexPolygonTools.cutPolygonWithLine(motionLine, contractedReachabilityPolygon, lineIntersector2d, RobotSide.LEFT);

      adjustmentLineSegment.set(referenceLocation, adjustedLocation);
      motionLimitLine.set(lineIntersector2d.getIntersectionPointOne(), lineIntersector2d.getIntersectionPointTwo());
   }

   /**
    * Get the polygon that describes the reachable region for the step position.
    */
   public FrameConvexPolygon2DReadOnly updateReachabilityConstraint()
   {
      contractedReachabilityPolygon.update();

      return contractedReachabilityPolygon;
   }
}
