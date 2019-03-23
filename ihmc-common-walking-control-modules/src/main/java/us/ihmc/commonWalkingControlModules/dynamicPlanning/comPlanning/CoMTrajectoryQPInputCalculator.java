package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.List;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.sufficientlyLongTime;

public class CoMTrajectoryQPInputCalculator
{
   private final CoMTrajectoryPlannerIndexHandler indexHandler;
   private final DoubleProvider omega;

   private final DenseMatrix64F bezierMapMultiplier = new DenseMatrix64F(40, 40);
   private final DenseMatrix64F tempJ = new DenseMatrix64F(40, 40);

   public CoMTrajectoryQPInputCalculator(CoMTrajectoryPlannerIndexHandler indexHandler,
                                         DoubleProvider omega)
   {
      this.indexHandler = indexHandler;
      this.omega = omega;
   }

   public void computeBezierMapMultiplier(List<? extends ContactStateProvider> contactSequence)
   {
      int size = indexHandler.getNumberOfVRPWaypoints();

      bezierMapMultiplier.reshape(size, size);
      bezierMapMultiplier.zero();

      int waypointSetNumber = 0;
      for (int segmentNumber = 0; segmentNumber < contactSequence.size(); segmentNumber++)
      {
         if (!contactSequence.get(segmentNumber).getContactState().isLoadBearing())
            continue;

         double segmentDuration = contactSequence.get(segmentNumber).getTimeInterval().getDuration();
         CubicBezierCurve.computeMapToCurveBounds(4 * waypointSetNumber, 4 * waypointSetNumber, bezierMapMultiplier, segmentDuration);

         waypointSetNumber++;
      }
   }

   public void computeWorkWeightMatrix(List<? extends ContactStateProvider> contactSequence, double weight, DenseMatrix64F matrixToPack)
   {
      for (int segmentNumber = 0; segmentNumber < contactSequence.size(); segmentNumber++)
      {
         if (!contactSequence.get(segmentNumber).getContactState().isLoadBearing())
            continue;

         double segmentDuration = contactSequence.get(segmentNumber).getTimeInterval().getDuration();
         populateWorkWeightTermForSegment(segmentNumber, segmentDuration, omega.getValue(), weight, matrixToPack);
      }
   }

   public void computeJacobianToCoMCoefficients(DenseMatrix64F coefficientConstraintInv, DenseMatrix64F vrpBoundsJacobian, DenseMatrix64F jacobianToPack)
   {
      int size = indexHandler.getNumberOfVRPWaypoints();

      tempJ.reshape(indexHandler.getTotalSize(), size);
      tempJ.zero();

      CommonOps.multAdd(vrpBoundsJacobian, bezierMapMultiplier, tempJ);
      CommonOps.mult(coefficientConstraintInv, tempJ, jacobianToPack);
   }

   public void computeJacobianToVRPBounds(DenseMatrix64F jacobianToPack)
   {
      jacobianToPack.set(bezierMapMultiplier);
   }

   static void populateWorkWeightTermForSegment(int segmentNumber, double segmentDuration, double omega, double costWeight, DenseMatrix64F matrixToPack)
   {
//      segmentDuration = Math.min(segmentDuration, CoMTrajectoryPlannerTools.sufficientlyLarge);
      segmentDuration = Math.min(segmentDuration, sufficientlyLongTime);
      double t2 = Math.min(segmentDuration * segmentDuration, CoMTrajectoryPlannerTools.sufficientlyLarge);
      double t3 = Math.min(segmentDuration * t2, CoMTrajectoryPlannerTools.sufficientlyLarge);
      double omega3 = Math.pow(omega, 3);
      double omega4 = Math.pow(omega, 4);
      double omegaT = omega * segmentDuration;
      double eOmegaT = Math.min(Math.exp(omegaT), CoMTrajectoryPlannerTools.sufficientlyLarge);
      double e2OmegaT = Math.min(eOmegaT * eOmegaT, CoMTrajectoryPlannerTools.sufficientlyLarge);
      double eOmegaNegT = Math.exp(-omegaT);
      double e2OmegaNegT = eOmegaNegT * eOmegaNegT;

      int startIndex = 6 * segmentNumber;

      matrixToPack.set(startIndex,     startIndex,     costWeight * 0.5 * omega3 * (e2OmegaT - 1.0));
      matrixToPack.set(startIndex,     startIndex + 1, costWeight * omega4 * segmentDuration);
      matrixToPack.set(startIndex,     startIndex + 2, costWeight * 6.0 * (Math.min(eOmegaT * (omegaT - 1), CoMTrajectoryPlannerTools.sufficientlyLarge) + 1));
      matrixToPack.set(startIndex,     startIndex + 3, costWeight * 2.0 * omega * (eOmegaT - 1.0));

      matrixToPack.set(startIndex + 1, startIndex,     matrixToPack.get(startIndex, startIndex + 1));
      matrixToPack.set(startIndex + 1, startIndex + 1, costWeight * -0.5 * omega3 * (e2OmegaNegT - 1.0));
      matrixToPack.set(startIndex + 1, startIndex + 2, costWeight * -6.0 * (eOmegaNegT * (omegaT + 1) - 1));
      matrixToPack.set(startIndex + 1, startIndex + 3, costWeight * -2.0 * omega * (eOmegaNegT - 1.0));

      matrixToPack.set(startIndex + 2, startIndex,     matrixToPack.get(startIndex,     startIndex + 2));
      matrixToPack.set(startIndex + 2, startIndex + 1, matrixToPack.get(startIndex + 1, startIndex + 2));
      matrixToPack.set(startIndex + 2, startIndex + 2, costWeight * 12.0 * t3);
      matrixToPack.set(startIndex + 2, startIndex + 3, costWeight * 6.0 * t2);

      matrixToPack.set(startIndex + 3, startIndex,     matrixToPack.get(startIndex,     startIndex + 3));
      matrixToPack.set(startIndex + 3, startIndex + 1, matrixToPack.get(startIndex + 1, startIndex + 3));
      matrixToPack.set(startIndex + 3, startIndex + 2, matrixToPack.get(startIndex + 2, startIndex + 3));
      matrixToPack.set(startIndex + 3, startIndex + 3, costWeight * 4.0 * segmentDuration);
   }
}
