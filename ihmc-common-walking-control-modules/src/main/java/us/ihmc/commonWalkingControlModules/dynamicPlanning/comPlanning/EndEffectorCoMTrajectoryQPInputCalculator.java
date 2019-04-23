package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.List;

public class EndEffectorCoMTrajectoryQPInputCalculator
{
   private final EndEffectorCoMTrajectoryPlannerIndexHandler indexHandler;
   private final DoubleProvider omega;

   private final DenseMatrix64F bezierPointsToVRPBoundsJacobian = new DenseMatrix64F(40, 40);
   private final DenseMatrix64F tempJ = new DenseMatrix64F(40, 40);

   public EndEffectorCoMTrajectoryQPInputCalculator(EndEffectorCoMTrajectoryPlannerIndexHandler indexHandler,
                                                    DoubleProvider omega)
   {
      this.indexHandler = indexHandler;
      this.omega = omega;
   }

   public void computeBezierMapMultiplier(List<? extends ContactStateProvider> contactSequence)
   {
      int size = indexHandler.getNumberOfEndEffectorVRPWaypoints();

      bezierPointsToVRPBoundsJacobian.reshape(size, size);
      bezierPointsToVRPBoundsJacobian.zero();

      int waypointSetNumber = 0;
      for (int segmentNumber = 0; segmentNumber < contactSequence.size(); segmentNumber++)
      {
         ContactStateProvider contactStateProvider = contactSequence.get(segmentNumber);
         if (!contactStateProvider.getContactState().isLoadBearing())
            continue;

         double segmentDuration = contactStateProvider.getTimeInterval().getDuration();
         for (int endEffectorNumber = 0; endEffectorNumber < contactStateProvider.getNumberOfBodiesInContact(); endEffectorNumber++)
         {
            CubicBezierCurve.computeMapToCurveBounds(4 * waypointSetNumber, 4 * waypointSetNumber, bezierPointsToVRPBoundsJacobian, segmentDuration);

            waypointSetNumber++;
         }
      }
   }

   public void computeEndEffector

   /*
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
   */

   public void computeJacobianToCoMCoefficients(DenseMatrix64F coefficientConstraintInv, DenseMatrix64F netVrpBoundsJacobian, DenseMatrix64F jacobianToPack)
   {
      int size = indexHandler.getNumberOfVRPWaypoints();

      tempJ.reshape(indexHandler.getTotalNumberOfCoefficients(), size);
      tempJ.zero();

      CommonOps.multAdd(netVrpBoundsJacobian, bezierPointsToVRPBoundsJacobian, tempJ);
      CommonOps.mult(coefficientConstraintInv, tempJ, jacobianToPack);
   }

   /*
   public void computeCoMCoefficientsObjective(DenseMatrix64F coefficientConstraintInv, DenseMatrix64F xCoefficientConstraintConstants,
                                               DenseMatrix64F yCoefficientConstraintConstants, DenseMatrix64F zCoefficientConstraintConstants,
                                               DenseMatrix64F xObjectiveToPack, DenseMatrix64F yObjectiveToPack, DenseMatrix64F zObjectiveToPack)
   {
      NativeCommonOps.mult(coefficientConstraintInv, xCoefficientConstraintConstants, xObjectiveToPack);
      NativeCommonOps.mult(coefficientConstraintInv, yCoefficientConstraintConstants, yObjectiveToPack);
      NativeCommonOps.mult(coefficientConstraintInv, zCoefficientConstraintConstants, zObjectiveToPack);

      CommonOps.scale(-1.0, xObjectiveToPack);
      CommonOps.scale(-1.0, yObjectiveToPack);
      CommonOps.scale(-1.0, zObjectiveToPack);
   }

   public void computeJacobianToVRPBounds(DenseMatrix64F jacobianToPack)
   {
      jacobianToPack.set(bezierPointsToVRPBoundsJacobian);
   }

   public void computeJacobianToCoMPosition(int segment, double timeInSegment, double omega, DenseMatrix64F jacobianToCoMCoefficients,
                                            DenseMatrix64F jacobianToPack)
   {
      int size = indexHandler.getNumberOfVRPWaypoints();
      jacobianToPack.reshape(1, size);
      jacobianToPack.zero();

      tempJ.reshape(1, indexHandler.getTotalNumberOfCoefficients());
      tempJ.zero();

      // TODO the tempJ is super sparse, so this could be a lot faster
      populateCoMPositionTimeVector(segment, timeInSegment, omega, tempJ);
      CommonOps.mult(tempJ, jacobianToCoMCoefficients, jacobianToPack);
   }

   public void computeJacobianToCoMVelocity(int segment, double timeInSegment, double omega, DenseMatrix64F jacobianToCoMCoefficients,
                                            DenseMatrix64F xConstraintObjective, DenseMatrix64F yConstraintObjective, DenseMatrix64F zConstraintObjective,
                                            FrameVector3DReadOnly desiredCoMVelocity, QPInput taskToPack)
   {
      int size = indexHandler.getNumberOfVRPWaypoints();

      tempJ.reshape(1, indexHandler.getTotalNumberOfCoefficients());
      tempJ.zero();

      // TODO the tempJ is super sparse, so this could be a lot faster
      populateCoMVelocityTimeVector(segment, timeInSegment, omega, tempJ);
      CommonOps.mult(tempJ, jacobianToCoMCoefficients, taskToPack.taskJacobian);
      CommonOps.mult(tempJ, xConstraintObjective, taskToPack.taskXObjective);
      CommonOps.mult(tempJ, yConstraintObjective, taskToPack.taskYObjective);
      CommonOps.mult(tempJ, zConstraintObjective, taskToPack.taskZObjective);
      CommonOps.add(taskToPack.taskXObjective, desiredCoMVelocity.getX());
      CommonOps.add(taskToPack.taskYObjective, desiredCoMVelocity.getY());
      CommonOps.add(taskToPack.taskZObjective, desiredCoMVelocity.getZ());
   }

   public void computeJacobianToCoMAcceleration(int segment, double timeInSegment, double omega, DenseMatrix64F jacobianToCoMCoefficients,
                                                DenseMatrix64F jacobianToPack)
   {
      int size = indexHandler.getNumberOfVRPWaypoints();
      jacobianToPack.reshape(1, size);
      jacobianToPack.zero();

      tempJ.reshape(1, indexHandler.getTotalNumberOfCoefficients());
      tempJ.zero();

      // TODO the tempJ is super sparse, so this could be a lot faster
      populateCoMAccelerationTimeVector(segment, timeInSegment, omega, tempJ);
      CommonOps.mult(tempJ, jacobianToCoMCoefficients, jacobianToPack);
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

   static void populateCoMPositionTimeVector(int segment, double segmentTime, double omega, DenseMatrix64F matrixToPack)
   {
      int startCol = 6 * segment;
      matrixToPack.add(0, startCol,     CoMTrajectoryPlannerTools.getCoMPositionFirstCoefficientTimeFunction(omega, segmentTime));
      matrixToPack.add(0, startCol + 1, CoMTrajectoryPlannerTools.getCoMPositionSecondCoefficientTimeFunction(omega, segmentTime));
      matrixToPack.add(0, startCol + 2, CoMTrajectoryPlannerTools.getCoMPositionThirdCoefficientTimeFunction(segmentTime));
      matrixToPack.add(0, startCol + 3, CoMTrajectoryPlannerTools.getCoMPositionFourthCoefficientTimeFunction(segmentTime));
      matrixToPack.add(0, startCol + 4, CoMTrajectoryPlannerTools.getCoMPositionFifthCoefficientTimeFunction(segmentTime));
      matrixToPack.add(0, startCol + 5, CoMTrajectoryPlannerTools.getCoMPositionSixthCoefficientTimeFunction());
   }

   static void populateCoMVelocityTimeVector(int segment, double segmentTime, double omega, DenseMatrix64F matrixToPack)
   {
      int startCol = 6 * segment;
      matrixToPack.add(0, startCol,     CoMTrajectoryPlannerTools.getCoMVelocityFirstCoefficientTimeFunction(omega, segmentTime));
      matrixToPack.add(0, startCol + 1, CoMTrajectoryPlannerTools.getCoMVelocitySecondCoefficientTimeFunction(omega, segmentTime));
      matrixToPack.add(0, startCol + 2, CoMTrajectoryPlannerTools.getCoMVelocityThirdCoefficientTimeFunction(segmentTime));
      matrixToPack.add(0, startCol + 3, CoMTrajectoryPlannerTools.getCoMVelocityFourthCoefficientTimeFunction(segmentTime));
      matrixToPack.add(0, startCol + 4, CoMTrajectoryPlannerTools.getCoMVelocityFifthCoefficientTimeFunction());
      matrixToPack.add(0, startCol + 5, CoMTrajectoryPlannerTools.getCoMVelocitySixthCoefficientTimeFunction());
   }

   static void populateCoMAccelerationTimeVector(int segment, double segmentTime, double omega, DenseMatrix64F matrixToPack)
   {
      int startCol = 6 * segment;
      matrixToPack.add(0, startCol,     CoMTrajectoryPlannerTools.getCoMAccelerationFirstCoefficientTimeFunction(omega, segmentTime));
      matrixToPack.add(0, startCol + 1, CoMTrajectoryPlannerTools.getCoMAccelerationSecondCoefficientTimeFunction(omega, segmentTime));
      matrixToPack.add(0, startCol + 2, CoMTrajectoryPlannerTools.getCoMAccelerationThirdCoefficientTimeFunction(segmentTime));
      matrixToPack.add(0, startCol + 3, CoMTrajectoryPlannerTools.getCoMAccelerationFourthCoefficientTimeFunction());
      matrixToPack.add(0, startCol + 4, CoMTrajectoryPlannerTools.getCoMAccelerationFifthCoefficientTimeFunction());
      matrixToPack.add(0, startCol + 5, CoMTrajectoryPlannerTools.getCoMAccelerationSixthCoefficientTimeFunction());
   }
   */
}
