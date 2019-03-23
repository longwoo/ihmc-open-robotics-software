package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.linearAlgebra.commonOps.NativeCommonOps;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.sufficientlyLarge;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.sufficientlyLongTime;

public class CubicBezierCurve
{
   private final Point3D point0 = new Point3D();
   private final Point3D point1 = new Point3D();
   private final Point3D point2 = new Point3D();
   private final Point3D point3 = new Point3D();

   private final DenseMatrix64F constraintMatrix = new DenseMatrix64F(4, 4);
   private final DenseMatrix64F constraintMatrixInv = new DenseMatrix64F(4, 4);
   private final DenseMatrix64F bounds = new DenseMatrix64F(4, 1);
   private final DenseMatrix64F coefficients = new DenseMatrix64F(4, 1);

   private final Point3D desiredPosition = new Point3D();
   private final Vector3D desiredVelocity = new Vector3D();

   public void setFromPoints(Point3DReadOnly point0, Point3DReadOnly point1, Point3DReadOnly point2, Point3DReadOnly point3)
   {
      this.point0.set(point0);
      this.point1.set(point1);
      this.point2.set(point2);
      this.point3.set(point3);
   }

   public void setFromBounds(double duration, Point3DReadOnly startPosition, Vector3DReadOnly startVelocity, Point3DReadOnly endPosition, Vector3DReadOnly endVelocity)
   {
      constraintMatrix.zero();
      constraintMatrixInv.zero();

      computeMapToCurveBounds(constraintMatrix, duration);
      NativeCommonOps.invert(constraintMatrix, constraintMatrixInv);

      for (int i = 0; i < 3; i++)
      {
         bounds.set(0, 0, startPosition.getElement(i));
         bounds.set(1, 0, startVelocity.getElement(i));
         bounds.set(2, 0, endPosition.getElement(i));
         bounds.set(3, 0, endVelocity.getElement(i));

         CommonOps.mult(constraintMatrixInv, bounds, coefficients);

         point0.setElement(i, coefficients.get(0, 0));
         point1.setElement(i, coefficients.get(1, 0));
         point2.setElement(i, coefficients.get(2, 0));
         point3.setElement(i, coefficients.get(3, 0));
      }
   }

   public void compute(double time)
   {
      desiredPosition.set(point0);
      desiredPosition.scale(getFirstCoefficientPositionMultiplier(time));
      desiredPosition.scaleAdd(getSecondCoefficientPositionMultiplier(time), point1, desiredPosition);
      desiredPosition.scaleAdd(getThirdCoefficientPositionMultiplier(time), point2, desiredPosition);
      desiredPosition.scaleAdd(getFourthCoefficientPositionMultiplier(time), point3, desiredPosition);

      desiredVelocity.set(point0);
      desiredVelocity.scale(getFirstCoefficientVelocityMultiplier(time));
      desiredVelocity.scaleAdd(getSecondCoefficientVelocityMultiplier(time), point1, desiredVelocity);
      desiredVelocity.scaleAdd(getThirdCoefficientVelocityMultiplier(time), point2, desiredVelocity);
      desiredVelocity.scaleAdd(getFourthCoefficientVelocityMultiplier(time), point3, desiredVelocity);
   }

   public Point3DReadOnly getDesiredPosition()
   {
      return desiredPosition;
   }

   public Vector3DReadOnly getDesiredVelocity()
   {
      return desiredVelocity;
   }

   public static void computeMapToCurveBounds(DenseMatrix64F mapToPack, double duration)
   {
      computeMapToCurveBounds(0, 0, mapToPack, duration);
   }

   public static void computeMapToCurveBounds(int rowStart, int colStart, DenseMatrix64F mapToPack, double duration)
   {
      duration = Math.min(duration, sufficientlyLongTime);
      duration = Math.min(duration, CoMTrajectoryPlannerTools.sufficientlyLargeThird);

      mapToPack.set(rowStart,     colStart,     1.0); // initial position
      mapToPack.set(rowStart + 1, colStart,    -3.0); // initial velocity
      mapToPack.set(rowStart + 1, colStart + 1, 3.0);
      mapToPack.set(rowStart + 2, colStart,     MathTools.clamp(getFirstCoefficientPositionMultiplier(duration), sufficientlyLarge)); // final position
      mapToPack.set(rowStart + 2, colStart + 1, MathTools.clamp(getSecondCoefficientPositionMultiplier(duration), sufficientlyLarge));
      mapToPack.set(rowStart + 2, colStart + 2, MathTools.clamp(getThirdCoefficientPositionMultiplier(duration), sufficientlyLarge));
      mapToPack.set(rowStart + 2, colStart + 3, MathTools.clamp(getFourthCoefficientPositionMultiplier(duration), sufficientlyLarge));
      mapToPack.set(rowStart + 3, colStart,     MathTools.clamp(getFirstCoefficientVelocityMultiplier(duration), sufficientlyLarge)); // final velocity
      mapToPack.set(rowStart + 3, colStart + 1, MathTools.clamp(getSecondCoefficientVelocityMultiplier(duration), sufficientlyLarge));
      mapToPack.set(rowStart + 3, colStart + 2, MathTools.clamp(getThirdCoefficientVelocityMultiplier(duration), sufficientlyLarge));
      mapToPack.set(rowStart + 3, colStart + 3, MathTools.clamp(getFourthCoefficientVelocityMultiplier(duration), sufficientlyLarge));
   }

   private static double getFirstCoefficientPositionMultiplier(double time)
   {
      return MathTools.pow(1.0 - time, 3);
   }

   private static double getSecondCoefficientPositionMultiplier(double time)
   {
      return 3.0 * MathTools.square(1.0 - time) * time;
   }

   private static double getThirdCoefficientPositionMultiplier(double time)
   {
      return 3.0 * (1.0 - time) * MathTools.square(time);
   }

   private static double getFourthCoefficientPositionMultiplier(double time)
   {
      return MathTools.pow(time, 3);
   }

   private static double getFirstCoefficientVelocityMultiplier(double time)
   {
      return -3.0 * MathTools.square(1.0 - time);
   }

   private static double getSecondCoefficientVelocityMultiplier(double time)
   {
      return 3.0 * MathTools.square(1.0 - time) - 6.0 * (1.0 - time) * time;
   }

   private static double getThirdCoefficientVelocityMultiplier(double time)
   {
      return 6.0 * (1.0 - time) * time - 3.0 * MathTools.square(time);
   }

   private static double getFourthCoefficientVelocityMultiplier(double time)
   {
      return 3.0 * MathTools.square(time);
   }
}
