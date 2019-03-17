package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class CubicBezierCurve
{
   private final Point3D initialPosition = new Point3D();
   private final Point3D finalPosition = new Point3D();
   private final Point3D point1 = new Point3D();
   private final Point3D point2 = new Point3D();

   private double duration;

   public void setInitialPosition(Point3DReadOnly initialPosition)
   {
      this.initialPosition.set(initialPosition);
   }

   public void setFinalPosition(Point3DReadOnly finalPosition)
   {
      this.finalPosition.set(finalPosition);
   }

   public void setPoint1(Point3DReadOnly point1)
   {
      this.point1.set(point1);
   }

   public void setPoint2(Point3DReadOnly point2)
   {
      this.point2.set(point2);
   }

   public void setDuration(double duration)
   {
      this.duration = duration;
   }

   public static void computeMapToCurveBounds(DenseMatrix64F mapToPack, double duration)
   {
      computeMapToCurveBounds(0, 0, mapToPack, duration);
   }

   public static void computeMapToCurveBounds(int rowStart, int colStart, DenseMatrix64F mapToPack, double duration)
   {
      mapToPack.reshape(4, 4);
      mapToPack.zero();

      mapToPack.set(rowStart,     colStart,     1.0); // initial position
      mapToPack.set(rowStart + 1, colStart,    -3.0); // initial velocity
      mapToPack.set(rowStart + 1, colStart + 1, 3.0);
      mapToPack.set(rowStart + 2, colStart,     getFirstCoefficientPositionMultiplier(duration)); // final position
      mapToPack.set(rowStart + 2, colStart + 1, getSecondCoefficientPositionMultiplier(duration)); // final position
      mapToPack.set(rowStart + 2, colStart + 2, getThirdCoefficientPositionMultiplier(duration)); // final position
      mapToPack.set(rowStart + 2, colStart + 3, getFourthCoefficientPositionMultiplier(duration)); // final position
      mapToPack.set(rowStart + 3, colStart,     getFirstCoefficientVelocityMultiplier(duration)); // final velocity
      mapToPack.set(rowStart + 3, colStart + 1, getSecondCoefficientVelocityMultiplier(duration)); // final velocity
      mapToPack.set(rowStart + 3, colStart + 2, getThirdCoefficientVelocityMultiplier(duration)); // final velocity
      mapToPack.set(rowStart + 3, colStart + 3, getFourthCoefficientVelocityMultiplier(duration)); // final velocity
   }

   private static double getFirstCoefficientPositionMultiplier(double time)
   {
      return MathTools.pow(1.0 - time, 3);
   }

   private static double getSecondCoefficientPositionMultiplier(double time)
   {
      return 3.0 * MathTools.square(1.0 - time);
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
