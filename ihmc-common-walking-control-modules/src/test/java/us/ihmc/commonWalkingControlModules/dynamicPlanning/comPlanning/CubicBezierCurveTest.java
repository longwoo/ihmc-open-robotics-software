package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.Trajectory3D;

import java.util.Random;

public class CubicBezierCurveTest
{
   private static final double epsilon = 1e-10;

   @Test
   public void testAgainstCubicSpline()
   {
      Trajectory3D trajectory3D = new Trajectory3D(4);
      CubicBezierCurve bezierCurve = new CubicBezierCurve();
      Random random = new Random(1738L);

      for (int i = 0; i < 100; i++)
      {
         double duration = RandomNumbers.nextDouble(random, 0.1, 10.0);
         Point3DReadOnly initialPosition = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Vector3DReadOnly initialVelocity = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         Point3DReadOnly finalPosition = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Vector3DReadOnly finalVelocity = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);

         trajectory3D.setCubic(0, duration, initialPosition, initialVelocity, finalPosition, finalVelocity);
         bezierCurve.setFromBounds(duration, initialPosition, initialVelocity, finalPosition, finalVelocity);

         for (double time = 0.0; time <= duration; time += 0.01)
         {
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(trajectory3D.getPosition(), bezierCurve.getDesiredPosition(), epsilon);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(trajectory3D.getVelocity(), bezierCurve.getDesiredVelocity(), epsilon);
         }
      }
   }
}
