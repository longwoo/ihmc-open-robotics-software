package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class CoMTrajectoryPlannerTest
{
   private static final double epsilon = 1e-4;

   @Test
   public void testNoSteps()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);
      double gravityZ = 9.81;
      double nominalHeight = 0.7;

      List<ContactStateProvider> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(omega, gravityZ, nominalHeight, registry);

      SettableContactStateProvider firstContact = new SettableContactStateProvider();

      firstContact.setTimeInterval(new TimeInterval(0.0, Double.POSITIVE_INFINITY));
      firstContact.setStartCopPosition(new FramePoint3D());
      firstContact.setEndCopPosition(new FramePoint3D());

      contactSequence.add(firstContact);

      FramePoint3D comPosition = new FramePoint3D();
      FrameVector3D comVelocity = new FrameVector3D();
      comPosition.setZ(nominalHeight);
      planner.setInitialCenterOfMassState(comPosition, comVelocity);

      planner.solveForTrajectory(contactSequence);
      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      FramePoint3D desiredDCM = new FramePoint3D(firstContact.getCopStartPosition());
      desiredDCM.addZ(nominalHeight);
      FrameVector3D desiredDCMVelocity = new FrameVector3D();

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredDCMPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredVRPPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredCoMPosition(), epsilon);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredDCMVelocity, planner.getDesiredDCMVelocity(), epsilon);

      planner.compute(1.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredDCMPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredVRPPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredCoMPosition(), epsilon);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredDCMVelocity, planner.getDesiredDCMVelocity(), epsilon);

      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredDCMPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredVRPPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredCoMPosition(), epsilon);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredDCMVelocity, planner.getDesiredDCMVelocity(), epsilon);

      planner.compute(10.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredDCMPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredVRPPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredCoMPosition(), epsilon);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredDCMVelocity, planner.getDesiredDCMVelocity(), epsilon);

      planner.compute(100.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredDCMPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredVRPPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredCoMPosition(), epsilon);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredDCMVelocity, planner.getDesiredDCMVelocity(), epsilon);

      planner.compute(1000.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredDCMPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredVRPPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredCoMPosition(), epsilon);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredDCMVelocity, planner.getDesiredDCMVelocity(), epsilon);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, 5000.0);
         planner.compute(time);
         checkPlannerDynamics(planner, omega.getDoubleValue());

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredDCMPosition(), epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredVRPPosition(), epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredCoMPosition(), epsilon);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredDCMVelocity, planner.getDesiredDCMVelocity(), epsilon);
      }
   }

   @Test
   public void testOneSimpleMovingSegmentInContact()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);
      double gravityZ = 9.81;
      double nominalHeight = 0.7;

      List<ContactStateProvider> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(omega, gravityZ, nominalHeight, registry);

      SettableContactStateProvider firstContact = new SettableContactStateProvider();
      SettableContactStateProvider secondContact = new SettableContactStateProvider();

      double duration = 1.0;

      firstContact.setTimeInterval(new TimeInterval(0.0, duration));
      firstContact.setStartCopPosition(new FramePoint3D());
      firstContact.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.25, 0.0));
      secondContact.setTimeInterval(new TimeInterval(1.0, 2.0));
      secondContact.setStartCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.25, 0.0));
      secondContact.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.25, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);

      FramePoint3D comPosition = new FramePoint3D();
      FrameVector3D comVelocity = new FrameVector3D();
      comPosition.setZ(nominalHeight);
      planner.setInitialCenterOfMassState(comPosition, comVelocity);

      planner.solveForTrajectory(contactSequence);
      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      FramePoint3D finalDCM = new FramePoint3D(secondContact.getCopStartPosition());
      finalDCM.addZ(nominalHeight);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comPosition, planner.getDesiredCoMPosition(), epsilon);

      planner.compute(duration);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(finalDCM, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3D initialDCM = new FramePoint3D();
      DCMTrajectoryTools
            .computeDCMUsingLinearVRP(omega.getDoubleValue(), -duration, -duration, secondContact.getCopStartPosition(), secondContact.getCopStartPosition(),
                                      firstContact.getCopStartPosition(), initialDCM);
      initialDCM.addZ(nominalHeight);

      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialDCM, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3D initialVRP = new FramePoint3D();
      FramePoint3D finalVRP = new FramePoint3D();
      initialVRP.set(firstContact.getCopStartPosition());
      finalVRP.set(firstContact.getCopEndPosition());
      initialVRP.addZ(nominalHeight);
      finalVRP.addZ(nominalHeight);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, 1.0);
         FramePoint3D expectedDCM = new FramePoint3D();
         DCMTrajectoryTools.computeDCMUsingLinearVRP(omega.getDoubleValue(), time, duration, initialDCM, initialVRP, finalVRP, expectedDCM);

         planner.compute(time);
         checkPlannerDynamics(planner, omega.getDoubleValue());

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("time : " + time, expectedDCM, planner.getDesiredDCMPosition(), epsilon);
      }
   }

   @Test
   public void testOneMovingSegmentInContact()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);
      double gravityZ = 9.81;
      double nominalHeight = 0.7;

      List<ContactStateProvider> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(omega, gravityZ, nominalHeight, registry);

      SettableContactStateProvider firstContact = new SettableContactStateProvider();
      SettableContactStateProvider secondContact = new SettableContactStateProvider();

      firstContact.setTimeInterval(new TimeInterval(0.0, 1.0));
      firstContact.setStartCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.1, 0.15, 0.0));
      firstContact.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.25, 0.0));
      secondContact.setTimeInterval(new TimeInterval(1.0, 2.0));
      secondContact.setStartCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.25, 0.0));
      secondContact.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.25, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);

      FramePoint3D comPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.1, 0.15, nominalHeight);
      FrameVector3D comVelocity = new FrameVector3D();
      planner.setInitialCenterOfMassState(comPosition, comVelocity);

      planner.solveForTrajectory(contactSequence);
      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      FramePoint3D finalICP = new FramePoint3D(secondContact.getCopStartPosition());
      finalICP.addZ(nominalHeight);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comPosition, planner.getDesiredCoMPosition(), epsilon);

      planner.compute(1.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(finalICP, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3DReadOnly initialDCM = recursivelyComputeInitialDCMLinear(contactSequence, nominalHeight, omega.getDoubleValue());

      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialDCM, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3D initialVRP = new FramePoint3D();
      initialVRP.set(firstContact.getCopStartPosition());
      initialVRP.addZ(nominalHeight);

      FramePoint3D finalVRP = new FramePoint3D();
      finalVRP.set(firstContact.getCopEndPosition());
      finalVRP.addZ(nominalHeight);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, 1.0);
         FramePoint3D expectedDCM = new FramePoint3D();
         DCMTrajectoryTools
               .computeDCMUsingLinearVRP(omega.getDoubleValue(), time, contactSequence.get(0).getTimeInterval().getDuration(), initialDCM, initialVRP, finalVRP,
                                         expectedDCM);

         planner.compute(time);
         checkPlannerDynamics(planner, omega.getDoubleValue());

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("time : " + time, expectedDCM, planner.getDesiredDCMPosition(), epsilon);
      }
   }

   @Test
   public void testTwoMovingSegmentsInContact()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);
      double gravityZ = 9.81;
      double nominalHeight = 0.7;

      List<ContactStateProvider> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(omega, gravityZ, nominalHeight, registry);

      SettableContactStateProvider firstContact = new SettableContactStateProvider();
      SettableContactStateProvider secondContact = new SettableContactStateProvider();
      SettableContactStateProvider thirdContact = new SettableContactStateProvider();

      firstContact.setTimeInterval(new TimeInterval(0.0, 0.75));
      firstContact.setStartCopPosition(new FramePoint3D());
      firstContact.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.5, 0.0));
      secondContact.setTimeInterval(new TimeInterval(0.75, 1.9));
      secondContact.setStartCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.5, 0.0));
      secondContact.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0));
      thirdContact.setTimeInterval(new TimeInterval(1.9, 3.0));
      thirdContact.setStartCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0));
      thirdContact.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);
      contactSequence.add(thirdContact);

      FramePoint3D comPosition = new FramePoint3D();
      FrameVector3D comVelocity = new FrameVector3D();
      comPosition.setZ(nominalHeight);
      planner.setInitialCenterOfMassState(comPosition, comVelocity);

      planner.solveForTrajectory(contactSequence);
      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comPosition, planner.getDesiredCoMPosition(), epsilon);

      FramePoint3DReadOnly initialDCM = recursivelyComputeInitialDCMLinear(contactSequence, nominalHeight, omega.getDoubleValue());

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialDCM, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3D initialVRP = new FramePoint3D(firstContact.getCopStartPosition());
      FramePoint3D finalVRP = new FramePoint3D(firstContact.getCopEndPosition());
      initialVRP.addZ(nominalHeight);
      finalVRP.addZ(nominalHeight);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, 1.0);
         FramePoint3D expectedDCM = new FramePoint3D();
         DCMTrajectoryTools
               .computeDCMUsingLinearVRP(omega.getDoubleValue(), time, contactSequence.get(0).getTimeInterval().getDuration(), initialDCM, initialVRP, finalVRP,
                                         expectedDCM);

         planner.compute(time);
         checkPlannerDynamics(planner, omega.getDoubleValue());

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedDCM, planner.getDesiredDCMPosition(), epsilon);
      }
   }

   private FramePoint3DReadOnly recursivelyComputeInitialDCMLinear(List<ContactStateProvider> contactPhases, double nominalHeight, double omega)
   {
      int numberOfPhases = contactPhases.size();
      FramePoint3D lastDCM = new FramePoint3D(contactPhases.get(numberOfPhases - 1).getCopStartPosition());
      lastDCM.addZ(nominalHeight);
      FramePoint3D initialDCM = new FramePoint3D(lastDCM);
      for (int i = numberOfPhases - 2; i >= 0; i--)
      {
         FramePoint3D endDCM = new FramePoint3D(initialDCM);
         FramePoint3D initialVRP = new FramePoint3D(contactPhases.get(i).getCopStartPosition());
         FramePoint3D endVRP = new FramePoint3D(contactPhases.get(i).getCopEndPosition());
         initialVRP.addZ(nominalHeight);
         endVRP.addZ(nominalHeight);

         double duration = contactPhases.get(i).getTimeInterval().getDuration();

         DCMTrajectoryTools.computeDCMUsingLinearVRP(omega, -duration, -duration, endDCM, endVRP, initialVRP, initialDCM);
      }

      return initialDCM;
   }

   @Test
   public void testManyMovingSegmentsInContact3D()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);
      double gravityZ = 9.81;
      double nominalHeight = 0.7;

      List<ContactStateProvider> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(omega, gravityZ, nominalHeight, registry);

      Random random = new Random(1738L);

      for (int iter = 0; iter < 10; iter++)
      {
         contactSequence.clear();

         double initialTime = 0.0;
         FramePoint3DReadOnly startCoPPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0);
         int numberOfContacts = RandomNumbers.nextInt(random, 2, 10);

         double currentStartTime = initialTime;

         double segmentDuration = RandomNumbers.nextDouble(random, 0.0, 5.0);

         // handle initial phase
         SettableContactStateProvider contactPhase = new SettableContactStateProvider();
         contactPhase.setTimeInterval(new TimeInterval(currentStartTime, segmentDuration + currentStartTime));
         contactPhase.setStartCopPosition(startCoPPosition);

         FramePoint3D currentCoPPosition = new FramePoint3D(startCoPPosition);
         currentCoPPosition.add(EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame(), new Vector3D(1.0, 1.0, 0.0)));

         contactPhase.setEndCopPosition(currentCoPPosition);

         contactSequence.add(contactPhase);

         currentStartTime += segmentDuration;

         // add more phases
         for (int contactIndex = 1; contactIndex < numberOfContacts; contactIndex++)
         {
            segmentDuration = RandomNumbers.nextDouble(random, 0.0, 5.0);

            contactPhase = new SettableContactStateProvider();
            contactPhase.setTimeInterval(new TimeInterval(currentStartTime, segmentDuration + currentStartTime));
            contactPhase.setStartCopPosition(contactSequence.get(contactIndex - 1).getCopEndPosition());

            currentCoPPosition.add(EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame(), new Vector3D(1.0, 1.0, 0.0)));

            contactPhase.setEndCopPosition(currentCoPPosition);

            contactSequence.add(contactPhase);

            currentStartTime += segmentDuration;
         }

         FramePoint3D comPosition = new FramePoint3D();
         FrameVector3D comVelocity = new FrameVector3D();
         comPosition.setZ(nominalHeight);
         planner.setInitialCenterOfMassState(comPosition, comVelocity);

         planner.solveForTrajectory(contactSequence);
         planner.compute(0.0);
         checkPlannerDynamics(planner, omega.getDoubleValue());

         EuclidCoreTestTools
               .assertPoint3DGeometricallyEquals("iter = " + iter + ", Initial CoM is wrong.", comPosition, planner.getDesiredCoMPosition(), epsilon);

         FramePoint3DReadOnly desiredInitialDCM = planner.getDesiredDCMPosition();

         FramePoint3DReadOnly initialDCM = recursivelyComputeInitialDCMLinear(contactSequence, nominalHeight, omega.getDoubleValue());
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("iter = " + iter + ", Initial DCM is wrong.", initialDCM, desiredInitialDCM,
                                                              initialDCM.distance(new Point3D()) * epsilon);

         FramePoint3D initialVRP = new FramePoint3D(contactSequence.get(0).getCopStartPosition());
         FramePoint3D finalVRP = new FramePoint3D(contactSequence.get(0).getCopEndPosition());
         initialVRP.addZ(nominalHeight);
         finalVRP.addZ(nominalHeight);

         double duration = contactSequence.get(0).getTimeInterval().getDuration();
         for (int i = 0; i < 100; i++)
         {
            double time = RandomNumbers.nextDouble(random, 0.0, duration);

            FramePoint3D expectedDCM = new FramePoint3D();
            DCMTrajectoryTools.computeDCMUsingLinearVRP(omega.getDoubleValue(), time, duration, initialDCM, initialVRP, finalVRP, expectedDCM);

            planner.compute(time);
            checkPlannerDynamics(planner, omega.getDoubleValue());

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals("time : " + time, expectedDCM, planner.getDesiredDCMPosition(), 100.0 * epsilon);
         }
      }
   }

   @Test
   public void testTwoMovingSegmentsOneFlight()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);
      double gravityZ = 9.81;
      double nominalHeight = 0.7;

      List<ContactStateProvider> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(omega, gravityZ, nominalHeight, registry);

      SettableContactStateProvider firstContact = new SettableContactStateProvider();
      SettableContactStateProvider secondContact = new SettableContactStateProvider();
      SettableContactStateProvider thirdContact = new SettableContactStateProvider();

      firstContact.setTimeInterval(new TimeInterval(0.0, 1.0));
      firstContact.setStartCopPosition(new FramePoint3D());
      firstContact.setEndCopPosition(new FramePoint3D());
      secondContact.setTimeInterval(new TimeInterval(1.0, 1.25));
      secondContact.setContactState(ContactState.FLIGHT);
      thirdContact.setTimeInterval(new TimeInterval(1.25, 2.25));
      thirdContact.setStartCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0));
      thirdContact.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);
      contactSequence.add(thirdContact);

      FramePoint3D comPosition = new FramePoint3D();
      FrameVector3D comVelocity = new FrameVector3D();
      comPosition.setZ(nominalHeight);
      planner.setInitialCenterOfMassState(comPosition, comVelocity);

      planner.solveForTrajectory(contactSequence);
      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Desired CoM is invalid.", comPosition, planner.getDesiredCoMPosition(), epsilon);

      FramePoint3D initialDCM = new FramePoint3D(planner.getDesiredDCMPosition());
      FramePoint3D firstVRP = new FramePoint3D(planner.getDesiredVRPPosition());

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, contactSequence.get(0).getTimeInterval().getDuration());
         FramePoint3D expectedDCM = new FramePoint3D();
         double exponential = Math.exp(omega.getDoubleValue() * time);
         expectedDCM.interpolate(firstVRP, initialDCM, exponential);

         planner.compute(time);
         checkPlannerDynamics(planner, omega.getDoubleValue());

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals("i = " + i, firstVRP, planner.getDesiredVRPPosition(), epsilon);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals("i = " + i, expectedDCM, planner.getDesiredDCMPosition(), epsilon);
      }
   }

   @Test
   public void testStartingInFlight()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);
      double gravityZ = 9.81;
      double nominalHeight = 0.7;

      List<ContactStateProvider> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(omega, gravityZ, nominalHeight, registry);

      SettableContactStateProvider firstContact = new SettableContactStateProvider();
      SettableContactStateProvider secondContact = new SettableContactStateProvider();

      firstContact.setTimeInterval(new TimeInterval(0.0, 0.25));
      firstContact.setContactState(ContactState.FLIGHT);
      secondContact.setTimeInterval(new TimeInterval(0.25, 1.25));
      secondContact.setStartCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0));
      secondContact.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);

      FramePoint3D comPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.05, 0.05, nominalHeight + 0.05);
      FrameVector3D comVelocity = new FrameVector3D();
      planner.setInitialCenterOfMassState(comPosition, comVelocity);

      planner.solveForTrajectory(contactSequence);
      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      FrameVector3DReadOnly acceleration = planner.getDesiredCoMAcceleration();

      assertEquals(0.0, acceleration.getX(), epsilon);
      assertEquals(0.0, acceleration.getY(), epsilon);
      assertEquals(-gravityZ, acceleration.getZ(), epsilon);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Desired CoM is invalid.", comPosition, planner.getDesiredCoMPosition(), epsilon);

      FramePoint3D secondVRP = new FramePoint3D(firstContact.getCopStartPosition());
      FramePoint3D thirdVRP = new FramePoint3D(secondContact.getCopStartPosition());
      secondVRP.addZ(nominalHeight);
      thirdVRP.addZ(nominalHeight);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, contactSequence.get(0).getTimeInterval().getDuration());
         planner.compute(time);
         checkPlannerDynamics(planner, omega.getDoubleValue());
      }
   }

   @Test
   public void testCoefficientCalculation()
   {
      Random random = new Random(1738L);
      double dt = 1e-4;
      for (int iter = 0; iter < 1000; iter++)
      {
         double omega = RandomNumbers.nextDouble(random, 0.3, 2.0);
         double time = RandomNumbers.nextDouble(random, 0.0, 1.5);

         double c0 = Math.exp(omega * time);
         double c1 = Math.exp(-omega * time);
         double c2 = Math.pow(time, 3.0);
         double c3 = Math.pow(time, 2.0);
         double c4 = time;
         double c5 = 1.0;

         assertEquals(c0, CoMTrajectoryPlannerTools.getCoMPositionFirstCoefficientTimeFunction(omega, time), epsilon);
         assertEquals(c1, CoMTrajectoryPlannerTools.getCoMPositionSecondCoefficientTimeFunction(omega, time), epsilon);
         assertEquals(c2, CoMTrajectoryPlannerTools.getCoMPositionThirdCoefficientTimeFunction(time), epsilon);
         assertEquals(c3, CoMTrajectoryPlannerTools.getCoMPositionFourthCoefficientTimeFunction(time), epsilon);
         assertEquals(c4, CoMTrajectoryPlannerTools.getCoMPositionFifthCoefficientTimeFunction(time), epsilon);
         assertEquals(c5, CoMTrajectoryPlannerTools.getCoMPositionSixthCoefficientTimeFunction(), epsilon);

         double c0_dot = omega * Math.exp(omega * time);
         double c1_dot = -omega * Math.exp(-omega * time);
         double c2_dot = 3.0 * time * time;
         double c3_dot = 2.0 * time;
         double c4_dot = 1.0;
         double c5_dot = 0.0;

         double c0_dot_numerical = (CoMTrajectoryPlannerTools.getCoMPositionFirstCoefficientTimeFunction(omega, time + dt) - CoMTrajectoryPlannerTools
               .getCoMPositionFirstCoefficientTimeFunction(omega, time)) / dt;
         double c1_dot_numerical = (CoMTrajectoryPlannerTools.getCoMPositionSecondCoefficientTimeFunction(omega, time + dt) - CoMTrajectoryPlannerTools
               .getCoMPositionSecondCoefficientTimeFunction(omega, time)) / dt;
         double c2_dot_numerical = (CoMTrajectoryPlannerTools.getCoMPositionThirdCoefficientTimeFunction(time + dt) - CoMTrajectoryPlannerTools
               .getCoMPositionThirdCoefficientTimeFunction(time)) / dt;
         double c3_dot_numerical = (CoMTrajectoryPlannerTools.getCoMPositionFourthCoefficientTimeFunction(time + dt) - CoMTrajectoryPlannerTools
               .getCoMPositionFourthCoefficientTimeFunction(time)) / dt;
         double c4_dot_numerical = (CoMTrajectoryPlannerTools.getCoMPositionFifthCoefficientTimeFunction(time + dt) - CoMTrajectoryPlannerTools
               .getCoMPositionFifthCoefficientTimeFunction(time)) / dt;

         assertEquals(c0_dot, CoMTrajectoryPlannerTools.getCoMVelocityFirstCoefficientTimeFunction(omega, time), epsilon);
         assertEquals(c1_dot, CoMTrajectoryPlannerTools.getCoMVelocitySecondCoefficientTimeFunction(omega, time), epsilon);
         assertEquals(c2_dot, CoMTrajectoryPlannerTools.getCoMVelocityThirdCoefficientTimeFunction(time), epsilon);
         assertEquals(c3_dot, CoMTrajectoryPlannerTools.getCoMVelocityFourthCoefficientTimeFunction(time), epsilon);
         assertEquals(c4_dot, CoMTrajectoryPlannerTools.getCoMVelocityFifthCoefficientTimeFunction(), epsilon);
         assertEquals(c5_dot, CoMTrajectoryPlannerTools.getCoMVelocitySixthCoefficientTimeFunction(), epsilon);
         assertEquals(c0_dot_numerical, CoMTrajectoryPlannerTools.getCoMVelocityFirstCoefficientTimeFunction(omega, time), 1e-3 * Math.max(c0_dot, 1.0));
         assertEquals(c1_dot_numerical, CoMTrajectoryPlannerTools.getCoMVelocitySecondCoefficientTimeFunction(omega, time), 1e-3 * Math.max(c1_dot, 1.0));
         assertEquals(c2_dot_numerical, CoMTrajectoryPlannerTools.getCoMVelocityThirdCoefficientTimeFunction(time), 1e-3 * Math.max(c2_dot, 1.0));
         assertEquals(c3_dot_numerical, CoMTrajectoryPlannerTools.getCoMVelocityFourthCoefficientTimeFunction(time), 1e-3 * Math.max(c3_dot, 1.0));
         assertEquals(c4_dot_numerical, CoMTrajectoryPlannerTools.getCoMVelocityFifthCoefficientTimeFunction(), 1e-3 * Math.max(c4_dot, 1.0));

         double c0_ddot = omega * omega * Math.exp(omega * time);
         double c1_ddot = omega * omega * Math.exp(-omega * time);
         double c2_ddot = 6.0 * time;
         double c3_ddot = 2.0;
         double c4_ddot = 0.0;
         double c5_ddot = 0.0;

         double c0_ddot_numerical = (CoMTrajectoryPlannerTools.getCoMVelocityFirstCoefficientTimeFunction(omega, time + dt) - CoMTrajectoryPlannerTools
               .getCoMVelocityFirstCoefficientTimeFunction(omega, time)) / dt;
         double c1_ddot_numerical = (CoMTrajectoryPlannerTools.getCoMVelocitySecondCoefficientTimeFunction(omega, time + dt) - CoMTrajectoryPlannerTools
               .getCoMVelocitySecondCoefficientTimeFunction(omega, time)) / dt;
         double c2_ddot_numerical = (CoMTrajectoryPlannerTools.getCoMVelocityThirdCoefficientTimeFunction(time + dt) - CoMTrajectoryPlannerTools
               .getCoMVelocityThirdCoefficientTimeFunction(time)) / dt;
         double c3_ddot_numerical = (CoMTrajectoryPlannerTools.getCoMVelocityFourthCoefficientTimeFunction(time + dt) - CoMTrajectoryPlannerTools
               .getCoMVelocityFourthCoefficientTimeFunction(time)) / dt;

         assertEquals(c0_ddot, CoMTrajectoryPlannerTools.getCoMAccelerationFirstCoefficientTimeFunction(omega, time), epsilon);
         assertEquals(c1_ddot, CoMTrajectoryPlannerTools.getCoMAccelerationSecondCoefficientTimeFunction(omega, time), epsilon);
         assertEquals(c2_ddot, CoMTrajectoryPlannerTools.getCoMAccelerationThirdCoefficientTimeFunction(time), epsilon);
         assertEquals(c3_ddot, CoMTrajectoryPlannerTools.getCoMAccelerationFourthCoefficientTimeFunction(), epsilon);
         assertEquals(c4_ddot, CoMTrajectoryPlannerTools.getCoMAccelerationFifthCoefficientTimeFunction(), epsilon);
         assertEquals(c5_ddot, CoMTrajectoryPlannerTools.getCoMAccelerationSixthCoefficientTimeFunction(), epsilon);
         assertEquals(c0_ddot_numerical, CoMTrajectoryPlannerTools.getCoMAccelerationFirstCoefficientTimeFunction(omega, time), 1e-3 * Math.max(c0_ddot, 1.0));
         assertEquals(c1_ddot_numerical, CoMTrajectoryPlannerTools.getCoMAccelerationSecondCoefficientTimeFunction(omega, time), 1e-3 * Math.max(c1_ddot, 1.0));
         assertEquals(c2_ddot_numerical, CoMTrajectoryPlannerTools.getCoMAccelerationThirdCoefficientTimeFunction(time), 1e-3 * Math.max(c2_ddot, 1.0));
         assertEquals(c3_ddot_numerical, CoMTrajectoryPlannerTools.getCoMAccelerationFourthCoefficientTimeFunction(), 1e-3 * Math.max(c3_dot, 1.0));

         double c0_dddot = Math.pow(omega, 3.0) * Math.exp(omega * time);
         double c1_dddot = -Math.pow(omega, 3.0) * Math.exp(-omega * time);
         double c2_dddot = 6.0;
         double c3_dddot = 0.0;
         double c4_dddot = 0.0;
         double c5_dddot = 0.0;
         double c0_dddot_numerical = (CoMTrajectoryPlannerTools.getCoMAccelerationFirstCoefficientTimeFunction(omega, time + dt) - CoMTrajectoryPlannerTools
               .getCoMAccelerationFirstCoefficientTimeFunction(omega, time)) / dt;
         double c1_dddot_numerical = (CoMTrajectoryPlannerTools.getCoMAccelerationSecondCoefficientTimeFunction(omega, time + dt) - CoMTrajectoryPlannerTools
               .getCoMAccelerationSecondCoefficientTimeFunction(omega, time)) / dt;
         double c2_dddot_numerical = (CoMTrajectoryPlannerTools.getCoMAccelerationThirdCoefficientTimeFunction(time + dt) - CoMTrajectoryPlannerTools
               .getCoMAccelerationThirdCoefficientTimeFunction(time)) / dt;


         assertEquals(c0_dddot, CoMTrajectoryPlannerTools.getCoMJerkFirstCoefficientTimeFunction(omega, time), epsilon);
         assertEquals(c1_dddot, CoMTrajectoryPlannerTools.getCoMJerkSecondCoefficientTimeFunction(omega, time), epsilon);
         assertEquals(c2_dddot, CoMTrajectoryPlannerTools.getCoMJerkThirdCoefficientTimeFunction(), epsilon);
         assertEquals(c3_dddot, CoMTrajectoryPlannerTools.getCoMJerkFourthCoefficientTimeFunction(), epsilon);
         assertEquals(c4_dddot, CoMTrajectoryPlannerTools.getCoMJerkFifthCoefficientTimeFunction(), epsilon);
         assertEquals(c5_dddot, CoMTrajectoryPlannerTools.getCoMJerkSixthCoefficientTimeFunction(), epsilon);
         assertEquals(c0_dddot_numerical, CoMTrajectoryPlannerTools.getCoMJerkFirstCoefficientTimeFunction(omega, time), 1e-3 * Math.max(c0_dddot, 1.0));
         assertEquals(c1_dddot_numerical, CoMTrajectoryPlannerTools.getCoMJerkSecondCoefficientTimeFunction(omega, time), 1e-3 * Math.max(c1_dddot, 1.0));
         assertEquals(c2_dddot_numerical, CoMTrajectoryPlannerTools.getCoMJerkThirdCoefficientTimeFunction(), 1e-3 * Math.max(c2_dddot, 1.0));

         assertEquals(c0 - 1 / (omega * omega) * c0_ddot, CoMTrajectoryPlannerTools.getVRPPositionFirstCoefficientTimeFunction(), epsilon);
         assertEquals(c1 - 1 / (omega * omega) * c1_ddot, CoMTrajectoryPlannerTools.getVRPPositionSecondCoefficientTimeFunction(), epsilon);
         assertEquals(c2 - 1 / (omega * omega) * c2_ddot, CoMTrajectoryPlannerTools.getVRPPositionThirdCoefficientTimeFunction(omega, time), epsilon);
         assertEquals(c3 - 1 / (omega * omega) * c3_ddot, CoMTrajectoryPlannerTools.getVRPPositionFourthCoefficientTimeFunction(omega, time), epsilon);
         assertEquals(c4 - 1 / (omega * omega) * c4_ddot, CoMTrajectoryPlannerTools.getVRPPositionFifthCoefficientTimeFunction(time), epsilon);
         assertEquals(c5 - 1 / (omega * omega) * c5_ddot, CoMTrajectoryPlannerTools.getVRPPositionSixthCoefficientTimeFunction(), epsilon);

         assertEquals(c0_dot - 1 / (omega * omega) * c0_dddot, CoMTrajectoryPlannerTools.getVRPVelocityFirstCoefficientTimeFunction(), epsilon);
         assertEquals(c1_dot - 1 / (omega * omega) * c1_dddot, CoMTrajectoryPlannerTools.getVRPVelocitySecondCoefficientTimeFunction(), epsilon);
         assertEquals(c2_dot - 1 / (omega * omega) * c2_dddot, CoMTrajectoryPlannerTools.getVRPVelocityThirdCoefficientTimeFunction(omega, time), epsilon);
         assertEquals(c3_dot - 1 / (omega * omega) * c3_dddot, CoMTrajectoryPlannerTools.getVRPVelocityFourthCoefficientTimeFunction(time), epsilon);
         assertEquals(c4_dot - 1 / (omega * omega) * c4_dddot, CoMTrajectoryPlannerTools.getVRPVelocityFifthCoefficientTimeFunction(), epsilon);
         assertEquals(c5_dot - 1 / (omega * omega) * c5_dddot, CoMTrajectoryPlannerTools.getVRPVelocitySixthCoefficientTimeFunction(), epsilon);

         assertEquals(c0 + 1 / omega * c0_dot, CoMTrajectoryPlannerTools.getDCMPositionFirstCoefficientTimeFunction(omega, time), epsilon);
         assertEquals(c1 + 1 / omega * c1_dot, CoMTrajectoryPlannerTools.getDCMPositionSecondCoefficientTimeFunction(), epsilon);
         assertEquals(c2 + 1 / omega * c2_dot, CoMTrajectoryPlannerTools.getDCMPositionThirdCoefficientTimeFunction(omega, time), epsilon);
         assertEquals(c3 + 1 / omega * c3_dot, CoMTrajectoryPlannerTools.getDCMPositionFourthCoefficientTimeFunction(omega, time), epsilon);
         assertEquals(c4 + 1 / omega * c4_dot, CoMTrajectoryPlannerTools.getDCMPositionFifthCoefficientTimeFunction(omega, time), epsilon);
         assertEquals(c5 + 1 / omega * c5_dot, CoMTrajectoryPlannerTools.getDCMPositionSixthCoefficientTimeFunction(), epsilon);
      }
   }

   @Test
   public void testTrajectoryConstruction()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1000; iter++)
      {
         double omega = RandomNumbers.nextDouble(random, 0.3, 2.0);
         double time = RandomNumbers.nextDouble(random, 0.0, 1.5);

         FramePoint3D c0 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D c1 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D c2 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D c3 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D c4 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D c5 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         FramePoint3D desiredCoMPosition = new FramePoint3D();
         FramePoint3D desiredDCMPosition = new FramePoint3D();
         FrameVector3D desiredCoMVelocity = new FrameVector3D();
         FrameVector3D desiredCoMAcceleration = new FrameVector3D();

         CoMTrajectoryPlannerTools.constructDesiredCoMPosition(desiredCoMPosition, c0, c1, c2, c3, c4, c5, time, omega);
         CoMTrajectoryPlannerTools.constructDesiredCoMVelocity(desiredCoMVelocity, c0, c1, c2, c3, c4, c5, time, omega);
         CoMTrajectoryPlannerTools.constructDesiredCoMAcceleration(desiredCoMAcceleration, c0, c1, c2, c3, c4, c5, time, omega);
         CapturePointTools.computeDesiredCapturePointPosition(desiredCoMPosition, desiredCoMVelocity, omega, desiredDCMPosition);

         FramePoint3D desiredCoMPositionExpected = new FramePoint3D();
         FramePoint3D desiredDCMPositionExpected = new FramePoint3D();
         FrameVector3D desiredCoMVelocityExpected = new FrameVector3D();
         FrameVector3D desiredCoMAccelerationExpected = new FrameVector3D();

         FramePoint3D temp = new FramePoint3D();

         // com position
         temp.set(c0);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionFirstCoefficientTimeFunction(omega, time));

         desiredCoMPositionExpected.add(temp);

         temp.set(c1);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionSecondCoefficientTimeFunction(omega, time));

         desiredCoMPositionExpected.add(temp);

         temp.set(c2);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionThirdCoefficientTimeFunction(time));

         desiredCoMPositionExpected.add(temp);

         temp.set(c3);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionFourthCoefficientTimeFunction(time));

         desiredCoMPositionExpected.add(temp);

         temp.set(c4);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionFifthCoefficientTimeFunction(time));

         desiredCoMPositionExpected.add(temp);

         temp.set(c5);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionSixthCoefficientTimeFunction());

         desiredCoMPositionExpected.add(temp);

         // com velocity
         temp.set(c0);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocityFirstCoefficientTimeFunction(omega, time));

         desiredCoMVelocityExpected.add(temp);

         temp.set(c1);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocitySecondCoefficientTimeFunction(omega, time));

         desiredCoMVelocityExpected.add(temp);

         temp.set(c2);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocityThirdCoefficientTimeFunction(time));

         desiredCoMVelocityExpected.add(temp);

         temp.set(c3);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocityFourthCoefficientTimeFunction(time));

         desiredCoMVelocityExpected.add(temp);

         temp.set(c4);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocityFifthCoefficientTimeFunction());

         desiredCoMVelocityExpected.add(temp);

         temp.set(c5);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocitySixthCoefficientTimeFunction());

         desiredCoMVelocityExpected.add(temp);

         // com acceleration
         temp.set(c0);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationFirstCoefficientTimeFunction(omega, time));

         desiredCoMAccelerationExpected.add(temp);

         temp.set(c1);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationSecondCoefficientTimeFunction(omega, time));

         desiredCoMAccelerationExpected.add(temp);

         temp.set(c2);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationThirdCoefficientTimeFunction(time));

         desiredCoMAccelerationExpected.add(temp);

         temp.set(c3);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationFourthCoefficientTimeFunction());

         desiredCoMAccelerationExpected.add(temp);

         temp.set(c4);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationFifthCoefficientTimeFunction());

         desiredCoMAccelerationExpected.add(temp);

         temp.set(c5);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationSixthCoefficientTimeFunction());

         desiredCoMAccelerationExpected.add(temp);

         // dcm position
         temp.set(c0);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionFirstCoefficientTimeFunction(omega, time));

         desiredDCMPositionExpected.add(temp);

         temp.set(c1);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionSecondCoefficientTimeFunction());

         desiredDCMPositionExpected.add(temp);

         temp.set(c2);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionThirdCoefficientTimeFunction(omega, time));

         desiredDCMPositionExpected.add(temp);

         temp.set(c3);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionFourthCoefficientTimeFunction(omega, time));

         desiredDCMPositionExpected.add(temp);

         temp.set(c4);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionFifthCoefficientTimeFunction(omega, time));

         desiredDCMPositionExpected.add(temp);

         temp.set(c5);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionSixthCoefficientTimeFunction());

         desiredDCMPositionExpected.add(temp);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredCoMPositionExpected, desiredCoMPosition, epsilon);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredCoMVelocityExpected, desiredCoMVelocity, epsilon);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredCoMAccelerationExpected, desiredCoMAcceleration, epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCMPositionExpected, desiredDCMPosition, epsilon);
      }
   }

   private static void checkPlannerDynamics(CoMTrajectoryPlanner planner, double omega)
   {
      FramePoint3D constructedDCMPosition = new FramePoint3D();
      constructedDCMPosition.scaleAdd(1.0 / omega, planner.getDesiredCoMVelocity(), planner.getDesiredCoMPosition());

      FrameVector3D constructedDCMVelocity = new FrameVector3D();
      constructedDCMVelocity.set(constructedDCMPosition);
      constructedDCMVelocity.sub(planner.getDesiredVRPPosition());
      constructedDCMVelocity.scale(omega);

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals("dcm dynamics are wrong.", constructedDCMPosition, planner.getDesiredDCMPosition(), epsilon);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("vrp dynamics are wrong.", constructedDCMVelocity, planner.getDesiredDCMVelocity(), epsilon);

   }
}
