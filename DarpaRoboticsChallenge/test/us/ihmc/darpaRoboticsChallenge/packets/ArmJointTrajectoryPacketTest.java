package us.ihmc.darpaRoboticsChallenge.packets;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.packets.manipulation.ArmJointTrajectoryPacket;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations.ArmConfigurations;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class ArmJointTrajectoryPacketTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   
   // allowed deviations on the simulated robot from the trajectory for position and velocity:
   private final static double epsilonQ = 0.05;
   private final static double epsilonQd = 0.1;
   
   private final long seed = 126497;
   Random random = new Random(seed);

   static
   {
      simulationTestingParameters.setKeepSCSUp(false);
   }
   
   public abstract DRCRobotModel getRobotModel();
   
   @EstimatedDuration(duration = 30.0)
   @Test(timeout = 90000)
   public void testPacketRight() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      int waypoints = 4;
      int armJoints = getRobotModel().getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(ArmConfigurations.HOME, RobotSide.LEFT).length;
      double[] rightArmHome = getRobotModel().getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(ArmConfigurations.HOME, RobotSide.RIGHT);

      ArmJointTrajectoryPacket packet = new ArmJointTrajectoryPacket(RobotSide.RIGHT, waypoints, armJoints);
      
      for (int i = 0; i < armJoints; i++)
      {
         packet.trajectoryPoints[0].positions[i] = 0.0;
         packet.trajectoryPoints[1].positions[i] = 0.0;
         packet.trajectoryPoints[2].positions[i] = 0.0;
         packet.trajectoryPoints[3].positions[i] = rightArmHome[i];
      }
      
      packet.trajectoryPoints[0].time = 2.0;
      packet.trajectoryPoints[1].time = 3.0;
      packet.trajectoryPoints[2].time = 4.0;
      packet.trajectoryPoints[3].time = 6.0;
      
      packet.trajectoryPoints[1].positions[3] = -0.5;
      packet.trajectoryPoints[1].velocities[3] = -0.5;
      
      packet.trajectoryPoints[2].positions[3] = -1.0;
      
      executePacket(packet);
   }

   private void executePacket(ArmJointTrajectoryPacket packet) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      drcSimulationTestHelper.sendArmJointTrajectoryPacketToListeners(packet);
      
      int waypoints = packet.trajectoryPoints.length;
      
      for (int i = 0; i < waypoints; i++)
      {
         System.out.println("Starting execution of waypoint " + (i+1) + "/" + waypoints + "...");
         
         double startWaypointTime;
         double endWaypointTime = packet.trajectoryPoints[i].time;
         if (i == 0)
         {
            startWaypointTime = 0.0;
         }
         else
         {
            startWaypointTime = packet.trajectoryPoints[i-1].time;
         }
         
         drcSimulationTestHelper.simulateAndBlock(endWaypointTime - startWaypointTime);
         System.out.println("done - check if target reached");
         
         SDFFullRobotModel fullRobotModel = drcSimulationTestHelper.getSDFFullRobotModel();
         SDFRobot sdfRobot = drcSimulationTestHelper.getRobot();
         
         ArrayList<OneDoFJoint> armJoints = fullRobotModel.armJointIDsList.get(packet.robotSide);
         
         for (int jointIdx = 0; jointIdx < armJoints.size(); jointIdx++)
         {
            OneDegreeOfFreedomJoint joint = sdfRobot.getOneDegreeOfFreedomJoint(armJoints.get(jointIdx).getName());
            
//            System.out.println(armJoints.get(jointIdx).getName() + " performance at waypoint " + i + ":");
//            System.out.println("expected position: " + packet.trajectoryPoints[i].positions[jointIdx] + " actual was: " + joint.getQ().getDoubleValue());
//            System.out.println("expected velocity: " + packet.trajectoryPoints[i].velocities[jointIdx] + " actual was: " + joint.getQD().getDoubleValue());
            
            assertEquals(packet.trajectoryPoints[i].positions[jointIdx], joint.getQ().getDoubleValue(), epsilonQ);
            assertEquals(packet.trajectoryPoints[i].velocities[jointIdx], joint.getQD().getDoubleValue(), epsilonQd);

         }
         
         System.out.println("success");
      }
   }

   @Before
   public void setUp() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage();
      
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWholeBodyTrajectoryPacketTest", null, DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.simulateAndBlock(1.0);
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      BambooTools.reportTestFinishedMessage();
      
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      GlobalTimer.clearTimers();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
