package us.ihmc.atlas;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCHighLevelPositionControlDemo;
import us.ihmc.darpaRoboticsChallenge.DRCLocalConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotInterface;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModelFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.PlainDRCRobot;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.visualization.SliderBoardFactory;
import us.ihmc.darpaRoboticsChallenge.visualization.WalkControllerSliderBoard;
import us.ihmc.graphics3DAdapter.GroundProfile;

import com.martiansoftware.jsap.JSAPException;
import com.yobotics.simulationconstructionset.util.FlatGroundProfile;

public class AtlasHighLevelPositionControlDemo
{
   
 private static final DRCRobotModel defaultModelForGraphicSelector = new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS, DRCLocalConfigParameters.RUNNING_ON_REAL_ROBOT);
 private static final double ROBOT_FLOATING_HEIGHT = 0.3;
   
   public static void main(String[] args) throws JSAPException
   {
      DRCRobotModel model = null;
      
      model = DRCRobotModelFactory.selectModelFromFlag(args);
      
      if (model == null)
         model = DRCRobotModelFactory.selectModelFromGraphicSelector(defaultModelForGraphicSelector);

      if (model == null)
         throw new RuntimeException("No robot model selected");
      
      AutomaticSimulationRunner automaticSimulationRunner = null;

      SliderBoardFactory sliderBoardFactory = WalkControllerSliderBoard.getFactory();
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false, sliderBoardFactory);

      DRCRobotInterface robotInterface = new PlainDRCRobot(model);
      
      final double groundHeight = 0.0;
      GroundProfile groundProfile = new FlatGroundProfile(groundHeight);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotInterface.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);
      
      double initialYaw = 0.0;
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = model.getDefaultRobotInitialSetup(groundHeight + ROBOT_FLOATING_HEIGHT, initialYaw);

      WalkingControllerParameters drcControlParameters = model.getWalkingControlParameters();
      ArmControllerParameters armControlParameters = model.getArmControllerParameters();
      
      new DRCHighLevelPositionControlDemo(drcControlParameters, armControlParameters, robotInterface, robotInitialSetup, guiInitialSetup, scsInitialSetup,
                                    automaticSimulationRunner, DRCConfigParameters.CONTROL_DT, 16000, model);
   }
}
