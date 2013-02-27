package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.projectM.R2Sim02.initialSetup.GuiInitialSetup;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.FlatGroundProfile;

public class DRCGuiInitialSetup implements GuiInitialSetup
{
   private boolean isGuiShown = true;
   private final boolean groundProfileVisible;
   private final boolean drawPlaneAtZ0;
   
   public DRCGuiInitialSetup(boolean groundProfileVisible, boolean drawPlaneAtZeroHeight)
   {
      this.groundProfileVisible = groundProfileVisible;
      this.drawPlaneAtZ0 = drawPlaneAtZeroHeight;
   }
   
   public void initializeGUI(SimulationConstructionSet scs, Robot robot)
   {
      CameraConfiguration behindPelvis = new CameraConfiguration("BehindPelvis");
      behindPelvis.setCameraTracking(false, true, true, false);
      behindPelvis.setCameraDolly(false, true, true, false);
      behindPelvis.setCameraFix(0.0, 0.0, 1.0);
      behindPelvis.setCameraPosition(-2.5, 0.0, 1.0);
      behindPelvis.setCameraTrackingVars("q_x", "q_y", "q_z");
      scs.setupCamera(behindPelvis);

      CameraConfiguration camera5 = new CameraConfiguration("stereo_camera");
      camera5.setCameraMount("stereo_camera");
      scs.setupCamera(camera5);
//      CameraConfiguration camera6 = new CameraConfiguration("right_camera_sensor");
//      camera6.setCameraMount("right_camera_sensor");
//      scs.setupCamera(camera6);
      
      scs.setGroundVisible(groundProfileVisible);
      
      if (drawPlaneAtZ0)
      {
         Graphics3DObject planeAtZ0 = new Graphics3DObject();
         planeAtZ0.addHeightMap(new FlatGroundProfile(), 100, 100, null);
         scs.addStaticLinkGraphics(planeAtZ0);
      }
   }

   public boolean isGuiShown()
   {
      return isGuiShown;
   }
   
   public void setIsGuiShown(boolean isGuiShown)
   {
      this.isGuiShown = isGuiShown;
   }
}
