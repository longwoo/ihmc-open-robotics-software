package com.yobotics.simulationconstructionset;

import static org.junit.Assert.*;

import java.awt.AWTException;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import com.yobotics.simulationconstructionset.gui.SimulationGUITestFixture;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.utilities.ThreadTools;

public class SimulationConstructionSetTest
{
   
   @Test
   public void testSimulationConstructionSetUsingDirectCalls() throws AWTException
   {
      SimpleRobot simpleRobot = new SimpleRobot();
      
      SimulationConstructionSet scs = new SimulationConstructionSet(simpleRobot);
      scs.setFrameMaximized();
      scs.startOnAThread();
      
      double simulateTime = 1.0;
      
      double startTime = scs.getTime();
      scs.simulate(simulateTime);
      while(scs.isSimulating())
      {
         ThreadTools.sleep(100L);
      }
      
      double endTime = scs.getTime();

      assertEquals(simulateTime, endTime-startTime, 1e-7);

//      ThreadTools.sleepForever();
      
      scs.closeAndDispose();
      scs = null;
   }
   
   @Test
   public void testSimulationConstructionSetUsingGUITestFixture() throws AWTException
   {
      SimpleRobot simpleRobot = new SimpleRobot();
      
      SimulationConstructionSet scs = new SimulationConstructionSet(simpleRobot);
      scs.setFrameMaximized();
      scs.startOnAThread();
      
      
      SimulationGUITestFixture testFixture = new SimulationGUITestFixture(scs);
      
      
      testFixture.selectNameSpaceTab();
      ThreadTools.sleep(1000);
      
      testFixture.selectSearchTab();
      
      testFixture.deleteSearchText();
      testFixture.enterSearchText("q_");
      
      testFixture.selectVariableInSearchTab("q_x");
      testFixture.clickRemoveEmptyGraphButton();
      testFixture.clickNewGraphButton();
      testFixture.middleClickInEmptyGraph();
      
      testFixture.clickNewGraphButton();
      testFixture.clickNewGraphButton();
      testFixture.selectVariableInSearchTab("q_z");
      testFixture.middleClickInNthGraph(2);
      
      testFixture.clickSimulateButton();
      ThreadTools.sleep(500);
      testFixture.clickStopButton();
      testFixture.clickPlayButton();
      
      
      testFixture.removeVariableFromNthGraph("q_z", 2);
      testFixture.clickRemoveEmptyGraphButton();


      ThreadTools.sleep(500);
      testFixture.clickStopButton();

      testFixture.clickGotoInPointButton();

      ThreadTools.sleep(1000);

      int index = scs.getIndex();
      int inPoint = scs.getInPoint();
      assertEquals(index, inPoint);
      
      int stepsForward = 8;
      for (int i=0; i<stepsForward; i++)
      {
         testFixture.clickStepForwardButton();
      }
      index = scs.getIndex();
      assertEquals(stepsForward, index);
      testFixture.clickAddKeyPointButton();
      
      for (int i=0; i<stepsForward; i++)
      {
         testFixture.clickStepForwardButton();
      }
      index = scs.getIndex();
      assertEquals(2*stepsForward, index);
      testFixture.clickAddKeyPointButton();

      testFixture.clickZoomInButton();
      testFixture.clickZoomInButton();
      testFixture.clickZoomInButton();
      testFixture.clickZoomInButton();
      testFixture.clickZoomOutButton();
      
      
      testFixture.clickGotoInPointButton();
      testFixture.clickToggleKeyModeButton();
      
      testFixture.clickStepForwardButton();
      index = scs.getIndex();
      assertEquals(stepsForward, index);
      
      testFixture.clickStepForwardButton();
      index = scs.getIndex();
      assertEquals(2*stepsForward, index);
      
      
      testFixture.closeAndDispose();
      scs.closeAndDispose();
      scs = null;
      testFixture = null;
   }
  


   public static class SimpleRobot extends Robot
   {
      private static final long serialVersionUID = 43883985473093746L;

      public SimpleRobot()
      {
         super("SimpleRobot");
         
         FloatingJoint rootJoint = new FloatingJoint("root", new Vector3d(), this);
         Link body = new Link("body");
         body.setMassAndRadiiOfGyration(1.0, 0.1, 0.1, 0.1);
         
         rootJoint.setPosition(new Point3d(0.1, 0.2, 1.2));
         
         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.addCube(0.1, 0.1, 0.1);
         
         body.setLinkGraphics(linkGraphics);
         rootJoint.setLink(body);
         
         this.addRootJoint(rootJoint);
      }
   }

}
