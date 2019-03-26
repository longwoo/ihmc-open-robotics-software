package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

public class CoMTrajectoryPlannerTest extends CoMTrajectoryPlannerInterfaceTest
{
   protected CoMTrajectoryPlannerInterface createComTrajectoryPlanner()
   {
      return new CoMTrajectoryPlanner(omega, gravityZ, nominalHeight, registry);
   }
}
