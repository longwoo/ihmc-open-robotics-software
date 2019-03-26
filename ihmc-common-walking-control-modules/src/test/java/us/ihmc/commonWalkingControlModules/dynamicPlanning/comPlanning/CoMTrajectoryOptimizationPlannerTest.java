package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

public class CoMTrajectoryOptimizationPlannerTest extends CoMTrajectoryPlannerInterfaceTest
{
   protected CoMTrajectoryPlannerInterface createComTrajectoryPlanner()
   {
      return new CoMTrajectoryOptimizationPlanner(omega, gravityZ, nominalHeight, registry);
   }
}
