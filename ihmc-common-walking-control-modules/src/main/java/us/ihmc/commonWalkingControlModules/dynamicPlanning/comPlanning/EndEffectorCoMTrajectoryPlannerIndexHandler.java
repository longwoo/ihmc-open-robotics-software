package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import gnu.trove.map.hash.TIntIntHashMap;

import java.util.List;

public class EndEffectorCoMTrajectoryPlannerIndexHandler extends CoMTrajectoryPlannerIndexHandler
{
   private int numberOfEndEffectorVRPWaypoints;

   @Override
   public void update(List<? extends ContactStateProvider> contactSequence)
   {
      super.update(contactSequence);

      numberOfEndEffectorVRPWaypoints = 0;
      if (contactSequence.get(0).getContactState() == ContactState.IN_CONTACT)
      {
         numberOfEndEffectorVRPWaypoints += contactSequence.get(0).getNumberOfBodiesInContact() * vrpConstraintsPerSegment; // start and end
      }
      for (int sequenceId = 1; sequenceId < contactSequence.size(); sequenceId++)
      {
         if (contactSequence.get(sequenceId).getContactState() == ContactState.IN_CONTACT)
         {
            numberOfEndEffectorVRPWaypoints += contactSequence.get(sequenceId).getNumberOfBodiesInContact() * vrpConstraintsPerSegment;
         }
      }
   }



   public int getNumberOfEndEffectorVRPWaypoints()
   {
      return numberOfEndEffectorVRPWaypoints;
   }

}
