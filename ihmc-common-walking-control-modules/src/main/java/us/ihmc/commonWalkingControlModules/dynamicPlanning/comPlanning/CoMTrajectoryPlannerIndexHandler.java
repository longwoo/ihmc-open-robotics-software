package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import gnu.trove.list.array.TIntArrayList;
import gnu.trove.map.hash.TIntIntHashMap;

import java.util.List;

/**
 * Provides the internal indices for the constraint matrices for the {@link CoMTrajectoryPlanner}.
 */
public class CoMTrajectoryPlannerIndexHandler
{
   private static final int sequenceSize = 6;
   private static final int vrpWaypointSize = 4;

   private int size;
   private int numberOfVRPWaypoints;
   private final TIntIntHashMap vrpWaypointIndices = new TIntIntHashMap();

   public void update(List<? extends ContactStateProvider> contactSequence)
   {
      vrpWaypointIndices.clear();
      size = 0;
      numberOfVRPWaypoints = 0;
      if (contactSequence.get(0).getContactState() == ContactState.IN_CONTACT)
      {
         vrpWaypointIndices.put(0, numberOfVRPWaypoints);
         numberOfVRPWaypoints += vrpWaypointSize; // start and end
      }
      size += sequenceSize;
      for (int sequenceId = 1; sequenceId < contactSequence.size(); sequenceId++)
      {
         if (contactSequence.get(sequenceId).getContactState() == ContactState.IN_CONTACT)
         {
            vrpWaypointIndices.put(sequenceId, numberOfVRPWaypoints);
            numberOfVRPWaypoints += vrpWaypointSize;
         }
         size += sequenceSize;
      }
   }

   public int getTotalSize()
   {
      return size;
   }

   public int getNumberOfVRPWaypoints()
   {
      return numberOfVRPWaypoints;
   }

   public int getContactSequenceStartIndex(int sequenceNumber)
   {
      return 6 * sequenceNumber;
   }

   public int getVRPWaypointStartPositionIndex(int sequenceNumber)
   {
      return vrpWaypointIndices.get(sequenceNumber);
   }

   public int getVRPWaypointStartVelocityIndex(int sequenceNumber)
   {
      return vrpWaypointIndices.get(sequenceNumber) + 1;
   }

   public int getVRPWaypointFinalPositionIndex(int sequenceNumber)
   {
      return vrpWaypointIndices.get(sequenceNumber) + 2;
   }

   public int getVRPWaypointFinalVelocityIndex(int sequenceNumber)
   {
      return vrpWaypointIndices.get(sequenceNumber) + 3;
   }
}
