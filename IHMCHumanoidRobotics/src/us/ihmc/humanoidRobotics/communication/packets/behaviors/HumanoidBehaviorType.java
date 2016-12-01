package us.ihmc.humanoidRobotics.communication.packets.behaviors;

public enum HumanoidBehaviorType
{
   STOP, TEST, WALK_TO_LOCATION, WALK_TO_GOAL, DIAGNOSTIC, PICK_UP_BALL,TURN_VALVE,
   EXAMPLE_BEHAVIOR, BALL_DETECTION, TEST_PIPELINE, TEST_STATEMACHINE,
   FOLLOW_FIDUCIAL_50, LOCATE_FIDUCIAL, WAlK_OVER_TERRAIN, FOLLOW_VALVE, LOCATE_VALVE, WALK_OVER_TERRAIN_TO_VALVE;

   public static final HumanoidBehaviorType[] values = values();
}
