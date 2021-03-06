package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.DoorLocationPacket;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkThroughDoorBehavior.WalkThroughDoorBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.GoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.primitives.TimingBehaviorHelper;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkThroughDoorBehavior extends StateMachineBehavior<WalkThroughDoorBehaviorState>
{
   public enum WalkThroughDoorBehaviorState
   {
      STOPPED,
      SETUP_ROBOT,
      //clear planar regions and look down, then up only come here after a failed plan
      SEARCHING_FOR_DOOR, //search for general door location
      //if distance to door < approach point location skip this step.
      //walk to general door location... point offseet from door approach location
      //search for door precise location
      WALKING_TO_DOOR, //if failed jump to clear state
      SEARCHING_FOR_DOOR_FINAL, //search for door handle
      OPEN_DOOR, // in paralelle run the detect open door behavior if open door is not detected, go back to search for door 
      SET_UP_ROBOT_FOR_DOOR_WALK,
      WAITING_FOR_USER_CONFIRMATION,
      WALK_THROUGH_DOOR,
      RESET_ROBOT,
      FAILED,
      DONE
   }

   //do you want to tuck in the arms before walking through the door
   private final boolean setUpArms = true;

   //this is the predefined walk to points relative to the door reference frame, these should eventualy be replaced by a behavior that finds the best location to walk up to given an arm task space 
   private Vector3D32 doorOffsetPoint1 = new Vector3D32(0.5f, -0.9f, 0f);
   private Vector3D32 doorOffsetPoint2 = new Vector3D32(0.5f, -0.6f, 0f);

   //define some of the sub-behaviors that will be used that are specific to this behavior
   private final SearchForDoorBehavior searchForDoorBehavior;
   private final OpenDoorBehavior openDoorBehavior;
   private final WalkToInteractableObjectBehavior walkToInteractableObjectBehavior;
   private final ResetRobotBehavior resetRobotBehavior;

   //this hold all the primitive behaviors that get used across most behaviors.
   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private SleepBehavior sleepBehavior;
   //sends out a door location packet for use in debugging. not really necesary until the door is found from a behavior instead of the user supplying its location

   // private BasicTimingBehavior basicTimingBehavior;

   public WalkThroughDoorBehavior(String robotName, String yoNamePrefix, Ros2Node ros2Node, YoDouble yoTime, YoBoolean yoDoubleSupport,
                                  FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames,
                                  WholeBodyControllerParameters wholeBodyControllerParameters, AtlasPrimitiveActions atlasPrimitiveActions,
                                  YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(robotName, "walkThroughDoorBehavior", WalkThroughDoorBehaviorState.class, yoTime, ros2Node);
      sleepBehavior = new SleepBehavior(robotName, ros2Node, yoTime);

      this.atlasPrimitiveActions = atlasPrimitiveActions;
      //    basicTimingBehavior = new BasicTimingBehavior(robotName, ros2Node);
      //set up behaviors
      searchForDoorBehavior = new SearchForDoorBehavior(robotName, yoNamePrefix, ros2Node, yoGraphicsListRegistry);
      walkToInteractableObjectBehavior = new WalkToInteractableObjectBehavior(robotName, yoTime, ros2Node, atlasPrimitiveActions);

      openDoorBehavior = new OpenDoorBehavior(robotName, yoTime, ros2Node, atlasPrimitiveActions);
      resetRobotBehavior = new ResetRobotBehavior(robotName, ros2Node, yoTime);

      //setup publisher for sending door location to UI
      setupStateMachine();
   }

   @Override
   public void doControl()
   {
      //should constantly be searching for door and updating its location here
      super.doControl();
      //  basicTimingBehavior.doControl();
   }

   @Override
   public void onBehaviorEntered()
   {
      publishTextToSpeech("Entering Walk Through Door behavior");

      super.onBehaviorEntered();
   }

   @Override
   protected WalkThroughDoorBehaviorState configureStateMachineAndReturnInitialKey(StateMachineFactory<WalkThroughDoorBehaviorState, BehaviorAction> factory)
   {
      //reset the robot in case it is in a wierd configuration before the behavior starts
      BehaviorAction resetRobot = new BehaviorAction(resetRobotBehavior);

      //if there are hands, close them
      BehaviorAction setup = new BehaviorAction(atlasPrimitiveActions.leftHandDesiredConfigurationBehavior,
                                                atlasPrimitiveActions.rightHandDesiredConfigurationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            HandDesiredConfigurationMessage leftHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.LEFT,
                                                                                                                         HandConfiguration.CLOSE);
            HandDesiredConfigurationMessage rightHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.RIGHT,
                                                                                                                          HandConfiguration.CLOSE);

            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);

            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(leftHandMessage);
         }

         @Override
         public void doTransitionIntoAction()
         {

            super.doTransitionIntoAction();
         }
      };

      //this is the first search for the door, once automated searching is in place, this should be an all the time thing.
      BehaviorAction searchForDoorFar = new BehaviorAction(searchForDoorBehavior)
      {

         @Override
         public void doTransitionIntoAction()
         {
            publishTextToSpeech("Searching For The Door");
            super.doTransitionIntoAction();
         }

         @Override
         public void doTransitionOutOfAction()
         {
            super.doTransitionOutOfAction();
            //found the door location, inform the UI of its location

         }
      };

      BehaviorAction searchForDoorNear = new BehaviorAction(searchForDoorBehavior)
      {
         @Override
         public void doTransitionOutOfAction()
         {
            super.doTransitionOutOfAction();
            //found the door location, inform the UI of its location

         }

         @Override
         public void doTransitionIntoAction()
         {
            publishTextToSpeech("Confirm door location before walking through");

            super.doTransitionIntoAction();
         }
      };

      BehaviorAction walkToDoorAction = new BehaviorAction(walkToInteractableObjectBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePoint3D point1 = offsetPointFromDoor(doorOffsetPoint1);
            FramePoint3D point2 = offsetPointFromDoor(doorOffsetPoint2);

            walkToInteractableObjectBehavior.setWalkPoints(point1, point2);
         }
      };

      BehaviorAction openDoorAction = new BehaviorAction(openDoorBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            openDoorBehavior.setGrabLocation(searchForDoorBehavior.getLocation());
         }
      };

      BehaviorAction setUpForWalk = new BehaviorAction(atlasPrimitiveActions.leftArmTrajectoryBehavior, atlasPrimitiveActions.rightArmTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

            double[] rightArmPose = new double[] {1.5708, 0.8226007082651046, 1.2241049170121854, -1.546127437107859, -0.8486641166791746, -1.3365746544030488,
                  1.3376930879072813};
            double[] leftArmPose = new double[] {-1.5383305366909918, -0.9340404711083553, 1.9634792241521146, 0.9236260708644913, -0.8710518130931819,
                  -0.8771109242461594, -1.336089159719967};

            ArmTrajectoryMessage rightPoseMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, 10, rightArmPose);

            ArmTrajectoryMessage leftPoseMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT, 10, leftArmPose);

            atlasPrimitiveActions.leftArmTrajectoryBehavior.setInput(leftPoseMessage);
            atlasPrimitiveActions.rightArmTrajectoryBehavior.setInput(rightPoseMessage);
         }
      };

      BehaviorAction walkThroughDoor = new BehaviorAction(atlasPrimitiveActions.footstepListBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FootstepDataListMessage message = setUpFootSteps();
            atlasPrimitiveActions.footstepListBehavior.set(message);
         }
      };

      BehaviorAction failedState = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Walking Through Door Failed");
         }
      };

      BehaviorAction doneState = new BehaviorAction(sleepBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            sleepBehavior.setSleepTime(3000);
            publishTextToSpeech("Finished Walking Through Door");
         }
      };

      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.SETUP_ROBOT, setup, WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR);
      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR, searchForDoorFar, WalkThroughDoorBehaviorState.WALKING_TO_DOOR);

      factory.addState(WalkThroughDoorBehaviorState.WALKING_TO_DOOR, walkToDoorAction);
      factory.addTransition(WalkThroughDoorBehaviorState.WALKING_TO_DOOR, WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR_FINAL,
                            t -> isWalkingDone() && hasWalkingSucceded());
      factory.addTransition(WalkThroughDoorBehaviorState.WALKING_TO_DOOR, WalkThroughDoorBehaviorState.FAILED, t -> isWalkingDone() && !hasWalkingSucceded());

      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR_FINAL, searchForDoorNear, WalkThroughDoorBehaviorState.OPEN_DOOR);
      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.OPEN_DOOR, openDoorAction,
                                        setUpArms ? WalkThroughDoorBehaviorState.SET_UP_ROBOT_FOR_DOOR_WALK : WalkThroughDoorBehaviorState.WALK_THROUGH_DOOR);

      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.SET_UP_ROBOT_FOR_DOOR_WALK, setUpForWalk, WalkThroughDoorBehaviorState.WALK_THROUGH_DOOR);
      //factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.WALK_THROUGH_DOOR, walkThroughDoor, WalkThroughDoorBehaviorState.RESET_ROBOT);
      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.WALK_THROUGH_DOOR, walkThroughDoor, WalkThroughDoorBehaviorState.DONE);

      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.RESET_ROBOT, resetRobot, WalkThroughDoorBehaviorState.DONE);
      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.FAILED, failedState, WalkThroughDoorBehaviorState.DONE);
      factory.addState(WalkThroughDoorBehaviorState.DONE, doneState);

      return WalkThroughDoorBehaviorState.SETUP_ROBOT;
   }

   private FramePoint3D offsetPointFromDoor(Vector3D32 point)
   {

      PoseReferenceFrame doorPose = new PoseReferenceFrame("doorFrame", ReferenceFrame.getWorldFrame());
      doorPose.setPoseAndUpdate(new Pose3D(searchForDoorBehavior.getLocation()));

      FramePoint3D point1 = new FramePoint3D(doorPose, point);
      return point1;
   }

   public FootstepDataListMessage setUpFootSteps()
   {

      PoseReferenceFrame doorPose = new PoseReferenceFrame("DoorReferenceFrame", ReferenceFrame.getWorldFrame());

      Pose3D unrotatedDoor = new Pose3D(searchForDoorBehavior.getLocation());

      unrotatedDoor.appendYawRotation(Math.toRadians(180));
      unrotatedDoor.appendTranslation(-0.9144, 0, 0);

      doorPose.setPoseAndUpdate(unrotatedDoor);

      RobotSide startStep = RobotSide.LEFT;

      //All z values were 0.11830896252372711 changing to 0 for current test until i can get them to automaticaly snap to the ground.

      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(atlasPrimitiveActions.footstepListBehavior.getDefaultSwingTime(),
                                                                                           atlasPrimitiveActions.footstepListBehavior.getDefaultTranferTime());

      FootstepDataMessage fs1 = createRelativeFootStep(doorPose, startStep, new Point3D(0.5864031335585762, 0.592160790421584, 0),
                                                       new Quaternion(-4.624094786785623E-5, 3.113506928734585E-6, -0.7043244487834723, 0.7098782069467541));

      FootstepDataMessage fs2 = createRelativeFootStep(doorPose, startStep.getOppositeSide(), new Point3D(0.4053278408799188, 0.23597592988662308, 0),
                                                       new Quaternion(-1.5943418991263463E-13, 2.75059506574629E-13, -0.7043243641759355, 0.7098782924052293));
      FootstepDataMessage fs3 = createRelativeFootStep(doorPose, startStep, new Point3D(0.5924372369454293, -0.26851462759487155, 0),
                                                       new Quaternion(-3.236982396751798E-13, 3.899712427026468E-14, -0.7043243760613419, 0.7098782806128114));
      FootstepDataMessage fs4 = createRelativeFootStep(doorPose, startStep.getOppositeSide(), new Point3D(0.36887783182356804, -0.7234607322382425, 0),
                                                       new Quaternion(1.7351711631778928E-14, -1.6924263791365571E-13, -0.7043243760613419,
                                                                      0.7098782806128114));
      FootstepDataMessage fs5 = createRelativeFootStep(doorPose, startStep, new Point3D(0.5896714303877739, -0.7199905519593679, 0),
                                                       new Quaternion(2.5501844493298926E-13, -3.0463423083022023E-13, -0.7043243760613419,
                                                                      0.7098782806128114));

      message.getFootstepDataList().add().set(fs1);
      message.getFootstepDataList().add().set(fs2);
      message.getFootstepDataList().add().set(fs3);
      message.getFootstepDataList().add().set(fs4);
      message.getFootstepDataList().add().set(fs5);

      return message;

   }

   private FootstepDataMessage createRelativeFootStep(PoseReferenceFrame frame, RobotSide side, Point3D location, Quaternion orientation)
   {

      FramePose3D pose = offsetPointFromFrameInWorldFrame(frame, location, orientation);
      FootstepDataMessage message = HumanoidMessageTools.createFootstepDataMessage(side, pose.getPosition(), pose.getOrientation());
      return message;
   }

   private FramePose3D offsetPointFromFrameInWorldFrame(PoseReferenceFrame frame, Point3D point3d, Quaternion quat4d)
   {
      FramePoint3D point1 = new FramePoint3D(frame, point3d);
      point1.changeFrame(ReferenceFrame.getWorldFrame());
      FrameQuaternion orient = new FrameQuaternion(frame, quat4d);
      orient.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose3D pose = new FramePose3D(point1, orient);

      return pose;
   }

   @Override
   public void onBehaviorExited()
   {
      publishTextToSpeech("Leaving Walk Through Door behavior");

   }

   private boolean isWalkingDone()
   {
      return walkToInteractableObjectBehavior.isDone();
   }

   private boolean hasWalkingSucceded()
   {
      return walkToInteractableObjectBehavior.succeded();
   }

}
