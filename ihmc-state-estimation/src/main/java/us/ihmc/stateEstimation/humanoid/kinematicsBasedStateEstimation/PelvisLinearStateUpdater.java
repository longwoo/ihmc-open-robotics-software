package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.*;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.math.filters.GlitchFilteredYoInteger;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * PelvisLinearUpdater estimates the pelvis position and linear velocity using leg kinematics and IMU acceleration data.
 * If enabled, it also estimates the IMU drift on the estimation of the pelvis yaw angle and velocity.
 * @author Sylvain
 *
 */
public class PelvisLinearStateUpdater
{
   private static final double minForceZInPercentThresholdToFilterFoot = 0.0;
   private static final double maxForceZInPercentThresholdToFilterFoot = 0.45;

   private static Comparator<DoubleProvider> footSpeedComparator = (DoubleProvider footASpeed, DoubleProvider footBSpeed) -> {
      double speedA = footASpeed.getValue();
      double speedB = footBSpeed.getValue();
      if (Double.isNaN(speedA) && Double.isNaN(speedB))
         return 0;
      else if (Double.isNaN(speedA))
         return -1;
      else if (Double.isNaN(speedB))
         return 1;
      else
         return Double.compare(speedA, speedB);
   };

   private static boolean VISUALIZE = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final CenterOfMassCalculator centerOfMassCalculator;
   private final CenterOfMassJacobian centerOfMassJacobianWorld;

   private final List<RigidBodyBasics> feet = new ArrayList<RigidBodyBasics>();

   private final YoFramePoint3D yoRootJointPosition = new YoFramePoint3D("estimatedRootJointPosition", worldFrame, registry);
   private final YoFrameVector3D yoRootJointVelocity = new YoFrameVector3D("estimatedRootJointVelocity", worldFrame, registry);

   private final YoFramePoint3D yoInitialFootPosition = new YoFramePoint3D("initialFootPosition", worldFrame, registry);

   private final YoFramePoint3D yoCenterOfMassPosition = new YoFramePoint3D("estimatedCenterOfMassPosition", worldFrame, registry);
   private final YoFrameVector3D yoCenterOfMassVelocityUsingPelvisAndKinematics = new YoFrameVector3D("estimatedCenterOfMassVelocityPelvisAndKin", worldFrame, registry);
   private final YoFrameVector3D yoCenterOfMassVelocityIntegrateGRF = new YoFrameVector3D("estimatedCenterOfMassVelocityGRF", worldFrame, registry);
   private final YoFrameVector3D yoCenterOfMassVelocity = new YoFrameVector3D("estimatedCenterOfMassVelocity", worldFrame, registry);

   private final CenterOfMassDataHolder estimatorCenterOfMassDataHolderToUpdate;

   private final YoFrameVector3D totalGroundReactionForce = new YoFrameVector3D("totalGroundForce", worldFrame, registry);
   private final YoDouble robotMass = new YoDouble("robotMass", registry);
   private final YoFrameVector3D comAcceleration = new YoFrameVector3D("comAcceleration", worldFrame, registry);

   private final DoubleProvider imuAgainstKinematicsForVelocityBreakFrequency;
   private final DoubleProvider grfAgainstIMUAndKinematicsForVelocityBreakFrequency;
   private final DoubleProvider imuAgainstKinematicsForPositionBreakFrequency;

   private final BooleanProvider useGroundReactionForcesToComputeCenterOfMassVelocity;
   private final YoInteger numberOfEndEffectorsTrusted = new YoInteger("numberOfEndEffectorsTrusted", registry);
   private final YoInteger numberOfEndEffectorsFilteredByLoad = new YoInteger("numberOfEndEffectorsFilteredByLoad", registry);
   private final YoInteger numberOfEndEffectorsFilteredByVelocity = new YoInteger("numberOfEndEffectorsFilteredByVelocity", registry);

   private final Map<RigidBodyBasics, YoDouble> footForcesZInPercentOfTotalForce = new LinkedHashMap<>();
   private final IntegerProvider optimalNumberOfTrustedFeet;
   private final DoubleProvider forceZInPercentThresholdToTrustFoot;
   private final DoubleProvider forceZInPercentThresholdToNotTrustFoot;

   private final Map<RigidBodyBasics, YoDouble> footSpeeds = new LinkedHashMap<>();
   private final YoFrameVector3D weightedAverageFootVelocity = new YoFrameVector3D("weightedAverageFootVelocity", worldFrame, registry);
   private final YoDouble weightedAverageFootSpeed = new YoDouble("weightedAverageFootSpeed", registry);
   private final DoubleProvider footVelocityDifferenceRatioToFilterFoot;
   private final DoubleProvider minFootVelocityToFilterFoot;

   private final Map<RigidBodyBasics, FootSwitchInterface> footSwitches;
   private final Map<RigidBodyBasics, FixedFrameVector3DBasics> footForces = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, Wrench> footWrenches = new LinkedHashMap<>();
   private final DoubleProvider delayTimeBeforeTrustingFoot;
   private final Map<RigidBodyBasics, GlitchFilteredYoBoolean> haveFeetHitGroundFiltered = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, YoBoolean> areFeetTrusted = new LinkedHashMap<>();
   private final Map<RigidBodyBasics, YoBoolean> wereFeetTrustedLastTick = new LinkedHashMap<>();
   private final List<RigidBodyBasics> listOfTrustedFeet = new ArrayList<>();
   private final List<RigidBodyBasics> listOfUnTrustedFeet = new ArrayList<>();

   private final ReferenceFrame rootJointFrame;
   private final Map<RigidBodyBasics, ReferenceFrame> footFrames;

   private final double estimatorDT;

   private final Map<RigidBodyBasics, ? extends ContactablePlaneBody> feetContactablePlaneBodies;

   private final YoBoolean reinitialize = new YoBoolean("reinitialize", registry);
   private final BooleanProvider trustImuWhenNoFeetAreInContact;

   private enum SlippageCompensatorMode
   {
      LOAD_THRESHOLD, MIN_PELVIS_ACCEL, LOAD_THRESHOLD_AND_PELVIS_ACCEL
   };

   private final YoEnum<SlippageCompensatorMode> slippageCompensatorMode = new YoEnum<SlippageCompensatorMode>("slippageCompensatorMode",
         registry, SlippageCompensatorMode.class);

   private final YoBoolean requestStopEstimationOfPelvisLinearState = new YoBoolean("userRequestStopEstimationOfPelvisLinearState", registry);

   private final PelvisKinematicsBasedLinearStateCalculator kinematicsBasedLinearStateCalculator;
   private final PelvisIMUBasedLinearStateCalculator imuBasedLinearStateCalculator;

   private final FloatingJointBasics rootJoint;

   private boolean initializeToActual = false;
   private final FramePoint3D initialRootJointPosition = new FramePoint3D(worldFrame);

   // Temporary variables
   private final FramePoint3D rootJointPosition = new FramePoint3D(worldFrame);
   private final FrameVector3D rootJointVelocity = new FrameVector3D(worldFrame);
   private final FramePoint3D centerOfMassPosition = new FramePoint3D(worldFrame);
   private final FrameVector3D centerOfMassVelocityUsingPelvisIMUAndKinematics = new FrameVector3D(worldFrame);
   private final Vector3D tempRootJointTranslation = new Vector3D();
   private final FramePoint3D footPositionInWorld = new FramePoint3D();
   private final FrameVector3D tempVelocity = new FrameVector3D();

   private final double gravitationalAcceleration;

   private final BooleanProvider trustOnlyLowestFoot = new BooleanParameter("TrustOnlyLowestFoot", registry, false);
   private final IntegerProvider lowestFootWindowSize = new IntegerParameter("LowestFootWindowSize", registry, 0);
   private final GlitchFilteredYoInteger lowestFootInContactIndex = new GlitchFilteredYoInteger("LowestFootInContact", lowestFootWindowSize, registry);

   public PelvisLinearStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure, List<? extends IMUSensorReadOnly> imuProcessedOutputs,
         IMUBiasProvider imuBiasProvider, BooleanProvider cancelGravityFromAccelerationMeasurement, Map<RigidBodyBasics, FootSwitchInterface> footSwitches,
         CenterOfMassDataHolder estimatorCenterOfMassDataHolderToUpdate, CenterOfPressureDataHolder centerOfPressureDataHolderFromController,
         Map<RigidBodyBasics, ? extends ContactablePlaneBody> feetContactablePlaneBodies, double gravitationalAcceleration, StateEstimatorParameters stateEstimatorParameters,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.estimatorDT = stateEstimatorParameters.getEstimatorDT();
      this.footSwitches = footSwitches;
      this.feetContactablePlaneBodies = feetContactablePlaneBodies;
      this.feet.addAll(footSwitches.keySet());

      this.estimatorCenterOfMassDataHolderToUpdate = estimatorCenterOfMassDataHolderToUpdate;

      rootJoint = inverseDynamicsStructure.getRootJoint();
      rootJointFrame = rootJoint.getFrameAfterJoint();

      footFrames = new LinkedHashMap<RigidBodyBasics, ReferenceFrame>();

      RigidBodyBasics elevator = inverseDynamicsStructure.getElevator();
      this.centerOfMassCalculator = new CenterOfMassCalculator(elevator, rootJointFrame);
      this.centerOfMassJacobianWorld = new CenterOfMassJacobian(elevator, rootJointFrame);

      this.gravitationalAcceleration = gravitationalAcceleration;

      robotMass.set(TotalMassCalculator.computeSubTreeMass(elevator));
      setupBunchOfVariables();

      reinitialize.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (reinitialize.getBooleanValue())
               initialize();
         }
      });

      kinematicsBasedLinearStateCalculator = new PelvisKinematicsBasedLinearStateCalculator(inverseDynamicsStructure, feetContactablePlaneBodies, footSwitches,
            centerOfPressureDataHolderFromController, estimatorDT, stateEstimatorParameters, yoGraphicsListRegistry, registry);

      imuBasedLinearStateCalculator = new PelvisIMUBasedLinearStateCalculator(inverseDynamicsStructure, imuProcessedOutputs, imuBiasProvider, cancelGravityFromAccelerationMeasurement, estimatorDT,
            gravitationalAcceleration, stateEstimatorParameters, yoGraphicsListRegistry, registry);

      imuAgainstKinematicsForVelocityBreakFrequency = new DoubleParameter("imuAgainstKinematicsForVelocityBreakFrequency", registry, stateEstimatorParameters.getPelvisLinearVelocityFusingFrequency());
      grfAgainstIMUAndKinematicsForVelocityBreakFrequency = new DoubleParameter("grfAgainstIMUAndKinematicsForVelocityBreakFrequency", registry, stateEstimatorParameters.getCenterOfMassVelocityFusingFrequency());
      imuAgainstKinematicsForPositionBreakFrequency = new DoubleParameter("imuAgainstKinematicsForPositionBreakFrequency", registry, stateEstimatorParameters.getPelvisPositionFusingFrequency());

      delayTimeBeforeTrustingFoot = new DoubleParameter("delayTimeBeforeTrustingFoot", registry, stateEstimatorParameters.getDelayTimeForTrustingFoot());
      optimalNumberOfTrustedFeet = new IntegerParameter("optimalNumberOfTrustedFeet", registry, 2);
      forceZInPercentThresholdToTrustFoot = new DoubleParameter("forceZInPercentThresholdToTrustFoot", registry, stateEstimatorParameters.getForceInPercentOfWeightThresholdToTrustFoot());
      forceZInPercentThresholdToNotTrustFoot = new DoubleParameter("forceZInPercentThresholdToNotTrustFoot", registry, stateEstimatorParameters.getForceInPercentOfWeightThresholdToNotTrustFoot());
      footVelocityDifferenceRatioToFilterFoot = new DoubleParameter("footVelocityDifferenceRatioToFilterFoot", registry, 1.6);
      minFootVelocityToFilterFoot = new DoubleParameter("minFootVelocityToFilterFoot", registry, 0.025);
      trustImuWhenNoFeetAreInContact = new BooleanParameter("trustImuWhenNoFeetAreInContact", registry, stateEstimatorParameters.getPelvisLinearStateUpdaterTrustImuWhenNoFeetAreInContact());
      useGroundReactionForcesToComputeCenterOfMassVelocity = new BooleanParameter("useGRFToComputeCoMVelocity", registry, stateEstimatorParameters.useGroundReactionForcesToComputeCenterOfMassVelocity());

      slippageCompensatorMode.set(SlippageCompensatorMode.LOAD_THRESHOLD_AND_PELVIS_ACCEL);

      //      requestStopEstimationOfPelvisLinearState.set(true);

      if (VISUALIZE)
      {
         if (yoGraphicsListRegistry != null)
         {
            YoArtifactPosition comArtifact = new YoGraphicPosition("Meas CoM", yoCenterOfMassPosition, 0.006, YoAppearance.Black(), GraphicType.CROSS)
                  .createArtifact();
            yoGraphicsListRegistry.registerArtifact("StateEstimator", comArtifact);
         }
      }
      parentRegistry.addChild(registry);
   }

   private void setupBunchOfVariables()
   {
      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         ReferenceFrame footFrame = feetContactablePlaneBodies.get(foot).getSoleFrame();
         footFrames.put(foot, footFrame);

         String footPrefix = foot.getName();

         final GlitchFilteredYoBoolean hasFootHitTheGroundFiltered = new GlitchFilteredYoBoolean("has" + footPrefix + "FootHitGroundFiltered",
               registry, 0);
         hasFootHitTheGroundFiltered.set(true);
         haveFeetHitGroundFiltered.put(foot, hasFootHitTheGroundFiltered);

         YoBoolean isFootTrusted = new YoBoolean("is" + footPrefix + "FootTrusted", registry);
         YoBoolean wasFootTrusted = new YoBoolean("was" + footPrefix + "FootTrustedLastTick", registry);
         if (i == 0)
         {
            isFootTrusted.set(true);
            wasFootTrusted.set(true);
         }
         areFeetTrusted.put(foot, isFootTrusted);
         wereFeetTrustedLastTick.put(foot, wasFootTrusted);

         YoDouble footForceZInPercentOfTotalForce = new YoDouble(footPrefix + "FootForceZInPercentOfTotalForce", registry);
         footForcesZInPercentOfTotalForce.put(foot, footForceZInPercentOfTotalForce);

         YoDouble footSpeed = new YoDouble(footPrefix + "FootSpeed", registry);
         footSpeeds.put(foot, footSpeed);

         footForces.put(foot, new FrameVector3D(worldFrame));
         footWrenches.put(foot, new Wrench());
      }
   }

   public void initialize()
   {
      initializeRobotState();
      updateRootJoint();
   }

   private void initializeRobotState()
   {
      reinitialize.set(false);

      if (!initializeToActual && DRCKinematicsBasedStateEstimator.INITIALIZE_HEIGHT_WITH_FOOT)
      {
         RigidBodyBasics foot = feet.get(0);
         footPositionInWorld.setToZero(footFrames.get(foot));
         footPositionInWorld.changeFrame(worldFrame);
         yoInitialFootPosition.set(footPositionInWorld);

         rootJointPosition.set(rootJoint.getJointPose().getPosition());
         rootJointPosition.setZ(-footPositionInWorld.getZ());
         yoRootJointPosition.set(rootJointPosition);
      }
      else
      {
         rootJointPosition.set(initialRootJointPosition);
      }

      rootJointVelocity.setToZero(worldFrame);
      yoRootJointPosition.set(rootJointPosition);
      yoRootJointVelocity.setToZero();

      kinematicsBasedLinearStateCalculator.initialize(rootJointPosition);
   }

   public void initializeRootJointPosition(Tuple3DReadOnly rootJointPosition)
   {
      initializeToActual = true;
      initialRootJointPosition.setIncludingFrame(worldFrame, rootJointPosition);
   }

   public void updateForFrozenState()
   {
      // Keep setting the position so the localization updater works properly.
      tempRootJointTranslation.set(yoRootJointPosition);
      rootJoint.setJointPosition(tempRootJointTranslation);
      kinematicsBasedLinearStateCalculator.updateKinematics();
      kinematicsBasedLinearStateCalculator.updateFeetPositionsWhenTrustingIMUOnly(yoRootJointPosition);
      kinematicsBasedLinearStateCalculator.setPelvisLinearVelocityToZero();

      // Set the rootJoint twist to zero.
      rootJointTwist.setIncludingFrame(rootJoint.getJointTwist());
      rootJointTwist.setToZero();
      rootJoint.setJointTwist(rootJointTwist);
      rootJoint.updateFramesRecursively();

      updateCoMState();
   }

   public void updateRootJointPositionAndLinearVelocity()
   {
      if (requestStopEstimationOfPelvisLinearState.getBooleanValue())
         return;

      kinematicsBasedLinearStateCalculator.updateKinematics();

      numberOfEndEffectorsTrusted.set(setTrustedFeetUsingFootSwitches());
      numberOfEndEffectorsFilteredByLoad.set(0);
      numberOfEndEffectorsFilteredByVelocity.set(0);

      if (numberOfEndEffectorsTrusted.getIntegerValue() >= optimalNumberOfTrustedFeet.getValue())
      {
         updateFootForcesInPercentOfTotalForce();

         switch (slippageCompensatorMode.getEnumValue())
         {
         case LOAD_THRESHOLD:
            int filteredNumberOfEndEffectorsTrusted = filterTrustedFeetBasedOnContactForces(numberOfEndEffectorsTrusted.getIntegerValue());
            numberOfEndEffectorsTrusted.set(filteredNumberOfEndEffectorsTrusted);
            break;
         case MIN_PELVIS_ACCEL:
            filteredNumberOfEndEffectorsTrusted = filterTrustedFeetBasedOnResultingPelvisAcceleration(numberOfEndEffectorsTrusted.getIntegerValue());
            numberOfEndEffectorsTrusted.set(filteredNumberOfEndEffectorsTrusted);
         case LOAD_THRESHOLD_AND_PELVIS_ACCEL:
            filteredNumberOfEndEffectorsTrusted = filterTrustedFeetBasedOnContactForces(numberOfEndEffectorsTrusted.getIntegerValue());
            filteredNumberOfEndEffectorsTrusted = filterTrustedFeetBasedOnResultingPelvisAcceleration(filteredNumberOfEndEffectorsTrusted);
            numberOfEndEffectorsTrusted.set(filteredNumberOfEndEffectorsTrusted);
            break;
         default:
            throw new RuntimeException("Should not get there");
         }
      }

      if (numberOfEndEffectorsTrusted.getIntegerValue() == 0)
      {
         if (trustImuWhenNoFeetAreInContact.getValue())
         {
            rootJointPosition.set(yoRootJointPosition);
            kinematicsBasedLinearStateCalculator.updateFeetPositionsWhenTrustingIMUOnly(rootJointPosition);

            imuBasedLinearStateCalculator.updateIMUAndRootJointLinearVelocity(rootJointVelocity);
            yoRootJointVelocity.set(rootJointVelocity);
            imuBasedLinearStateCalculator.correctIMULinearVelocity(rootJointVelocity);

            rootJointPosition.set(yoRootJointPosition);
            imuBasedLinearStateCalculator.updatePelvisPosition(rootJointPosition, pelvisPositionIMUPart);
            rootJointPosition.set(pelvisPositionIMUPart);
            yoRootJointPosition.set(rootJointPosition);
         }
         else
            throw new RuntimeException("No foot trusted!");
      }
      else if (numberOfEndEffectorsTrusted.getIntegerValue() > 0)
      {
         updateTrustedFeetLists();
         rootJointPosition.set(yoRootJointPosition);
         kinematicsBasedLinearStateCalculator.estimatePelvisLinearState(listOfTrustedFeet, listOfUnTrustedFeet, rootJointPosition);

         if (imuBasedLinearStateCalculator.isEstimationEnabled())
         {
            computeLinearStateFromMergingMeasurements();
         }
         else
         {
            rootJointPosition.set(yoRootJointPosition);
            kinematicsBasedLinearStateCalculator.getRootJointPositionAndVelocity(rootJointPosition, rootJointVelocity);
            yoRootJointPosition.set(rootJointPosition);
            yoRootJointVelocity.set(rootJointVelocity);
         }
      }
      else
      {
         throw new RuntimeException(
               "Computation of the number of end effectors to be trusted for state estimation is broken, computed: " + numberOfEndEffectorsTrusted);
      }

      updateRootJoint();

      updateCoMState();
   }

   private Twist rootJointTwist = new Twist();

   private void updateRootJoint()
   {
      tempRootJointTranslation.set(yoRootJointPosition);
      rootJoint.setJointPosition(tempRootJointTranslation);

      tempVelocity.setIncludingFrame(rootJointVelocity);
      rootJointTwist.setIncludingFrame(rootJoint.getJointTwist());
      tempVelocity.changeFrame(rootJointTwist.getReferenceFrame());
      rootJointTwist.getLinearPart().set(tempVelocity);
      rootJoint.setJointTwist(rootJointTwist);
      rootJoint.updateFramesRecursively();
   }

   private int setTrustedFeetUsingFootSwitches()
   {
      int numberOfEndEffectorsTrusted = 0;

      int windowSize = (int) (delayTimeBeforeTrustingFoot.getValue() / estimatorDT);

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         wereFeetTrustedLastTick.get(foot).set(areFeetTrusted.get(foot).getValue());
         haveFeetHitGroundFiltered.get(foot).setWindowSize(windowSize);

         if (footSwitches.get(foot).hasFootHitGround())
            haveFeetHitGroundFiltered.get(foot).update(true);
         else
            haveFeetHitGroundFiltered.get(foot).set(false);

         if (haveFeetHitGroundFiltered.get(foot).getBooleanValue())
            numberOfEndEffectorsTrusted++;
      }

      // Update only if at least one foot hit the ground
      if (numberOfEndEffectorsTrusted > 0)
      {
         if (trustOnlyLowestFoot.getValue())
         {
            numberOfEndEffectorsTrusted = filterAndTrustLowestFoot();
         }
         else
         {
            for (int i = 0; i < feet.size(); i++)
            {
               RigidBodyBasics foot = feet.get(i);
               boolean isFootOnGround = haveFeetHitGroundFiltered.get(foot).getBooleanValue();
               areFeetTrusted.get(foot).set(isFootOnGround);
            }
         }
      }
      // Else if there is a foot with a force past the threshold trust the force and not the CoP
      else
      {
         RigidBodyBasics trustedFoot = null;

         for (int i = 0; i < feet.size(); i++)
         {
            RigidBodyBasics foot = feet.get(i);
            if (footSwitches.get(foot).getForceMagnitudePastThreshhold())
            {
               trustedFoot = foot;
               numberOfEndEffectorsTrusted = 1;
               break;
            }
         }

         if (trustedFoot != null)
         {
            for (int i = 0; i < feet.size(); i++)
            {
               RigidBodyBasics foot = feet.get(i);
               areFeetTrusted.get(foot).set(foot == trustedFoot);
            }
         }
      }

      if (numberOfEndEffectorsTrusted == 0)
      {
         if (trustImuWhenNoFeetAreInContact.getValue())
         {
            for (int i = 0; i < feet.size(); i++)
            {
               RigidBodyBasics foot = feet.get(i);
               areFeetTrusted.get(foot).set(false);
            }
         }
         else
         {
            for (int i = 0; i < feet.size(); i++)
            {
               RigidBodyBasics foot = feet.get(i);
               if (areFeetTrusted.get(foot).getBooleanValue())
                  numberOfEndEffectorsTrusted++;
            }
         }
      }

      return numberOfEndEffectorsTrusted;
   }

   private int filterAndTrustLowestFoot()
   {
      int lastLowestFootIdx = lowestFootInContactIndex.getValue();
      int lowestFootIdx = findLowestFootInContact();

      if (haveFeetHitGroundFiltered.get(feet.get(lastLowestFootIdx)).getValue())
      {
         // If the previously trusted foot is still in contact glitch filter the trusted foot to avoid
         // jumping between the feet.
         lowestFootInContactIndex.update(lowestFootIdx);
      }
      else
      {
         // In case the previously trusted foot is not in contact anymore we do not need to glitch filter.
         // Just use the new lowest foot.
         lowestFootInContactIndex.set(lowestFootIdx);
      }

      for (int footIdx = 0; footIdx < feet.size(); footIdx++)
      {
         areFeetTrusted.get(feet.get(footIdx)).set(footIdx == lowestFootInContactIndex.getValue());
      }

      return 1;
   }

   FramePoint3D tmpFramePoint = new FramePoint3D();

   private int findLowestFootInContact()
   {
      int lowestFootInContact = -1;
      double lowestFootZ = Double.MAX_VALUE;
      for (int footIdx = 0; footIdx < feet.size(); footIdx++)
      {
         RigidBodyBasics foot = feet.get(footIdx);
         tmpFramePoint.setToZero(foot.getBodyFixedFrame());
         tmpFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
         double footZ = tmpFramePoint.getZ();

         if (haveFeetHitGroundFiltered.get(foot).getBooleanValue())
         {
            if (footZ < lowestFootZ)
            {
               lowestFootZ = footZ;
               lowestFootInContact = footIdx;
            }
         }
      }
      return lowestFootInContact;
   }

   private void updateFootForcesInPercentOfTotalForce()
   {
      double totalForceZ = 0.0;
      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         if (!areFeetTrusted.get(foot).getBooleanValue())
            continue;
         Wrench footWrench = footWrenches.get(foot);
         footSwitches.get(foot).computeAndPackFootWrench(footWrench);
         FixedFrameVector3DBasics footForce = footForces.get(foot);
         footForce.setMatchingFrame(footWrench.getLinearPart());
         totalForceZ += footForce.getZ();
      }

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         YoDouble footLoad = footForcesZInPercentOfTotalForce.get(foot);
         if (!areFeetTrusted.get(foot).getBooleanValue())
         {
            footLoad.setToNaN();
            continue;
         }

         FixedFrameVector3DBasics footForce = footForces.get(foot);
         footLoad.set(footForce.getZ() / totalForceZ);
      }
   }

   private int filterTrustedFeetBasedOnContactForces(int numberOfEndEffectorsTrusted)
   {
      int filteredNumberOfEndEffectorsTrusted = 0;

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         if (!areFeetTrusted.get(foot).getBooleanValue())
            continue;

         double percentForceToTrustFootAgain = forceZInPercentThresholdToTrustFoot.getValue();
         double percentForceToNotTrustFoot = forceZInPercentThresholdToNotTrustFoot.getValue();
         percentForceToTrustFootAgain = MathTools.clamp(percentForceToTrustFootAgain, minForceZInPercentThresholdToFilterFoot, maxForceZInPercentThresholdToFilterFoot);
         percentForceToNotTrustFoot = MathTools.clamp(percentForceToNotTrustFoot, minForceZInPercentThresholdToFilterFoot, maxForceZInPercentThresholdToFilterFoot);

         double magnitudeForTrust;
         if (wereFeetTrustedLastTick.get(foot).getValue())
            magnitudeForTrust = percentForceToNotTrustFoot;
         else
            magnitudeForTrust = percentForceToTrustFootAgain;

         if (footForcesZInPercentOfTotalForce.get(foot).getValue() < magnitudeForTrust)
            areFeetTrusted.get(foot).set(false);
         else
            filteredNumberOfEndEffectorsTrusted++;
      }

      numberOfEndEffectorsFilteredByLoad.set(numberOfEndEffectorsTrusted - filteredNumberOfEndEffectorsTrusted);

      return filteredNumberOfEndEffectorsTrusted;
   }

   private int filterTrustedFeetBasedOnResultingPelvisAcceleration(int numberOfEndEffectorsTrusted)
   {
      weightedAverageFootVelocity.setToZero();

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         // don't add it if it isn't already trusted
         if (!areFeetTrusted.get(foot).getBooleanValue())
         {
            footSpeeds.get(foot).setToNaN();
            continue;
         }

         // any foot velocity is bad, as it should all be zero, unless it is slipping, but we can trust the velocity more if the foot is loaded more.
         FrameVector3DReadOnly footVelocity = kinematicsBasedLinearStateCalculator.getFootVelocityInWorld(foot);
         footSpeeds.get(foot).set(footVelocity.length());
         weightedAverageFootVelocity.scaleAdd(footForcesZInPercentOfTotalForce.get(foot).getValue(), footVelocity, weightedAverageFootVelocity);
      }

      weightedAverageFootSpeed.set(weightedAverageFootVelocity.length());

      int filteredNumberOfEndEffectorsTrusted = numberOfEndEffectorsTrusted;

      sortFeetBySpeed();

      int footIndex = feet.size() - 1;
      while (footIndex >= 0 && filteredNumberOfEndEffectorsTrusted > 1)
      {
         RigidBodyBasics foot = feet.get(footIndex);
         // don't bother if it isn't already trusted
         if (!areFeetTrusted.get(foot).getBooleanValue())
         {
            footSpeeds.get(foot).setToNaN();
            continue;
         }

         if (footSpeeds.get(foot).getValue() > Math.max(minFootVelocityToFilterFoot.getValue(), weightedAverageFootSpeed.getDoubleValue() * footVelocityDifferenceRatioToFilterFoot.getValue()))
         {
            areFeetTrusted.get(foot).set(false);
            filteredNumberOfEndEffectorsTrusted--;
         }
         else
         { // these are in order, so once one isn't filtered, the rest will be ok
            break;
         }

         footIndex--;
      }


      numberOfEndEffectorsFilteredByVelocity.set(numberOfEndEffectorsTrusted - filteredNumberOfEndEffectorsTrusted);
      return filteredNumberOfEndEffectorsTrusted;
   }


   private final FrameVector3D pelvisVelocityIMUPart = new FrameVector3D();
   private final FramePoint3D pelvisPositionIMUPart = new FramePoint3D();

   private final FrameVector3D pelvisVelocityKinPart = new FrameVector3D();
   private final FramePoint3D pelvisPositionKinPart = new FramePoint3D();

   private void computeLinearStateFromMergingMeasurements()
   {
      computeLinearVelocityFromMergingMeasurements();
      computePositionFromMergingMeasurements();
   }

   private void computeLinearVelocityFromMergingMeasurements()
   {
      // TODO Check out AlphaFusedYoVariable to that
      imuBasedLinearStateCalculator.updateIMUAndRootJointLinearVelocity(pelvisVelocityIMUPart);
      kinematicsBasedLinearStateCalculator.getPelvisVelocity(pelvisVelocityKinPart);

      double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(imuAgainstKinematicsForVelocityBreakFrequency.getValue(), estimatorDT);
      pelvisVelocityIMUPart.scale(alpha);
      pelvisVelocityKinPart.scale(1.0 - alpha);

      rootJointVelocity.add(pelvisVelocityIMUPart, pelvisVelocityKinPart);
      yoRootJointVelocity.set(rootJointVelocity);

      imuBasedLinearStateCalculator.correctIMULinearVelocity(rootJointVelocity);
   }

   private void computePositionFromMergingMeasurements()
   {
      rootJointPosition.set(yoRootJointPosition);
      imuBasedLinearStateCalculator.updatePelvisPosition(rootJointPosition, pelvisPositionIMUPart);
      kinematicsBasedLinearStateCalculator.getPelvisPosition(pelvisPositionKinPart);

      double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(imuAgainstKinematicsForPositionBreakFrequency.getValue(), estimatorDT);
      pelvisPositionIMUPart.scale(alpha);
      pelvisPositionKinPart.scale(1.0 - alpha);

      rootJointPosition.set(pelvisPositionIMUPart);
      rootJointPosition.add(pelvisPositionKinPart);
      yoRootJointPosition.set(rootJointPosition);
   }

   private final FrameVector3D comVelocityGRFPart = new FrameVector3D();
   private final FrameVector3D comVelocityPelvisAndKinPart = new FrameVector3D();

   private void updateCoMState()
   {
      centerOfMassCalculator.reset();
      centerOfMassPosition.setIncludingFrame(centerOfMassCalculator.getCenterOfMass());
      centerOfMassPosition.changeFrame(worldFrame);
      yoCenterOfMassPosition.set(centerOfMassPosition);

      centerOfMassJacobianWorld.reset();
      centerOfMassVelocityUsingPelvisIMUAndKinematics.setToZero(ReferenceFrame.getWorldFrame());
      centerOfMassVelocityUsingPelvisIMUAndKinematics.setIncludingFrame(centerOfMassJacobianWorld.getCenterOfMassVelocity());
      centerOfMassVelocityUsingPelvisIMUAndKinematics.changeFrame(ReferenceFrame.getWorldFrame());
      yoCenterOfMassVelocityUsingPelvisAndKinematics.set(centerOfMassVelocityUsingPelvisIMUAndKinematics);

      if (useGroundReactionForcesToComputeCenterOfMassVelocity.getValue())
      {
         //centerOfMassVelocity at this point is from pelvis velocity and joint velocities, where pelvis velocity is estimated from pelvis imu at high freq and leg kinematics at low freq.

         //TODO: Lots of duplicated computation with reading the force sensors and changing frames. Just do it once and share...
         computeTotalGroundReactionForce();

         double totalMass = robotMass.getDoubleValue();
         if (totalMass < 0.01) totalMass = 0.01;

         comAcceleration.scale(1.0/totalMass);

         comAcceleration.scale(estimatorDT);
         yoCenterOfMassVelocityIntegrateGRF.add(comAcceleration);

         comVelocityGRFPart.set(yoCenterOfMassVelocity);
         comVelocityGRFPart.add(comAcceleration);
         comAcceleration.scale(1.0/estimatorDT);

         comVelocityPelvisAndKinPart.set(centerOfMassVelocityUsingPelvisIMUAndKinematics);

         double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(grfAgainstIMUAndKinematicsForVelocityBreakFrequency.getValue(), estimatorDT);
         comVelocityGRFPart.scale(alpha);
         comVelocityPelvisAndKinPart.scale(1.0 - alpha);


         yoCenterOfMassVelocity.add(comVelocityGRFPart, comVelocityPelvisAndKinPart);
      }
      else
      {
         yoCenterOfMassVelocity.set(centerOfMassVelocityUsingPelvisIMUAndKinematics);
      }

      if (estimatorCenterOfMassDataHolderToUpdate != null) estimatorCenterOfMassDataHolderToUpdate.setCenterOfMassVelocity(yoCenterOfMassVelocity);
   }


   private final FrameVector3D tempCoMAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D tempFootForce = new FrameVector3D(ReferenceFrame.getWorldFrame());

   private void computeTotalGroundReactionForce()
   {
      totalGroundReactionForce.setToZero();

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         Wrench footWrench = footWrenches.get(foot);
         footSwitches.get(foot).computeAndPackFootWrench(footWrench);
         tempFootForce.setIncludingFrame(footWrench.getLinearPart());
         tempFootForce.changeFrame(worldFrame);

         totalGroundReactionForce.add(tempFootForce);
      }

      tempCoMAcceleration.set(totalGroundReactionForce);
      comAcceleration.set(tempCoMAcceleration);
      comAcceleration.setZ(comAcceleration.getZ() - robotMass.getDoubleValue() * gravitationalAcceleration);
   }

   private void updateTrustedFeetLists()
   {
      listOfTrustedFeet.clear();
      listOfUnTrustedFeet.clear();

      for (int i = 0; i < feet.size(); i++)
      {
         RigidBodyBasics foot = feet.get(i);
         if (areFeetTrusted.get(foot).getBooleanValue())
            listOfTrustedFeet.add(foot);
         else
            listOfUnTrustedFeet.add(foot);
      }
   }

   private void sortFeetBySpeed()
   {
      boolean ordered = false;

      while (!ordered)
      {
         ordered = true;
         for (int i = 0; i < feet.size() - 1; i++)
         {
            RigidBodyBasics footA = feet.get(i);
            RigidBodyBasics footB = feet.get(i + 1);

            if (footSpeedComparator.compare(footSpeeds.get(footA), footSpeeds.get(footB)) > 0)
            {
               ordered = false;
               swap(feet, i, i + 1);
            }
         }
      }
   }

   private static <T> void swap(List<T> ts, int a, int b)
   {
      T tmp = ts.get(a);
      ts.set(a, ts.get(b));
      ts.set(b, tmp);
   }

   public void getEstimatedPelvisPosition(FramePoint3D pelvisPositionToPack)
   {
      pelvisPositionToPack.setIncludingFrame(yoRootJointPosition);
   }

   public void getEstimatedPelvisLinearVelocity(FrameVector3D pelvisLinearVelocityToPack)
   {
      pelvisLinearVelocityToPack.setIncludingFrame(yoRootJointVelocity);
   }

   public void getEstimatedCoMPosition(FramePoint3D comPositionToPack)
   {
      comPositionToPack.setIncludingFrame(yoCenterOfMassPosition);
   }

   public void getEstimatedCoMVelocity(FrameVector3D comVelocityToPack)
   {
      comVelocityToPack.setIncludingFrame(yoCenterOfMassVelocity);
   }

   public List<RigidBodyBasics> getCurrentListOfTrustedFeet()
   {
      return listOfTrustedFeet;
   }
}
