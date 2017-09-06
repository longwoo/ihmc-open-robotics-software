package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.CoPPointsInFoot;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.WalkingTrajectoryType;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.TrajectoryMathTools;
import us.ihmc.robotics.math.trajectories.YoFrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.YoSegmentedFrameTrajectory3D;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Estimates the angular momentum generated by the swing foot about the CoM during a footstep
 * Needs a footstep CoP plan. Uses the entry, exit and end CoPs defined in the CoP plan to calculate a segmented CoM trajectory
 * The CoM trajectory is then used along with the footstep plan to determine the angular momentum generated
 */
public class FootstepAngularMomentumPredictor implements AngularMomentumTrajectoryGeneratorInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final FrameVector3D zeroVector = new FrameVector3D();
   private static final int maxNumberOfTrajectoryCoefficients = 7;
   private static final int numberOfSwingSegments = 3;
   private static final int numberOfTransferSegments = 2;

   private final boolean DEBUG;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final int maxNumberOfFootstepsToConsider = 4;
   private final TrajectoryMathTools trajectoryMathTools;

   private final YoBoolean computePredictedAngularMomentum;
   private final YoInteger numberOfFootstepsToConsider;
   private CoPPointName entryCoPName;

   private final YoDouble gravityZ;
   private final YoDouble swingLegMass;
   private final YoDouble supportLegMass;
   private final YoDouble comHeight;
   private final YoDouble swingFootMaxHeight;

   private final List<CoPPointsInFoot> upcomingCoPsInFootsteps;
   private final YoInteger numberOfRegisteredFootsteps;

   private final List<SwingAngularMomentumTrajectory> swingAngularMomentumTrajectories;
   private final List<TransferAngularMomentumTrajectory> transferAngularMomentumTrajectories;

   private final FrameVector3D desiredAngularMomentum = new FrameVector3D();
   private final FrameVector3D desiredTorque = new FrameVector3D();
   private final FrameVector3D desiredRotatum = new FrameVector3D();

   private final FrameTrajectory3D segmentCoMTrajectory;
   private final FrameTrajectory3D segmentCoMVelocity;
   private final FrameTrajectory3D phaseSwingFootTrajectory;
   private final FrameTrajectory3D segmentSwingFootTrajectory;
   private final FrameTrajectory3D swingFootVelocity;
   private final FrameTrajectory3D phaseSupportFootTrajectory;
   private final FrameTrajectory3D segmentSupportFootTrajectory;
   private final FrameTrajectory3D supportFootVelocity;
   private final FrameTrajectory3D estimatedAngularMomentumTrajectory;
   private AngularMomentumTrajectoryInterface activeTrajectory;
   private double initialTime;

   private List<FramePoint3D> comInitialPositions;
   private List<FramePoint3D> comFinalPositions;
   private List<FrameVector3D> comInitialVelocities;
   private List<FrameVector3D> comFinalVelocities;
   private List<FrameVector3D> comInitialAccelerations;
   private List<FrameVector3D> comFinalAccelerations;

   private final FramePoint3D tempFramePoint1 = new FramePoint3D();
   private final FramePoint3D tempFramePoint2 = new FramePoint3D();
   private final FrameVector3D tempFrameVector = new FrameVector3D();

   // DEBUGGING
   private final YoFramePoint comPosDebug;
   private final YoFramePoint comVelDebug;
   private final YoFramePoint comAccDebug;
   private final YoFramePoint swingFootPosDebug;
   private final YoFramePoint swingFootVelDebug;
   private final YoFramePoint swingFootAccDebug;
   private final YoFramePoint supportFootPosDebug;
   private final YoFramePoint supportFootVelDebug;
   private final YoFramePoint supportFootAccDebug;
   private final List<TrajectoryDebug> transferCoMTrajectories;
   private final List<TrajectoryDebug> swingCoMTrajectories;
   private final List<TrajectoryDebug> transferSwingFootTrajectories;
   private final List<TrajectoryDebug> swingSwingFootTrajectories;
   private final List<TrajectoryDebug> transferSupportFootTrajectories;
   private final List<TrajectoryDebug> swingSupportFootTrajectories;
   private TrajectoryDebug activeCoMTrajectory;
   private TrajectoryDebug activeSwingFootTrajectory;
   private TrajectoryDebug activeSupportFootTrajectory;


   public FootstepAngularMomentumPredictor(String namePrefix, YoDouble omega0, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, omega0, false, parentRegistry);
   }

   public FootstepAngularMomentumPredictor(String namePrefix, YoDouble omega0, boolean debug, YoVariableRegistry parentRegistry)
   {
      this.DEBUG = debug;
      String fullPrefix = namePrefix + "AngularMomentum";
      this.trajectoryMathTools = new TrajectoryMathTools(2 * maxNumberOfTrajectoryCoefficients);
      this.computePredictedAngularMomentum = new YoBoolean(fullPrefix + "ComputePredictedAngularMomentum", registry);
      this.numberOfFootstepsToConsider = new YoInteger(fullPrefix + "MaxFootsteps", registry);
      this.gravityZ = new YoDouble("AngularMomentumGravityZ", parentRegistry);
      omega0.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            comHeight.set(gravityZ.getDoubleValue() / (omega0.getDoubleValue() * omega0.getDoubleValue()));
         }
      });
      gravityZ.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            comHeight.set(gravityZ.getDoubleValue() / (omega0.getDoubleValue() * omega0.getDoubleValue()));
         }
      });
      this.swingLegMass = new YoDouble(fullPrefix + "SwingFootMass", registry);
      this.supportLegMass = new YoDouble(fullPrefix + "SupportFootMass", registry);
      this.comHeight = new YoDouble(fullPrefix + "CoMHeight", registry);

      this.swingFootMaxHeight = new YoDouble(fullPrefix + "SwingFootMaxHeight", registry);
      this.swingAngularMomentumTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider);
      this.transferAngularMomentumTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider + 1);

      if (DEBUG)
      {
         this.swingCoMTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider);
         this.transferCoMTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider + 1);
         this.swingSwingFootTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider);
         this.transferSwingFootTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider + 1);
         this.swingSupportFootTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider);
         this.transferSupportFootTrajectories = new ArrayList<>(maxNumberOfFootstepsToConsider + 1);
      }
      else
      {
         this.swingCoMTrajectories = null;
         this.transferCoMTrajectories = null;
         this.swingSwingFootTrajectories = null;
         this.transferSwingFootTrajectories = null;
         this.swingSupportFootTrajectories = null;
         this.transferSupportFootTrajectories = null;
      }

      this.upcomingCoPsInFootsteps = new ArrayList<>(maxNumberOfFootstepsToConsider + 2);
      this.numberOfRegisteredFootsteps = new YoInteger(fullPrefix + "NumberOfRegisteredFootsteps", registry);
      ReferenceFrame[] referenceFrames = {worldFrame};
      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         SwingAngularMomentumTrajectory swingTrajectory = new SwingAngularMomentumTrajectory(i, worldFrame,
                                                                                             numberOfSwingSegments, 2 * maxNumberOfTrajectoryCoefficients);
         this.swingAngularMomentumTrajectories.add(swingTrajectory);

         TransferAngularMomentumTrajectory transferTrajectory = new TransferAngularMomentumTrajectory(i, worldFrame,
                                                                                                      numberOfTransferSegments,
                                                                                                      2 * maxNumberOfTrajectoryCoefficients);
         this.transferAngularMomentumTrajectories.add(transferTrajectory);
         CoPPointsInFoot copLocations = new CoPPointsInFoot(i, referenceFrames, registry);
         upcomingCoPsInFootsteps.add(copLocations);

         if (DEBUG)
         {
            TrajectoryDebug swingCoMTrajectory = new TrajectoryDebug("SwingCoMTrajDebug" + i, numberOfSwingSegments, maxNumberOfTrajectoryCoefficients,
                                                                     registry);
            swingCoMTrajectories.add(swingCoMTrajectory);
            TrajectoryDebug transferCoMTrajectory = new TrajectoryDebug("TransferCoMTrajDebug" + i, numberOfTransferSegments, maxNumberOfTrajectoryCoefficients,
                                                                        registry);
            transferCoMTrajectories.add(transferCoMTrajectory);
            TrajectoryDebug swingSwingFootTrajectory = new TrajectoryDebug("SwingSwFootTrajDebug" + i, numberOfSwingSegments, maxNumberOfTrajectoryCoefficients,
                                                                           registry);
            swingSwingFootTrajectories.add(swingSwingFootTrajectory);
            TrajectoryDebug transferSwingFootTrajectory = new TrajectoryDebug("TransferSwFootTrajDebug" + i, numberOfTransferSegments,
                                                                              maxNumberOfTrajectoryCoefficients, registry);
            transferSwingFootTrajectories.add(transferSwingFootTrajectory);
            TrajectoryDebug swingSupportFootTrajectory = new TrajectoryDebug("SwingSpFootTrajDebug" + i, numberOfSwingSegments,
                                                                             maxNumberOfTrajectoryCoefficients, registry);
            swingSupportFootTrajectories.add(swingSupportFootTrajectory);
            TrajectoryDebug transferSupportFootTrajectory = new TrajectoryDebug("TransferSpFootTrajDebug" + i, numberOfTransferSegments,
                                                                                maxNumberOfTrajectoryCoefficients, registry);
            transferSupportFootTrajectories.add(transferSupportFootTrajectory);
         }
      }
      CoPPointsInFoot copLocations = new CoPPointsInFoot(maxNumberOfFootstepsToConsider, referenceFrames, registry);
      upcomingCoPsInFootsteps.add(copLocations);
      copLocations = new CoPPointsInFoot(maxNumberOfFootstepsToConsider + 1, referenceFrames, registry);
      upcomingCoPsInFootsteps.add(copLocations);
      TransferAngularMomentumTrajectory transferTrajectory = new TransferAngularMomentumTrajectory(maxNumberOfFootstepsToConsider,
                                                                                                   worldFrame, numberOfTransferSegments,
                                                                                                   2 * maxNumberOfTrajectoryCoefficients);
      this.transferAngularMomentumTrajectories.add(transferTrajectory);
      if (DEBUG)
      {
         TrajectoryDebug transferCoMTrajectory = new TrajectoryDebug("TransferCoMTrajDebug" + maxNumberOfFootstepsToConsider, numberOfTransferSegments,
                                                                     maxNumberOfTrajectoryCoefficients, registry);
         transferCoMTrajectories.add(transferCoMTrajectory);
         TrajectoryDebug transferSwingFootTrajectory = new TrajectoryDebug("TransferSwFootTrajDebug" + maxNumberOfFootstepsToConsider, numberOfTransferSegments,
                                                                           maxNumberOfTrajectoryCoefficients, registry);
         transferSwingFootTrajectories.add(transferSwingFootTrajectory);
         TrajectoryDebug transferSupportFootTrajectory = new TrajectoryDebug("TransferSpFootTrajDebug" + maxNumberOfFootstepsToConsider,
                                                                             numberOfTransferSegments, maxNumberOfTrajectoryCoefficients, registry);
         transferSupportFootTrajectories.add(transferSupportFootTrajectory);
      }

      this.segmentCoMTrajectory = new FrameTrajectory3D(maxNumberOfTrajectoryCoefficients, worldFrame);
      this.segmentCoMVelocity = new FrameTrajectory3D(maxNumberOfTrajectoryCoefficients, worldFrame);
      this.phaseSwingFootTrajectory = new FrameTrajectory3D(2 * maxNumberOfTrajectoryCoefficients, worldFrame);
      this.segmentSwingFootTrajectory = new FrameTrajectory3D(2 * maxNumberOfTrajectoryCoefficients, worldFrame);
      this.swingFootVelocity = new FrameTrajectory3D(maxNumberOfTrajectoryCoefficients, worldFrame);
      this.phaseSupportFootTrajectory = new FrameTrajectory3D(2 * maxNumberOfTrajectoryCoefficients, worldFrame);
      this.segmentSupportFootTrajectory = new FrameTrajectory3D(2 * maxNumberOfTrajectoryCoefficients, worldFrame);
      this.supportFootVelocity = new FrameTrajectory3D(maxNumberOfTrajectoryCoefficients, worldFrame);
      this.estimatedAngularMomentumTrajectory = new FrameTrajectory3D(2 * maxNumberOfTrajectoryCoefficients, worldFrame);
      if (DEBUG)
      {
         this.comPosDebug = new YoFramePoint("CoMPosViz", "", worldFrame, registry);
         this.comVelDebug = new YoFramePoint("CoMVelViz", "", worldFrame, registry);
         this.comAccDebug = new YoFramePoint("CoMAccViz", "", worldFrame, registry);
         this.swingFootPosDebug = new YoFramePoint("SwFPosViz", worldFrame, registry);
         this.swingFootVelDebug = new YoFramePoint("SwFVelViz", worldFrame, registry);
         this.swingFootAccDebug = new YoFramePoint("SwFAccViz", worldFrame, registry);
         this.supportFootPosDebug = new YoFramePoint("SpFPosViz", worldFrame, registry);
         this.supportFootVelDebug = new YoFramePoint("SpFVelViz", worldFrame, registry);
         this.supportFootAccDebug = new YoFramePoint("SpFAccViz", worldFrame, registry);
      }
      else
      {
         comPosDebug = null;
         comVelDebug = null;
         comAccDebug = null;
         swingFootPosDebug = null;
         swingFootVelDebug = null;
         swingFootAccDebug = null;
         supportFootPosDebug = null;
         supportFootVelDebug = null;
         supportFootAccDebug = null;
      }

      parentRegistry.addChild(registry);
   }

   public void initializeParameters(AngularMomentumEstimationParameters angularMomentumParameters)
   {
      this.computePredictedAngularMomentum.set(angularMomentumParameters.computePredictedAngularMomentum());
      this.numberOfFootstepsToConsider.set(angularMomentumParameters.getNumberOfFootstepsToConsider());
      this.entryCoPName = angularMomentumParameters.getEntryCoPName();
      this.swingLegMass.set(angularMomentumParameters.getSwingLegMass());
      this.supportLegMass.set(angularMomentumParameters.getSupportLegMass());
      this.swingFootMaxHeight.set(angularMomentumParameters.getSwingFootMaxLift());
      this.gravityZ.set(angularMomentumParameters.getGravityZ());
   }

   @Override
   public void updateListeners()
   {
      // TODO Auto-generated method stub      
   }

   @Override
   public void createVisualizerForConstantAngularMomentum(YoGraphicsList yoGraphicsList, ArtifactList artifactList)
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void clear()
   {
      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         swingAngularMomentumTrajectories.get(i).reset();
         transferAngularMomentumTrajectories.get(i).reset();
      }
      transferAngularMomentumTrajectories.get(maxNumberOfFootstepsToConsider).reset();
      for (int i = 0; i < upcomingCoPsInFootsteps.size(); i++)
         upcomingCoPsInFootsteps.get(i).reset();

      if (DEBUG)
      {
         transferCoMTrajectories.get(maxNumberOfFootstepsToConsider).reset();
         transferSwingFootTrajectories.get(maxNumberOfFootstepsToConsider).reset();
         transferSupportFootTrajectories.get(maxNumberOfFootstepsToConsider).reset();

         for (int i = 0 ; i < maxNumberOfFootstepsToConsider; i++)
         {
            swingCoMTrajectories.get(i).reset();
            transferCoMTrajectories.get(i).reset();
            swingSwingFootTrajectories.get(i).reset();
            transferSwingFootTrajectories.get(i).reset();
            swingSupportFootTrajectories.get(i).reset();
            transferSupportFootTrajectories.get(i).reset();
         }
      }
   }

   public void addFootstepCoPsToPlan(List<CoPPointsInFoot> copLocations, List<FramePoint3D> comInitialPositions, List<FramePoint3D> comFinalPositions,
                                     List<FrameVector3D> comInitialVelocities, List<FrameVector3D> comFinalVelocities,
                                     List<FrameVector3D> comInitialAccelerations, List<FrameVector3D> comFinalAccelerations, int numberOfRegisteredFootsteps)
   {
      for (int i = 0; i < copLocations.size(); i++)
         upcomingCoPsInFootsteps.get(i).setIncludingFrame(copLocations.get(i));
      this.numberOfRegisteredFootsteps.set(numberOfRegisteredFootsteps);
      this.comInitialPositions = comInitialPositions;
      this.comFinalPositions = comFinalPositions;
      this.comInitialVelocities = comInitialVelocities;
      this.comFinalVelocities = comFinalVelocities;
      this.comInitialAccelerations = comInitialAccelerations;
      this.comFinalAccelerations = comFinalAccelerations;
   }

   @Override
   public void update(double currentTime)
   {
      if (activeTrajectory != null && computePredictedAngularMomentum.getBooleanValue())
         activeTrajectory.update(currentTime - initialTime, desiredAngularMomentum, desiredTorque, desiredRotatum);
      else
      {
         desiredAngularMomentum.setToZero();
         desiredTorque.setToZero();
         desiredRotatum.setToZero();
      }
      getPredictedCenterOfMassPosition(currentTime);
      getPredictedSwingFootPosition(currentTime);
   }

   @Override
   public void getDesiredAngularMomentum(FrameVector3D desiredAngMomToPack)
   {
      desiredAngMomToPack.setIncludingFrame(desiredAngularMomentum);
   }

   @Override
   public void getDesiredAngularMomentum(FrameVector3D desiredAngMomToPack, FrameVector3D desiredTorqueToPack)
   {
      desiredAngMomToPack.setIncludingFrame(desiredAngularMomentum);
      desiredTorqueToPack.setIncludingFrame(desiredTorque);
   }

   public void getDesiredAngularMomentum(FrameVector3D desiredAngMomToPack, FrameVector3D desiredTorqueToPack, FrameVector3D desiredRotatumToPack)
   {
      desiredAngMomToPack.setIncludingFrame(desiredAngularMomentum);
      desiredTorqueToPack.setIncludingFrame(desiredTorque);
      desiredRotatumToPack.setIncludingFrame(desiredRotatum);
   }

   @Override
   public void getDesiredAngularMomentum(YoFrameVector desiredAngMomToPack)
   {
      desiredAngMomToPack.set(desiredAngularMomentum);
   }

   @Override
   public void getDesiredAngularMomentum(YoFrameVector desiredAngMomToPack, YoFrameVector desiredTorqueToPack)
   {
      desiredAngMomToPack.set(desiredAngularMomentum);
      desiredTorqueToPack.set(desiredTorque);
   }

   public void getDesiredAngularMomentum(YoFrameVector desiredAngMomToPack, YoFrameVector desiredTorqueToPack, YoFrameVector desiredRotatumToPack)
   {
      desiredAngMomToPack.set(desiredAngularMomentum);
      desiredTorqueToPack.set(desiredTorque);
      desiredRotatumToPack.set(desiredRotatum);
   }

   @Override
   public void initializeForTransfer(double currentTime)
   {
      initialTime = currentTime;
      activeTrajectory = transferAngularMomentumTrajectories.get(0);
      if (DEBUG)
      {
         activeCoMTrajectory = transferCoMTrajectories.get(0);
         activeSwingFootTrajectory = transferSwingFootTrajectories.get(0);
         activeSupportFootTrajectory = transferSupportFootTrajectories.get(0);
      }
   }

   @Override
   public void initializeForSwing(double currentTime)
   {
      initialTime = currentTime;
      activeTrajectory = swingAngularMomentumTrajectories.get(0);
      if (DEBUG)
      {
         activeCoMTrajectory = swingCoMTrajectories.get(0);
         activeSwingFootTrajectory = swingSwingFootTrajectories.get(0);
         activeSupportFootTrajectory = swingSupportFootTrajectories.get(0);
      }
   }

   @Override
   public void computeReferenceAngularMomentumStartingFromDoubleSupport(boolean atAStop)
   {
      int footstepIndex = 0;
      setAngularMomentumTrajectoryForFootsteps(footstepIndex, WalkingTrajectoryType.TRANSFER);
   }

   @Override
   public void computeReferenceAngularMomentumStartingFromSingleSupport()
   {
      int footstepIndex = 0;
      setAngularMomentumTrajectoryForFootsteps(footstepIndex, WalkingTrajectoryType.SWING);
   }

   private void setAngularMomentumTrajectoryForFootsteps(int footstepIndex, WalkingTrajectoryType initialWalkingPhase)
   {
      CoPPointsInFoot copPointsInFoot = upcomingCoPsInFootsteps.get(footstepIndex);
      double phaseTime = 0.0;
      int comIndex = 0;
      if(initialWalkingPhase == WalkingTrajectoryType.SWING)
      {
         copPointsInFoot = upcomingCoPsInFootsteps.get(footstepIndex + 1);
         setFootTrajectoriesForPhase(footstepIndex, initialWalkingPhase);
         for(int j = CoPPlanningTools.getCoPPointIndex(copPointsInFoot.getCoPPointList(), entryCoPName) + 1; j < copPointsInFoot.getNumberOfCoPPoints(); j++, comIndex++)
         {
            setCoMTrajectory(phaseTime, phaseTime + copPointsInFoot.get(j).getTime(), comIndex);
            setFootTrajectoriesForSegment(phaseTime, phaseTime + copPointsInFoot.get(j).getTime());
            if(DEBUG)
            {
               swingCoMTrajectories.get(footstepIndex).set(segmentCoMTrajectory);
               swingSwingFootTrajectories.get(footstepIndex).set(segmentSwingFootTrajectory);
               swingSupportFootTrajectories.get(footstepIndex).set(segmentSupportFootTrajectory);
            }
            calculateAngularMomentumTrajectory();
            swingAngularMomentumTrajectories.get(footstepIndex).set(estimatedAngularMomentumTrajectory);
            phaseTime += copPointsInFoot.get(j).getTime();
         }
         footstepIndex++;
         phaseTime = 0.0;
      }
      WalkingTrajectoryType currentWalkingPhase = WalkingTrajectoryType.TRANSFER;
      int numberOfSteps = Math.min(numberOfRegisteredFootsteps.getIntegerValue(), numberOfFootstepsToConsider.getIntegerValue());
      for(int i = footstepIndex; i < numberOfSteps; i++)
      {
         copPointsInFoot = upcomingCoPsInFootsteps.get(i + 1);
         setFootTrajectoriesForPhase(i, currentWalkingPhase);
         for(int j = 0; j < copPointsInFoot.getNumberOfCoPPoints(); j++, comIndex++)
         {
            setCoMTrajectory(phaseTime, phaseTime + copPointsInFoot.get(j).getTime(), comIndex);
            setFootTrajectoriesForSegment(phaseTime, phaseTime + copPointsInFoot.get(j).getTime());
            if(DEBUG)
            {
               if(currentWalkingPhase == WalkingTrajectoryType.TRANSFER)
               {
                  transferCoMTrajectories.get(i).set(segmentCoMTrajectory);
                  transferSwingFootTrajectories.get(i).set(segmentSwingFootTrajectory);
                  transferSupportFootTrajectories.get(i).set(segmentSupportFootTrajectory);
               }
               else if(currentWalkingPhase == WalkingTrajectoryType.SWING)
               {
                  swingCoMTrajectories.get(i).set(segmentCoMTrajectory);
                  swingSwingFootTrajectories.get(i).set(segmentSwingFootTrajectory);
                  swingSupportFootTrajectories.get(i).set(segmentSupportFootTrajectory);
               }
            }
            calculateAngularMomentumTrajectory();
            if(currentWalkingPhase == WalkingTrajectoryType.TRANSFER)
               transferAngularMomentumTrajectories.get(i).set(estimatedAngularMomentumTrajectory);
            else if(currentWalkingPhase == WalkingTrajectoryType.SWING)
               swingAngularMomentumTrajectories.get(i).set(estimatedAngularMomentumTrajectory);
            phaseTime += copPointsInFoot.get(j).getTime();
            if(copPointsInFoot.getCoPPointList().get(j) == entryCoPName)
            {
               currentWalkingPhase = WalkingTrajectoryType.SWING;
               phaseTime = 0.0;
            }
            else if(j >= copPointsInFoot.getNumberOfCoPPoints() - 1)
            {
               currentWalkingPhase = WalkingTrajectoryType.TRANSFER;
               phaseTime = 0.0;
            }
         }
      }
      copPointsInFoot = upcomingCoPsInFootsteps.get(numberOfSteps + 1);
      setFootTrajectoriesForPhase(numberOfSteps, currentWalkingPhase);
      for(int j = 0; j < copPointsInFoot.getNumberOfCoPPoints(); j++, comIndex++)
      {
         setCoMTrajectory(phaseTime, phaseTime + copPointsInFoot.get(j).getTime(), comIndex);
         setFootTrajectoriesForSegment(phaseTime, phaseTime + copPointsInFoot.get(j).getTime());
         if(DEBUG)
         {
               transferCoMTrajectories.get(numberOfSteps).set(segmentCoMTrajectory);
               transferSwingFootTrajectories.get(numberOfSteps).set(segmentSwingFootTrajectory);
               transferSupportFootTrajectories.get(numberOfSteps).set(segmentSupportFootTrajectory);
         }
         calculateAngularMomentumTrajectory();
         transferAngularMomentumTrajectories.get(numberOfSteps).set(estimatedAngularMomentumTrajectory);
         phaseTime += copPointsInFoot.get(j).getTime();
      }
   }

   private void setCoMTrajectory(double initialTime, double finalTime, int comIndex)
   {
      tempFramePoint1.set(comInitialPositions.get(comIndex));
      tempFramePoint1.addZ(comHeight.getDoubleValue());
      tempFramePoint2.set(comFinalPositions.get(comIndex));
      tempFramePoint2.addZ(comHeight.getDoubleValue());
      segmentCoMTrajectory.setQuintic(initialTime, finalTime, tempFramePoint1, comInitialVelocities.get(comIndex),
                                      comInitialAccelerations.get(comIndex), tempFramePoint2, comFinalVelocities.get(comIndex),
                                      comFinalAccelerations.get(comIndex));
      trajectoryMathTools.getDerivative(segmentCoMVelocity, segmentCoMTrajectory);
   }

   private void setFootTrajectoriesForSegment(double initialTime, double finalTime)
   {
      segmentSwingFootTrajectory.set(phaseSwingFootTrajectory);
      segmentSwingFootTrajectory.setTime(initialTime, finalTime);
      trajectoryMathTools.getDerivative(swingFootVelocity, segmentSwingFootTrajectory);
      segmentSupportFootTrajectory.set(phaseSupportFootTrajectory);
      segmentSupportFootTrajectory.setTime(initialTime, finalTime);
      trajectoryMathTools.getDerivative(supportFootVelocity, segmentSupportFootTrajectory);
   }
   
   private void setFootTrajectoriesForPhase(int footstepIndex, WalkingTrajectoryType phase)
   {
      CoPPointsInFoot copPointsInFoot = upcomingCoPsInFootsteps.get(footstepIndex + 1);
      double phaseDuration = 0.0;
      if(phase == WalkingTrajectoryType.SWING)
      {
         for(int j = CoPPlanningTools.getCoPPointIndex(copPointsInFoot.getCoPPointList(), entryCoPName) + 1; j < copPointsInFoot.getNumberOfCoPPoints(); j++)
            phaseDuration += copPointsInFoot.get(j).getTime();
      }
      else
      {
         int j = 0;
         for(; j < copPointsInFoot.getNumberOfCoPPoints() - 1 && copPointsInFoot.getCoPPointList().get(j) != entryCoPName && copPointsInFoot.getCoPPointList().get(j) != CoPPointName.FINAL_COP; j++)
            phaseDuration += copPointsInFoot.get(j).getTime();
         phaseDuration += copPointsInFoot.get(j).getTime();
      }
      setSwingFootTrajectoryForPhase(footstepIndex, phase, phaseDuration);
      setSupportFootTrajectoryForPhase(footstepIndex, phase, phaseDuration);
   }
   
   private void setSwingFootTrajectoryForPhase(int footstepIndex, WalkingTrajectoryType phase, double phaseDuration)
   {
      if(phase == WalkingTrajectoryType.SWING)
      {
         upcomingCoPsInFootsteps.get(footstepIndex).getSupportFootLocation(tempFramePoint1);
         upcomingCoPsInFootsteps.get(footstepIndex + 1).getSwingFootLocation(tempFramePoint2);
         phaseSwingFootTrajectory.setQuinticWithZeroTerminalAcceleration(0.0, phaseDuration, tempFramePoint1, zeroVector, tempFramePoint2, zeroVector);
      }
      else
      {
         upcomingCoPsInFootsteps.get(footstepIndex).getSupportFootLocation(tempFramePoint1);
         phaseSwingFootTrajectory.setQuinticWithZeroTerminalAcceleration(0.0, phaseDuration, tempFramePoint1, zeroVector, tempFramePoint1, zeroVector);
      }
   }
   
   private void setSupportFootTrajectoryForPhase(int footstepIndex, WalkingTrajectoryType phase, double phaseDuration)
   {
      upcomingCoPsInFootsteps.get(footstepIndex + 1).getSupportFootLocation(tempFramePoint1);
      phaseSupportFootTrajectory.setQuinticWithZeroTerminalAcceleration(0.0, phaseDuration, tempFramePoint1, zeroVector, tempFramePoint1, zeroVector);
   }

   private void calculateAngularMomentumTrajectory()
   {
      trajectoryMathTools.subtract(segmentSwingFootTrajectory, segmentSwingFootTrajectory, segmentCoMTrajectory);
      trajectoryMathTools.subtract(swingFootVelocity, swingFootVelocity, segmentCoMVelocity);
      trajectoryMathTools.subtract(segmentSupportFootTrajectory, segmentSupportFootTrajectory, segmentCoMTrajectory);
      trajectoryMathTools.subtract(supportFootVelocity, supportFootVelocity, segmentCoMVelocity);

      trajectoryMathTools.crossProduct(segmentSwingFootTrajectory, segmentSwingFootTrajectory, swingFootVelocity);
      trajectoryMathTools.scale(segmentSwingFootTrajectory, segmentSwingFootTrajectory, swingLegMass.getDoubleValue());
      trajectoryMathTools.crossProduct(segmentSupportFootTrajectory, segmentSupportFootTrajectory, supportFootVelocity);
      trajectoryMathTools.scale(segmentSupportFootTrajectory, segmentSupportFootTrajectory, supportLegMass.getDoubleValue());

      trajectoryMathTools.add(estimatedAngularMomentumTrajectory, segmentSupportFootTrajectory, segmentSwingFootTrajectory);
   }

   public void getPredictedCenterOfMassPosition(YoFramePoint pointToPack, double time)
   {
      if (DEBUG && computePredictedAngularMomentum.getBooleanValue())
      {
         activeCoMTrajectory.update(time - initialTime);
         activeCoMTrajectory.getFramePosition(tempFramePoint1);
         pointToPack.set(tempFramePoint1);
         comPosDebug.set(tempFramePoint1);
         activeCoMTrajectory.getFrameVelocity(tempFrameVector);
         comVelDebug.set(tempFrameVector);
         activeCoMTrajectory.getFrameAcceleration(tempFrameVector);
         comAccDebug.set(tempFrameVector);
      }
   }

   public void getPredictedCenterOfMassPosition(double time)
   {
      if (DEBUG && computePredictedAngularMomentum.getBooleanValue())
      {
         activeCoMTrajectory.update(time - initialTime);
         activeCoMTrajectory.getFramePosition(tempFramePoint1);
         comPosDebug.set(tempFramePoint1);
         activeCoMTrajectory.getFrameVelocity(tempFrameVector);
         comVelDebug.set(tempFrameVector);
         activeCoMTrajectory.getFrameAcceleration(tempFrameVector);
         comAccDebug.set(tempFrameVector);
      }
   }

   public void getPredictedFootPosition(YoFramePoint pointToPack, double time)
   {
      if (DEBUG && computePredictedAngularMomentum.getBooleanValue())
      {
         activeSwingFootTrajectory.update(time - initialTime);
         activeSwingFootTrajectory.getFramePosition(tempFramePoint1);
         swingFootPosDebug.set(tempFramePoint1);
         pointToPack.set(tempFramePoint1);
         activeSwingFootTrajectory.getFrameVelocity(tempFrameVector);
         swingFootVelDebug.set(tempFrameVector);
         activeSwingFootTrajectory.getFrameAcceleration(tempFrameVector);
         swingFootAccDebug.set(tempFrameVector);

         activeSupportFootTrajectory.update(time - initialTime);
         activeSupportFootTrajectory.getFramePosition(tempFramePoint1);
         supportFootPosDebug.set(tempFramePoint1);
         activeSupportFootTrajectory.getFrameVelocity(tempFrameVector);
         supportFootVelDebug.set(tempFrameVector);
         activeSupportFootTrajectory.getFrameAcceleration(tempFrameVector);
         supportFootAccDebug.set(tempFrameVector);
      }
   }

   public void getPredictedSwingFootPosition(double time)
   {
      if (DEBUG && computePredictedAngularMomentum.getBooleanValue())
      {
         activeSwingFootTrajectory.update(time - initialTime);
         activeCoMTrajectory.getFramePosition(tempFramePoint1);
         swingFootPosDebug.set(tempFramePoint1);
         activeCoMTrajectory.getFrameVelocity(tempFrameVector);
         swingFootVelDebug.set(tempFrameVector);
         activeCoMTrajectory.getFrameAcceleration(tempFrameVector);
         swingFootAccDebug.set(tempFrameVector);

         activeSupportFootTrajectory.update(time - initialTime);
         activeCoMTrajectory.getFramePosition(tempFramePoint1);
         supportFootPosDebug.set(tempFramePoint1);
         activeCoMTrajectory.getFrameVelocity(tempFrameVector);
         supportFootVelDebug.set(tempFrameVector);
         activeCoMTrajectory.getFrameAcceleration(tempFrameVector);
         supportFootAccDebug.set(tempFrameVector);
      }
   }

   @Override
   public List<? extends AngularMomentumTrajectory> getTransferAngularMomentumTrajectories()
   {
      if (computePredictedAngularMomentum.getBooleanValue())
         return transferAngularMomentumTrajectories; //null
      else
         return null;
   }

   @Override
   public List<? extends AngularMomentumTrajectory> getSwingAngularMomentumTrajectories()
   {
      if (computePredictedAngularMomentum.getBooleanValue())
         return swingAngularMomentumTrajectories; //null
      else
         return null;
   }

   private class TrajectoryDebug extends YoSegmentedFrameTrajectory3D
   {

      public TrajectoryDebug(String name, int maxNumberOfSegments, int maxNumberOfCoefficients, YoVariableRegistry registry)
      {
         super(name, maxNumberOfSegments, maxNumberOfCoefficients, registry);
      }

      public void set(YoFrameTrajectory3D trajToCopy)
      {
         segments.get(getNumberOfSegments()).set(trajToCopy);
         numberOfSegments.increment();
      }

      public void set(FrameTrajectory3D trajToCopy)
      {
         segments.get(getNumberOfSegments()).set(trajToCopy);
         numberOfSegments.increment();
      }
   }
}
