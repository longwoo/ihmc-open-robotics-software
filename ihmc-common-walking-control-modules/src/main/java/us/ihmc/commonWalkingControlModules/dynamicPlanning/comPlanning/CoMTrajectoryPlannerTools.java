package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;

public class CoMTrajectoryPlannerTools
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double sufficientlyLarge = 1.0e10;

   /**
    * e<sup>&omega; t</sup>
    */
   static double getCoMPositionFirstCoefficient(double omega, double time)
   {
      return Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * e<sup>-&omega; t</sup>
    */
   static double getCoMPositionSecondCoefficient(double omega, double time)
   {
      return Math.exp(-omega * time);
   }

   /**
    * t<sup>3</sup>
    */
   static double getCoMPositionThirdCoefficient(double time)
   {
      return Math.min(sufficientlyLarge, time * time * time);
   }

   /**
    * t<sup>2</sup>
    */
   static double getCoMPositionFourthCoefficient(double time)
   {
      return Math.min(sufficientlyLarge, time * time);
   }

   /**
    * t
    */
   static double getCoMPositionFifthCoefficient(double time)
   {
      return Math.min(sufficientlyLarge, time);
   }

   /**
    * 1.0
    */
   static double getCoMPositionSixthCoefficient()
   {
      return 1.0;
   }

   /**
    * &omega; e<sup>&omega; t</sup>
    */
   static double getCoMVelocityFirstCoefficient(double omega, double time)
   {
      return omega * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * -&omega; e<sup>-&omega; t</sup>
    */
   static double getCoMVelocitySecondCoefficient(double omega, double time)
   {
      return -omega * Math.exp(-omega * time);
   }

   /**
    * 3 t<sup>2</sup>
    */
   static double getCoMVelocityThirdCoefficient(double time)
   {
      return 3.0 * Math.min(sufficientlyLarge, time * time);
   }

   /**
    * 2 t
    */
   static double getCoMVelocityFourthCoefficient(double time)
   {
      return 2.0 * Math.min(sufficientlyLarge, time);
   }

   /**
    * 1.0
    */
   static double getCoMVelocityFifthCoefficient()
   {
      return 1.0;
   }

   /**
    * 0.0
    */
   static double getCoMVelocitySixthCoefficient()
   {
      return 0.0;
   }

   /**
    * &omega;<sup>2</sup> e<sup>&omega; t</sup>
    */
   static double getCoMAccelerationFirstCoefficient(double omega, double time)
   {
      return omega * omega * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * &omega;<sup>2</sup> e<sup>-&omega; t</sup>
    */
   static double getCoMAccelerationSecondCoefficient(double omega, double time)
   {
      return omega * omega * Math.exp(-omega * time);
   }

   /**
    * 6 t
    */
   static double getCoMAccelerationThirdCoefficient(double time)
   {
      return 6.0 * Math.min(sufficientlyLarge, time);
   }

   /**
    * 2
    */
   static double getCoMAccelerationFourthCoefficient()
   {
      return 2.0;
   }

   /**
    * 0.0
    */
   static double getCoMAccelerationFifthCoefficient()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   static double getCoMAccelerationSixthCoefficient()
   {
      return 0.0;
   }

   /**
    * &omega;<sup>3</sup> e<sup>&omega; t</sup>
    */
   static double getCoMJerkFirstCoefficient(double omega, double time)
   {
      return omega * omega * omega * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * -&omega;<sup>3</sup> e<sup>-&omega; t</sup>
    */
   static double getCoMJerkSecondCoefficient(double omega, double time)
   {
      return -omega * omega * omega * Math.exp(-omega * time);
   }

   /**
    * 6.0
    */
   static double getCoMJerkThirdCoefficient()
   {
      return 6.0;
   }

   /**
    * 0.0
    */
   static double getCoMJerkFourthCoefficient()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   static double getCoMJerkFifthCoefficient()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   static double getCoMJerkSixthCoefficient()
   {
      return 0.0;
   }

   /**
    * 2 e<sup>&omega; t</sup>
    */
   static double getDCMPositionFirstCoefficient(double omega, double time)
   {
      return 2.0 * Math.min(sufficientlyLarge, Math.exp(omega * time));
   }

   /**
    * 0.0
    */
   static double getDCMPositionSecondCoefficient()
   {
      return 0.0;
   }

   /**
    * t<sup>3</sup> + 3.0 / &omega; t<sup>2</sup>
    */
   static double getDCMPositionThirdCoefficient(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time * time * time) + 3.0 / omega * Math.min(sufficientlyLarge, time * time);
   }

   /**
    * t<sup>2</sup> + 2.0 / &omega; t
    */
   static double getDCMPositionFourthCoefficient(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time * time) + 2.0 / omega * Math.min(sufficientlyLarge, time);
   }

   /**
    * t + 1/ &omega;
    */
   static double getDCMPositionFifthCoefficient(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time) + 1.0 / omega;
   }

   /**
    * 1.0
    */
   static double getDCMPositionSixthCoefficient()
   {
      return 1.0;
   }

   /**
    * 0.0
    */
   static double getVRPPositionFirstCoefficient()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   static double getVRPPositionSecondCoefficient()
   {
      return 0.0;
   }

   /**
    * t<sup>3</sup> - 6.0 t / &omega;<sup>2</sup>
    */
   static double getVRPPositionThirdCoefficient(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time * time * time) - 6.0 * Math.min(sufficientlyLarge, time) / (omega * omega);
   }

   /**
    * t<sup>2</sup> - 2.0 / &omega;<sup>2</sup>
    */
   static double getVRPPositionFourthCoefficient(double omega, double time)
   {
      return Math.min(sufficientlyLarge, time * time) - 2.0 / (omega * omega);
   }

   /**
    * t
    */
   static double getVRPPositionFifthCoefficient(double time)
   {
      return Math.min(sufficientlyLarge, time);
   }

   /**
    * 1.0
    */
   static double getVRPPositionSixthCoefficient()
   {
      return 1.0;
   }

   /**
    * 0.0
    */
   static double getVRPVelocityFirstCoefficient()
   {
      return 0.0;
   }

   /**
    * 0.0
    */
   static double getVRPVelocitySecondCoefficient()
   {
      return 0.0;
   }

   /**
    * 3 t<sup>2</sup> - 6 / &omega;<sup>2</sup>
    */
   static double getVRPVelocityThirdCoefficient(double omega, double time)
   {
      return 3.0 * Math.min(sufficientlyLarge, time * time) - 6.0 / (omega * omega);
   }

   /**
    * 2 t
    */
   static double getVRPVelocityFourthCoefficient(double time)
   {
      return 2.0 * Math.min(sufficientlyLarge, time);
   }

   /**
    * 1.0
    */
   static double getVRPVelocityFifthCoefficient()
   {
      return 1.0;
   }

   /**
    * 0.0
    */
   static double getVRPVelocitySixthCoefficient()
   {
      return 0.0;
   }

   static void constructDesiredCoMPosition(FixedFramePoint3DBasics comPositionToPack, FramePoint3DReadOnly firstCoefficient,
                                           FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                           FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient, FramePoint3DReadOnly sixthCoefficient,
                                           double timeInPhase, double omega)
   {
      comPositionToPack.checkReferenceFrameMatch(worldFrame);
      comPositionToPack.setToZero();
      comPositionToPack.scaleAdd(getCoMPositionFirstCoefficient(omega, timeInPhase), firstCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionSecondCoefficient(omega, timeInPhase), secondCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionThirdCoefficient(timeInPhase), thirdCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionFourthCoefficient(timeInPhase), fourthCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionFifthCoefficient(timeInPhase), fifthCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(getCoMPositionSixthCoefficient(), sixthCoefficient, comPositionToPack);
   }

   static void constructDesiredCoMVelocity(FixedFrameVector3DBasics comVelocityToPack, FramePoint3DReadOnly firstCoefficient,
                                           FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                           FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient, FramePoint3DReadOnly sixthCoefficient,
                                           double timeInPhase, double omega)
   {
      comVelocityToPack.checkReferenceFrameMatch(worldFrame);
      comVelocityToPack.setToZero();
      comVelocityToPack.scaleAdd(getCoMVelocityFirstCoefficient(omega, timeInPhase), firstCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocitySecondCoefficient(omega, timeInPhase), secondCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocityThirdCoefficient(timeInPhase), thirdCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocityFourthCoefficient(timeInPhase), fourthCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocityFifthCoefficient(), fifthCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(getCoMVelocitySixthCoefficient(), sixthCoefficient, comVelocityToPack);
   }

   static void constructDesiredCoMAcceleration(FixedFrameVector3DBasics comAccelerationToPack, FramePoint3DReadOnly firstCoefficient,
                                               FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                               FramePoint3DReadOnly fourthCoefficient, FramePoint3DReadOnly fifthCoefficient,
                                               FramePoint3DReadOnly sixthCoefficient, double timeInPhase, double omega)
   {
      comAccelerationToPack.checkReferenceFrameMatch(worldFrame);
      comAccelerationToPack.setToZero();
      comAccelerationToPack.scaleAdd(getCoMAccelerationFirstCoefficient(omega, timeInPhase), firstCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationSecondCoefficient(omega, timeInPhase), secondCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationThirdCoefficient(timeInPhase), thirdCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationFourthCoefficient(), fourthCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationFifthCoefficient(), fifthCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(getCoMAccelerationSixthCoefficient(), sixthCoefficient, comAccelerationToPack);
   }
}
