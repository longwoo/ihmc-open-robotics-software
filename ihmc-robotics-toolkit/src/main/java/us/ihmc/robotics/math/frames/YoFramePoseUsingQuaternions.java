package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFramePoseUsingQuaternions implements FixedFramePose3DBasics
{
   private final YoFramePoint position;
   private final YoFrameQuaternion orientation;

   public YoFramePoseUsingQuaternions(YoFramePoint position, YoFrameQuaternion orientation)
   {
      position.checkReferenceFrameMatch(orientation);
      this.position = position;
      this.orientation = orientation;
   }

   public YoFramePoseUsingQuaternions(String prefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      this(prefix, "", frame, registry);
   }

   public YoFramePoseUsingQuaternions(String prefix, String suffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      position = new YoFramePoint(prefix, suffix, frame, registry);
      orientation = new YoFrameQuaternion(prefix, suffix, frame, registry);
   }

   public YoFramePoint getPosition()
   {
      return position;
   }

   public YoFrameQuaternion getOrientation()
   {
      return orientation;
   }

   public void setAndMatchFrame(FramePose3D framePose)
   {
      position.setAndMatchFrame(framePose.getPosition());
      orientation.setAndMatchFrame(framePose.getOrientation());
   }

   public void setPosition(YoFramePoint yoFramePoint)
   {
      position.set(yoFramePoint);
   }

   public void setPosition(FramePoint3D framePoint)
   {
      position.set(framePoint);
   }

   public void setPosition(Tuple3DReadOnly position)
   {
      this.position.set(position);
   }

   public void setOrientation(YoFrameQuaternion yoFrameQuaternion)
   {
      orientation.set(yoFrameQuaternion);
   }
   
   public void setOrientation(FrameQuaternionReadOnly frameOrientation)
   {
      orientation.set(frameOrientation);
   }

   public void setOrientation(QuaternionReadOnly quaternion)
   {
      orientation.set(quaternion);
   }

   public void set(FramePoint3DReadOnly framePoint, FrameQuaternionReadOnly frameOrientation)
   {
      position.set(framePoint);
      orientation.set(frameOrientation);
   }

   public void set(YoFramePose yoFramePose)
   {
      set(yoFramePose.getPosition(), yoFramePose.getOrientation().getFrameOrientation());
   }

   public void setAndMatchFrame(FramePoint3DReadOnly framePoint, FrameQuaternionReadOnly frameOrientation)
   {
      position.setAndMatchFrame(framePoint);
      orientation.setAndMatchFrame(frameOrientation);
   }

   public void setPosition(double x, double y, double z)
   {
      position.set(x, y, z);
   }

   public void setXYZ(double x, double y, double z)
   {
      position.set(x, y, z);
   }

   public void setXYZ(double[] pos)
   {
      setXYZ(pos[0], pos[1], pos[2]);
   }

   @Override
   public void setToNaN()
   {
      position.setToNaN();
      orientation.setToNaN();
   }

   @Override
   public void setToZero()
   {
      position.setToZero();
      orientation.setToZero();
   }

   @Override
   public boolean containsNaN()
   {
      return position.containsNaN() || orientation.containsNaN();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return position.getReferenceFrame();
   }

   public void attachVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      position.attachVariableChangedListener(variableChangedListener);
      orientation.attachVariableChangedListener(variableChangedListener);
   }

   public double getDistance(YoFramePose goalYoPose)
   {
      return position.distance(goalYoPose.getPosition());
   }

   public void setX(double x)
   {
      position.setX(x);
   }

   public void setY(double y)
   {
      position.setY(y);
   }

   public void setZ(double z)
   {
      position.setZ(z);
   }

   public double getX()
   {
      return getPosition().getX();
   }

   public double getY()
   {
      return getPosition().getY();
   }

   public double getZ()
   {
      return getPosition().getZ();
   }

   public void getOrientation(Quaternion quaternionToPack)
   {
      quaternionToPack.set(getOrientation());
   }

   public YoDouble getYoX()
   {
      return getPosition().getYoX();
   }

   public YoDouble getYoY()
   {
      return getPosition().getYoY();
   }

   public YoDouble getYoZ()
   {
      return getPosition().getYoZ();
   }

   public YoDouble getYoQs()
   {
      return getOrientation().getYoQs();
   }

   public YoDouble getYoQx()
   {
      return getOrientation().getYoQx();
   }

   public YoDouble getYoQy()
   {
      return getOrientation().getYoQy();
   }

   public YoDouble getYoQz()
   {
      return getOrientation().getYoQz();
   }
}
