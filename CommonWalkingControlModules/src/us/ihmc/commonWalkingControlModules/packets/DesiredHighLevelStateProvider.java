package us.ihmc.commonWalkingControlModules.packets;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.HighLevelState;
import us.ihmc.communication.packets.HighLevelStatePacket;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredHighLevelStateProvider implements ObjectConsumer<HighLevelStatePacket>
{
   private final AtomicReference<HighLevelState> highLevelState = new AtomicReference<HighLevelState>(null);
   
   public DesiredHighLevelStateProvider()
   {
   }
   
   public boolean checkForNewState()
   {
      return highLevelState.get() != null;
   }
   
   public HighLevelState getDesiredHighLevelState()
   {
      return highLevelState.getAndSet(null);
   }
   
   public void consumeObject(HighLevelStatePacket object)
   {
       highLevelState.set(object.getHighLevelState());
   }

}
