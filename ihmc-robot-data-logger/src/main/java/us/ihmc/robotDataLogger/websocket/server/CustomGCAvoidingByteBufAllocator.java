package us.ihmc.robotDataLogger.websocket.server;

import java.nio.ByteBuffer;
import java.util.ArrayList;

import io.netty.buffer.AbstractByteBufAllocator;
import io.netty.buffer.ByteBuf;
import io.netty.buffer.ByteBufAllocator;
import io.netty.buffer.Unpooled;

public class CustomGCAvoidingByteBufAllocator extends AbstractByteBufAllocator
{
   private final static int INITIAL_MAX_CAPACITY = 128;
   private final static int INITIAL_POOL_SIZE = 16;

   private final ArrayList<ByteBuf> pool = new ArrayList<ByteBuf>(INITIAL_POOL_SIZE);

   private int capacity = INITIAL_MAX_CAPACITY;

   private int index = 0;

   private final ByteBufAllocator backupAllocator;

   public CustomGCAvoidingByteBufAllocator(ByteBufAllocator backupAllocator)
   {
      super(true);
      refill(INITIAL_MAX_CAPACITY);
      this.backupAllocator = backupAllocator;
   }

   private ByteBuf add(int capacity)
   {
      ByteBuf wrappedBuffer = Unpooled.wrappedBuffer(ByteBuffer.allocateDirect(capacity));
      wrappedBuffer.internalNioBuffer(0, 0); // Force allocation of temporary object
      pool.add(wrappedBuffer);
      return wrappedBuffer;
   }

   private void refill(int capacity)
   {
      pool.clear();
      for (int i = 0; i < INITIAL_POOL_SIZE; i++)
      {
         add(capacity);
      }
      this.capacity = capacity;
   }

   @Override
   public boolean isDirectBufferPooled()
   {
      return false;
   }

   @Override
   protected ByteBuf newHeapBuffer(int initialCapacity, int maxCapacity)
   {
      return backupAllocator.heapBuffer(initialCapacity, maxCapacity);
   }

   @Override
   protected ByteBuf newDirectBuffer(int initialCapacity, int maxCapacity)
   {
      if (initialCapacity > capacity)
      {
         return backupAllocator.directBuffer(initialCapacity, maxCapacity);
      }

      for (int i = index; i < index + pool.size(); i++)
      {
         int item = i < pool.size() ? i : i - pool.size();

         ByteBuf byteBuf = pool.get(item);
         if (byteBuf.refCnt() == 1)
         {
            index = item + 1;
            byteBuf.clear();
            byteBuf.internalNioBuffer(0, 0); // Force allocation of temporary object
            return byteBuf.retain();
         }
      }

      return add(capacity);
   }

}
