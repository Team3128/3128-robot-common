package org.team3128.narwhalvision;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

import org.team3128.common.util.Log;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.io.ByteBufferInput;

public class NarwhalVisionReceiver
{
	final static String TAG = "NarwhalVisionReceiver";

	private DatagramSocket visionDataSocket;
	private Thread internalThread;
	
	private Kryo kryo;
	private ByteBufferInput packetReader;
	
	private long lastPacketRecvTime = 0;
	
	private TargetInformation mostRecentTarget;
	
	public NarwhalVisionReceiver()
	{
		try
		{
			visionDataSocket = new DatagramSocket(3128);
		}
		catch(SocketException e)
		{
			e.printStackTrace();
		}
		
		internalThread = new Thread(this::receiveLoop);
		internalThread.start();
		
		kryo = new Kryo();
		kryo.register(TargetInformation.class);
		packetReader = new ByteBufferInput();
	}
	
	// static buffer used to hold packets/serialized objects
	// no, as far as I can tell, there's no way to not have a fixed size buffer
	final static int SERIALIZATION_BUFFER_SIZE=1024;
	
	private void receiveLoop()
	{
		byte[] recvBuffer = new byte[SERIALIZATION_BUFFER_SIZE];
		
		DatagramPacket visionPacket = new DatagramPacket(recvBuffer, SERIALIZATION_BUFFER_SIZE);
		
		while(true)
		{
			
			try
			{
				visionDataSocket.receive(visionPacket);
			}
			catch(IOException e)
			{
				Log.recoverable(TAG, "Failed to receive vision packet: " + e.getMessage());
				e.printStackTrace();
				continue;
			}
			
			packetReader.setBuffer(recvBuffer);
			
			try
			{
				setMostRecentTarget(kryo.readObject(packetReader, TargetInformation.class));
				setLastPacketReceivedTime(System.currentTimeMillis());
			}
			catch(ClassCastException ex)
			{
				Log.recoverable(TAG, "Received the wrong class from the phone: " + ex.getMessage());
				ex.printStackTrace();
			}
			catch(RuntimeException ex)
			{
				Log.recoverable(TAG, "Error deserializing: " + ex.getMessage());
				ex.getMessage();
			}
			
		}
	}
	
	//these functions are synchronized because they return data set by the thread
	
	/**
	 * Gets the UNIX timestamp of when the last packet was received.  If the value is 0, no packet has ever been received.
	 * @return
	 */
	public synchronized long getLastPacketReceivedTime()
	{
		return lastPacketRecvTime;
	}
	
	private synchronized void setLastPacketReceivedTime(long time)
	{
		lastPacketRecvTime = time;
	}
	
	/**
	 * Get the most recent target information sent by the phone
	 * @return
	 */
	public synchronized TargetInformation getMostRecentTarget()
	{
		return mostRecentTarget;
	}
	
	private synchronized void setMostRecentTarget(TargetInformation target)
	{
		mostRecentTarget = target;
	}
}
