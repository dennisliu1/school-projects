
package Comm.UDP.Conn;

import Comm.ParamFunction;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.ArrayList;

/**
 * Thread to keep receiving packets from the socket.
 * 
 * @version 1.0
 * @author Dennis Liu, YURT 2012
 */
public class UDPReceive {
	/** Running status for the thread */
	public static final String STATUS_STARTED = "started";
	/** Paused status for the thread */
	public static final String STATUS_PAUSED = "paused";
	/** Stopped status for the thread */
	public static final String STATUS_STOPPED = "stopped";
	//-----------------------		Variables			--------------------------//
	/** Socket to receive packets from */
	private DatagramSocket inSocket;
	/** Packet placeholder to hold received packets */
	private DatagramPacket receivePacket;
	/** Buffer for received packets */
	private ArrayList<byte[]> receiveQueue;
	public ArrayList<InetAddress> receivePacketQueue;
	public ArrayList<Integer> receivePortQueue;
	/** Maximum size for each packet */
	private int packetSize;
	/** The receiving packet thread */
	private ReceivePacketThread RPThread;
	/** Status of the thread */
	private String threadStatus;
	/** Functions running at each iteration of the thread */
	private ArrayList<ParamFunction> threadFuncts;
	//-----------------------		Constructor		--------------------------//
	/**
	 * Constructs a receive thread to get packets coming to the UDP Socket.
	 * @param inSocket socket to receive packets from
	 * @param receiveQueue queue to add packets to
	 * @param packetSize maximum size of packets; bigger packets received will be truncated
	 */
	public UDPReceive(DatagramSocket inSocket, ArrayList<byte[]> receiveQueue, ArrayList<InetAddress> receivePacketQueue, ArrayList<Integer> receivePortQueue, int packetSize) {
		this.inSocket = inSocket;
		this.receiveQueue = receiveQueue;
		this.packetSize = packetSize;
		this.receivePacketQueue = receivePacketQueue; 
		this.receivePortQueue = receivePortQueue;
		receivePacket = new DatagramPacket(new byte[packetSize], packetSize);
		RPThread = null;
		threadStatus = STATUS_STOPPED;//status is started as stopped
		threadFuncts = new ArrayList<ParamFunction>();
	}
	public boolean setVars(DatagramSocket inSocket, ArrayList<byte[]> receiveQueue, int packetSize) {
		if(threadStatus.equals(STATUS_STOPPED)) {
			this.inSocket = inSocket;
			this.receiveQueue = receiveQueue;
			this.packetSize = packetSize;
			return true;
		}
		return false;
	}
	//-----------------------		Accessors			------	--------------------//
	/**
	 * Returns the maximum packet size that can be received
	 */
	public int getMaxPacketSize() {
		return packetSize;
	}
	/**
	 * Sets the maximum packet size that can be received. Works only when thread
	 * is not running.
	 * @param packetSize new maximum packet size
	 * @return if change worked
	 */
	public boolean setMaxPacketSize(int packetSize) {
		if(threadStatus.equals(STATUS_STOPPED)) {
			this.packetSize = packetSize;
			receivePacket.setData(new byte[packetSize]);
			receivePacket.setLength(packetSize);
			return true;
		}
		return false;
	}
	//-----------------------		Functs			------	--------------------//
	/** Add a function to run in the connection thread. Does not work while thread is running */
	public boolean addReceiveThreadFunct(ParamFunction threadFunct) {
		if(threadStatus.equals(STATUS_STOPPED)) {
			if(threadFunct != null) {
				threadFuncts.add(threadFunct);
				return true;
			}
		}
		return false;
	}
	/** Remove a function running in the thread. Does not work while thread is running. */
	public boolean removeReceiveThreadFunct(ParamFunction threadFunct) {
		if(threadStatus.equals(STATUS_STOPPED))
			if(threadFunct != null)
				return threadFuncts.remove(threadFunct);
		return false;
	}
	/** Get function index running in the thread. */
	public ParamFunction getReceiveThreadFunct(int index) {
		if(!threadFuncts.isEmpty()) {
			return threadFuncts.get(index);
		}
		return null;
	}
	//-----------------------		Threading			------	--------------------//
	/**
	 * Start receiving packets. If thread is already running, unpauses thread.
	 * Constructs the thread if it is dead.
	 */
	public boolean startReceive() {
		if(RPThread == null) {
			RPThread = new ReceivePacketThread();
			RPThread.start();
		}
		if(RPThread.pause) {//if thread just started
			RPThread.pause = false;
			threadStatus = STATUS_STARTED;
			return true;
		}
		return false;
	}
	/**
	 * Pause receiving packets. Means packets are not accepted by thread,
	 * but thread is still running.
	 */
	public boolean pauseReceive() {
		if(RPThread != null) {
			RPThread.pause = true;
			threadStatus = STATUS_PAUSED;
			return true;
		}
		return false;
	}
	/**
	 * Stop receiving packets. Kills thread.
	 */
	public boolean stopReceive() {
		if(RPThread != null) {
			RPThread.pause = false;
			RPThread.isRunning = false;
			RPThread = null;
			threadStatus = STATUS_STOPPED;
			return true;
		}
		return false;
	}
	/**
	 * Thread that receives packets coming to the socket.
	 */
	private class ReceivePacketThread extends Thread {
		private boolean isRunning = true;
		private boolean pause = true;
		@Override
		public void run() {
		while(isRunning) {
			//pause
			while(pause) { try{Thread.sleep(1); } catch(Exception e) {} }//lock thread
			//get a packet; pauses while waiting for a packet
			try { inSocket.receive(receivePacket); } catch (IOException e) { }
			//assign packet
			receivePacketQueue.add(receivePacket.getAddress());
			receivePortQueue.add(receivePacket.getPort());
			receiveQueue.add(receivePacket.getData());
			receivePacket.setData(new byte[packetSize]);
			//run functions
			if(!threadFuncts.isEmpty()) {
				for(int i = 0; i < threadFuncts.size(); i++)
				threadFuncts.get(i).execute();
			}
			try{Thread.sleep(1); } catch(Exception e) {}
		}}
	}//ReceivePacketThread
}//UDPReceive