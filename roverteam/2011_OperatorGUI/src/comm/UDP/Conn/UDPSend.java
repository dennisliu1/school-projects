
package comm.UDP.Conn;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.ArrayList;

import comm.ParamFunction;

/**
 * Creates a thread that sends packets to a destination. 
 * To send a packet, use sendBytes to add a packet to the queue.
 * Queue has a maximum size of MAX_INTEGER.
 * 
 * @version 1.0
 * @author Dennis Liu, YURT 2012
 */
public class UDPSend {
	/** Running status for the thread */
	public static final String STATUS_STARTED = "started";
	/** Paused status for the thread */
	public static final String STATUS_PAUSED = "paused";
	/** Stopped status for the thread */
	public static final String STATUS_STOPPED = "stopped";
	//-----------------------		Variables			--------------------------//
	/** Socket to send packets out of */
	private DatagramSocket outSocket;
	/** IP to send packets to */
	private String address;
	/** port to send packets to */
	private int sendPort;
	/** queue of packets to send */
	private ArrayList<byte[]> sendQueue;
	/** Status of the thread */
	private String threadStatus;
	/** Functions running at each iteration of the thread */
	private ArrayList<ParamFunction> threadFuncts;
	/** thread that sends packets */
	private SendThread SThread;
	//-----------------------		Constructor		--------------------------//
	/**
	 * Constructs a send thread that sends packets to IP/port destination.
	 * @param inSocket socket to send packets from
	 * @param address address to send packets to
	 * @param port port to send packets to
	 * @param sendQueue queue storing packets to send
	 */
	public UDPSend(DatagramSocket inSocket, String address, int port) {
		this.outSocket = inSocket;
		this.address = address;
		this.sendPort = port;
		this.sendQueue = new ArrayList<byte[]>();
		threadStatus = STATUS_STOPPED;
		threadFuncts = new ArrayList<ParamFunction>();
		SThread = null;
	}
	public boolean setVars(DatagramSocket inSocket, String address, int port) {
		if(threadStatus.equals(STATUS_STOPPED)) {
			this.outSocket = inSocket;
			this.address = address;
			this.sendPort = port;
			return true;
		}
		return false;
	}
	//-----------------------		Accessors			------	--------------------//
	/**
	 * Send a packet to IP:Port using socket.
	 * @param packet packet to be send.
	 * @return if packet has successfully been sent.
	 */
	private boolean send(byte[] packet) {
		try {
			//System.out.printf("armSend: %d %d %d %d\n", packet[0], packet[1], packet[2], packet[3]);
			outSocket.send(new DatagramPacket(packet, packet.length, InetAddress.getByName(address), sendPort));
			return true;
		} catch(Exception e) { return false; }
	}
	public boolean sendPacket(String packet, String address, int port) {
		return sendPacket(packet.getBytes(), address, port);
	}
	public boolean sendPacket(byte[] packet, String address, int port) {
		try {
			//System.out.printf("armSend: %d %d %d %d\n", packet[0], packet[1], packet[2], packet[3]);
			outSocket.send(new DatagramPacket(packet, packet.length, InetAddress.getByName(address), port));
			return true;
		} catch(Exception e) { return false; }
	}
	public boolean sendPacket(String packet, InetAddress addr, int port) {
		try {
			//System.out.printf("armSend: %d %d %d %d\n", packet[0], packet[1], packet[2], packet[3]);
			outSocket.send(new DatagramPacket(packet.getBytes(), packet.getBytes().length, addr, port));
			return true;
		} catch(Exception e) { return false; }
	}
	public boolean sendPacket(byte[] packet, InetAddress addr, int port) {
		try {
			//System.out.printf("armSend: %d %d %d %d\n", packet[0], packet[1], packet[2], packet[3]);
			outSocket.send(new DatagramPacket(packet, packet.length, addr, port));
			return true;
		} catch(Exception e) { return false; }
	}
	/**
	 * Add a byte[] to be sent.
	 */
	public void sendBytes(byte[] packet) {
		sendQueue.add(packet);
	}
	/**
	 * Clears queue. Queue will be empty after this call.
	 */
	public void clearQueue() {
		sendQueue.clear();
	}
	/**
	 * Check whether queue is empty.
	 */
	public boolean isQueueEmpty() {
		return sendQueue.isEmpty();
	}
	/**
	 * Returns length of the queue.
	 */
	public int queueSize() {
		return sendQueue.size();
	}
	//-----------------------		Functs			------	--------------------//
	/** Add a function to run in the connection thread. Does not work while thread is running */
	public boolean addSendThreadFunct(ParamFunction threadFunct) {
		if(threadStatus.equals(STATUS_STOPPED)) {
			if(threadFunct != null) {
				threadFuncts.add(threadFunct);
				return true;
			}
		}
		return false;
	}
	/** Remove a function running in the thread. Does not work while thread is running. */
	public boolean removeSendThreadFunct(ParamFunction threadFunct) {
		if(threadStatus.equals(STATUS_STOPPED))
			if(threadFunct != null)
				return threadFuncts.remove(threadFunct);
		return false;
	}
	/** Get function index running in the thread. */
	public ParamFunction getSendThreadFunct(int index) {
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
	public boolean startSend() {
		if(SThread == null) {
			SThread = new SendThread();
			SThread.start();
		}
		if(SThread.pause) {//if thread just started
			SThread.pause = false;
			threadStatus = STATUS_STARTED;
			return true;
		}
		return false;
	}
	/**
	 * Pause receiving packets. Means packets are not sent by thread,
	 * but thread is still running.
	 */
	public boolean pauseSend() {
		if(SThread != null) {
			SThread.pause = true;
			threadStatus = STATUS_PAUSED;
			return true;
		}
		return false;
	}
	/**
	 * Stop sending packets. Kills thread.
	 */
	public boolean stopSend() {
		if(SThread != null) {
			SThread.pause = false;
			SThread.isRunning = false;
			SThread = null;
			threadStatus = STATUS_STOPPED;
			return true;
		}
		return false;
	}
	/**
	 * Thread that sends packets to the destination IP/port.
	 */
	public class SendThread extends Thread {
		private boolean isRunning = true;
		private boolean pause = true;
		@Override
		public void run() {
		while(isRunning) {
			//pause
			while(pause) { try{Thread.sleep(1);} catch(Exception e) {} }//lock thread
			//sleep when queue is empty
			while(sendQueue.isEmpty()) {
				try { Thread.sleep(1);
				} catch (InterruptedException e1) { e1.printStackTrace(); }
			}
			//try and send a byte[]
			send(sendQueue.get(0));
			if(!sendQueue.isEmpty()) try {
				sendQueue.remove(0);
			} catch(Exception e) {}
			//run functions
			if(!threadFuncts.isEmpty()) {
				for(int i = 0; i < threadFuncts.size(); i++)
				threadFuncts.get(i).execute();
			}
		}}
	}//SendThread class
}//UDPSend