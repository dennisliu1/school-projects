
package comm.UDP.Conn;

import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.ArrayList;

import comm.ParamFunction;

/**
 * Opens a socket and allows sending/receiving from that socket. Sends through a
 * queue and receives packets and places into another queue.
 * use: construct class, run:
 * startReceiving to open up the receiving
 * startSending to allow sending
 * to send a command, use sendBytes(arr)
 * @version 1.0
 * @author Dennis Liu, YURT 2012
 */
public class UDPConnection {
	//----------------		CONSTANTS		------------------------------------/
	public static final String STOP_STATUS = "stop";
	public static final String START_STATUS = "start";
	public static final String PAUSE_STATUS = "pause";
	private static final int DEFAULT_PACKET_SIZE = 100;
	private final String DefIP = "192.168.3.1";
	private final int DefPort = 30002;
	//----------------		VARIABLES		------------------------------------/
	public boolean debugON = false;
	private String commStatus;
	private String receiveStatus;
	private String sendStatus;
	private DatagramSocket socket;
	private String sendIP;
	private int receivePort;
	private int sendPort;
	private UDPSend sendHandler;
	private UDPReceive receiveHandler;
	private boolean socketMade;
	private ArrayList<byte[]> receiveQueue;
	private ArrayList<InetAddress> receiveAddress;
	private ArrayList<Integer> receivePorts;
	private int packetSize;
	//----------------------		CONSTRUCTOR		----------------------------//
	/**
	 * Creates A Socket with two queues for controlling send/receive.
	 * @param ip IP to send to 
	 * @param receivePort port to receive packets from
	 * @param sendPort port to send to (destination)
	 * @param wait_interval interval in milliseconds between sending
	 * @param debugON debug messages turn on or off
	 */
	public UDPConnection(String ip, int receivePort, int sendPort, boolean debugON ) {
		//save variables
		this.sendIP = ip;
		this.receivePort = receivePort;
		this.sendPort = sendPort;
		this.debugON = debugON;
		if(this.debugON) System.out.printf("IP:%s receivePort:%d sendPort: %d\n", sendIP, this.receivePort, this.sendPort);
		//initialize Socket and Threads
		receiveQueue = new ArrayList<byte[]>();
		receiveAddress = new ArrayList<InetAddress>();
		receivePorts = new ArrayList<Integer>();
		packetSize = DEFAULT_PACKET_SIZE;
		socket = null;
		sendHandler = new UDPSend(socket, sendIP, sendPort);
		receiveHandler = new UDPReceive(socket, receiveQueue, receiveAddress, receivePorts, packetSize);
		commStatus = STOP_STATUS;
		receiveStatus = STOP_STATUS;
		sendStatus = STOP_STATUS;
		socketMade = false;
	}
	//----------------------		ACCESSORS		----------------------------//
	public boolean isSocketMade() {
		return socketMade;
	}
	/**
	 * Returns the maximum packet size this connection can receive.
	 */
	public int getPacketSize() {
		return packetSize;
	}
	/**
	 * Sets the maximum packet size this connection can receive.
	 * Works when receive thread is not running.
	 * @param packetSize new maximum packet size
	 * @return if size was changed.
	 */
	public boolean setPacketSize(int packetSize) {
		if(receiveStatus.equals(STOP_STATUS)) {
			this.packetSize = packetSize;
			receiveHandler.setMaxPacketSize(this.packetSize);
			return true;
		}
		return false;
	}
	/** Returns IP of destination */
	public String getIP() {
		return sendIP;
	}
	/** 
	 * Sets the IP of destination. Can only be changed when socket is not made.
	 * @return whether IP is set
	 */
	public boolean setIP(String ip) {
		if(socket == null) {
			sendIP = ip;
			return true;
		}
		return false;
	}
	/** Returns port to receive packets from. */
	public int getReceivePort() {
		return receivePort;
	}
	/** Sets the port to receive packets from. Can only be changed when socket is not started. */
	public boolean setReceivePort(int port) {
		if(socket == null) {
			receivePort = port;
			return true;
		} else return false;
	}
	/** Returns port to send packets to. */
	public int getSendPort() {
		return sendPort;
	}
	/** 
	 * Sets the port to send packets to. 
	 * @param sendIP IP to send to
	 */
	public boolean setSendPort(int port) {
		if(socket == null) {
			sendPort = port;
			return true;
		} else return false;
	}
	//----------------------		THREAD FUNCTIONS		----------------------------//
	/**
	 * Send bytes from socket to IP/sendPort.
	 * @param arr byte packet to send
	 */
	public void sendBytes(byte[] arr) {
		sendHandler.sendBytes(arr);
	}
	public void sendPacket(String packet, String address, int port) {
		sendHandler.sendPacket(packet, address, port);
	}
	public void sendPacket(byte[] packet, String address, int port) {
		sendHandler.sendPacket(packet, address, port);
	}
	public void sendPacket(String packet, InetAddress address, int port) {
		sendHandler.sendPacket(packet, address, port);
	}
	public void sendPacket(byte[] packet, InetAddress address, int port) {
		sendHandler.sendPacket(packet, address, port);
	}
	
	/**
	 * returns whether send queue is empty
	 */
	public boolean isSendEmpty() {
		return sendHandler.isQueueEmpty();
	}
	/**
	 * Pop the most recent received packet.
	 */
	public byte[] popReceiveQueue() {
		byte[] r = receiveQueue.get(0);
		receiveQueue.remove(0);
		return r;
	}
	/**
	 * Get a received packet.
	 */
	public byte[] getReceiveQueue(int index) {
		return receiveQueue.get(index);
	}
	/**
	 * Get the least recent received packet.
	 */
	public int getReceiveQueueSize() {
		return receiveQueue.size();
	}
	/**
	 * returns whether receive queue is empty
	 */
	public boolean isReceiveEmpty() {
		return sendHandler.isQueueEmpty();
	}
	public void receiveQueueClear() {
		receiveQueue.clear();
	}
	public InetAddress popReceiveAddress() {
		InetAddress addr = receiveAddress.get(0);
		receiveAddress.remove(0);
		return addr;
	}
	public int popReceivePort() {
		int port = receivePorts.get(0);
		receivePorts.remove(0);
		return port;
	}
	//----------------------		THREAD FUNCTIONS		----------------------------//
	/** Add a function to run in the connection thread. Does not work while thread is running */
	public boolean addReceiveFunct(ParamFunction threadFunct) {
		if(receiveStatus.equals(STOP_STATUS)) {
			if(threadFunct != null) {
				receiveHandler.addReceiveThreadFunct(threadFunct);
				return true;
			}
		}
		return false;
	}
	/** Remove a function running in the thread. Does not work while thread is running. */
	public boolean removeReceiveFunct(ParamFunction threadFunct) {
		if(receiveStatus.equals(STOP_STATUS))
			if(threadFunct != null)
				return receiveHandler.removeReceiveThreadFunct(threadFunct);
		return false;
	}
	/** Get function index running in the thread. */
	public ParamFunction getReceiveFunct(int index) {
		return receiveHandler.getReceiveThreadFunct(index);
	}
	/** Add a function to run in the connection thread. Does not work while thread is running */
	public boolean addSendFunct(ParamFunction threadFunct) {
		if(sendStatus.equals(STOP_STATUS)) {
			if(threadFunct != null) {
				sendHandler.addSendThreadFunct(threadFunct);
				return true;
			}
		}
		return false;
	}
	/** Remove a function running in the thread. Does not work while thread is running. */
	public boolean removeSendFunct(ParamFunction threadFunct) {
		if(sendStatus.equals(STOP_STATUS))
			if(threadFunct != null)
				return sendHandler.removeSendThreadFunct(threadFunct);
		return false;
	}
	/** Get function index running in the thread. */
	public ParamFunction getSendFunct(int index) {
		return sendHandler.getSendThreadFunct(index);
	}
	//----------------------		START COMM		----------------------------//
	/** 
	 * Start receiving thread.
	 * @return whether receiving thread started
	 */ 
	public boolean startReceiving() {
		if(socketMade) {
			receiveHandler.startReceive();
			receiveStatus = START_STATUS;
			return true;
		} return false;
	}
	/** 
	 * Start sending thread.
	 * @return whether sending thread started
	 */ 
	public boolean startSending() { 
		if(socketMade) {
			sendHandler.startSend();
			sendStatus = START_STATUS;
			return true;
		} return false;
	}
	/**
	 * Start socket connection. Creates the socket.
	 * @return whether socket is made.
	 */
	public boolean startConnection() {
		try { 
			socket = new DatagramSocket(this.receivePort);
			sendHandler.setVars(socket, sendIP, sendPort);
			receiveHandler.setVars(socket, receiveQueue, packetSize);
			commStatus = START_STATUS;
			receiveStatus = START_STATUS;
			sendStatus = START_STATUS;
			return socketMade = true;
		} catch(Exception e) { 
			if(this.debugON) System.out.printf("CommMain: socket failed\n"); 
			return false;
		}
	}
	//----------------------		PAUSE COMM		----------------------------//
	/**
	 *  Pause receiving thread. Fails if socket is not made.
	 * @return whether receiving thread paused.
	 */
	public boolean pauseReceiving() {
		if(socketMade) {
			receiveHandler.pauseReceive(); 
			receiveStatus = PAUSE_STATUS;
			return true;
		} return false;
	}
	/** 
	 * Pause sending thread. Fails if socket is not made.
	 * @return whether sending thread paused.
	 */ 
	public boolean pauseSending() { 
		if(socketMade) {
			sendHandler.pauseSend();
			sendStatus = PAUSE_STATUS;
			return true;
		} return false;
	}
	//----------------------		STOP COMM		----------------------------//
	/**
	 * Stop receiving thread. Works only when socket is made.
	 * @return whether receiving thread stopped
	 */
	public boolean stopReceiving() {
		if(socketMade) {
			receiveHandler.stopReceive(); 
			receiveStatus = STOP_STATUS;
			return true;
		} return false;
	}
	/** 
	 * Stop sending thread. Works only when socket is made.
	 * @return whether sending thread stopped
	 */ 
	public boolean stopSending() { 
		if(socketMade) {
			sendHandler.stopSend();
			sendStatus = STOP_STATUS;
			return true;
		} return false;
	}
	/**
	 * Stop connection. Kills threads and socket. Does not work if connection is made.
	 * @return whether connection is stopped.
	 */
	public boolean stopConnection() {
		if(socket != null) {
			try {
				receiveHandler.stopReceive();
				sendHandler.stopSend();
				socket.close();
				socket = null;
				commStatus = STOP_STATUS;
				receiveStatus = STOP_STATUS;
				sendStatus = STOP_STATUS;
				return true;
			} catch(Exception e) { 
				if(this.debugON) System.out.printf("CommMain: socket failed\n"); 
				return false;
			}
		} return false;
	}
}//CommMain class
