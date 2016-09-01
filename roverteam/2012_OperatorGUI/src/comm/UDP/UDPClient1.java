/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Comm.UDP;

import Comm.ParamFunction;
import Comm.UDP.Conn.UDPConnection;
import java.util.ArrayList;
import java.util.Arrays;
import javax.swing.JFrame;

/**
 * Creates an UDP client connection. 
 * This class creates 3 threads:
 * one thread manages connection status, another handles sending keep alive packets
 * and one for handling received packets.
 * 
 * To use:
 * - construct with UDPClient(IP1, Port1, IP2, Port2)
 * - plug in *Funct functions to add function to run inside the threads
 * - start threads with startThreads()
 * - use getReceivePacket/String to handle received packets
 * - use sendCommand(bytes) to send packets
 * 
 * Several things to note:
 * - received packets are NOT stored; they are replaced. So if you want to keep a queue or log,
 * capture them outside.
 * 
 * For YURT 2012
 * @author Dennis Liu
 */
public class UDPClient1 {
	//----------------------		Constants		----------------------------//
	/** keep alive packet */ public static final byte[] KEEP_ALIVE_PACKET = new byte[]{127,127,127,2};
	/** disconnected status*/public static final String STATUS_DISCONNECTED = "disconnected";
	/** connected status*/   public static final String STATUS_CONNECTED = "connected";
	//----------------------		Variables		----------------------------//
	/** keep alive countdown timer*/      private int keepAliveMax;
	/** wait time b/t sending keep-alive*/private int keepAliveTimer;
	/** IP to receive from(self)*/        private String sendIP;
	/** port to receive packets from*/    private int receivePort;
	/** port to send packets to*/         private int sendPort;
	/** keep alive countdown*/            private int keepAlive;
	/** UDP connector*/                   private UDPConnection comm;
	/** whether a packet is received*/    private boolean gotPacket;
	/** whether connection is made*/      private boolean isConnected;
	/** whether the connector is started*/private boolean isStarted;
	//----------------------		Thread-related		------------------------//
	/** whether threads started*/              private boolean threadStarted;
	/** thread handling connection status*/    private ConnectionThread CThread;
	/** thread sending keep alive packets out*/private KeepAliveThread KAThread;
	/** connection status*/                    private String connectionStatus;
	/** function runs when connection status is changed*/private ArrayList<ParamFunction> connectionThreadFuncts;
	/** function runs when packet is received*/private ArrayList<ParamFunction> keepAlivePacketFuncts;
	/** received packet*/                      private byte[] receivePacket;
	/** receive packet, cast to string*/       private String receiveString;
	/** function that updates latest received string*/private ReceiveFunction receiveFunct;
	//----------------------		Constructor		----------------------------//
	/**
	 * creates a UDP Client with default parameters.
	 * IP = ""
	 * receivePort = -1
	 * sendIP = ""
	 * sendPort = -1
	 */
	public UDPClient1() { this("",-1,-1); }
	/**
	 * creates a UDP Client with the following parameters
	 * @param sendIP IP to connect to
	 * @param receivePort port to receive packets from
	 * @param sendPort port to send packets to
	 */
	public UDPClient1(String sendIP, int receivePort, int sendPort) {
		this.sendIP = sendIP;
		this.receivePort = receivePort;
		this.sendPort = sendPort;
		
		comm = new UDPConnection(sendIP, receivePort, sendPort, false);
		receiveString = "";
		setKeepAliveMax(2000);//when packet is received, reset to 2000.
		keepAlive = keepAliveMax;
		gotPacket = false;
		isConnected = false;
		isStarted = false;
		setKeepAliveTimer(1000);//time between sending keep alive packets
		receiveFunct = new ReceiveFunction();
		comm.addReceiveFunct(receiveFunct);
		keepAlivePacketFuncts = new ArrayList<ParamFunction>();
		connectionThreadFuncts = new ArrayList<ParamFunction>();
		
		connectionStatus = STATUS_DISCONNECTED;
		threadStarted = false;
	}
	//----------------------		Overrides		----------------------------//
	@Override
	public String toString() {
		return sendIP+" "+receivePort+" "+sendPort;
	}
	//----------------------		Controls		----------------------------//
	public boolean isSocketMade() {
		return comm.isSocketMade();
	}
	public String connectionStatus() {
		return connectionStatus;
	}
	public int getMaxPacketSize() {
		return comm.getPacketSize();
	}
	public boolean setMaxPacketSize(int packetSize) {
		return comm.setPacketSize(packetSize);
	}
	/** returns IP to receive packets from */
	public String getSendIP() {
		return sendIP;
	}
	/** returns port to receive packets from */
	public int getReceivePort() {
		return receivePort;
	}
	/** returns port to send packets to */
	public int getSendPort() {
		return sendPort;
	}
	/** whether connection is made */
	public boolean isConnected() {
		return isConnected;
	}
	/** whether this UDP connector has started sending/receiving */
	public boolean isClientStarted() {
		return isStarted;
	}
	/** whether threads have started */
	public boolean getThreadStarted() {
		return threadStarted;
	}
	/** returns received String; same as getReceivePacket. */
	public String getReceiveString() {
		return receiveString;
	}
	/** returns received packet */
	public byte[] getReceivePacket() {
		return receivePacket;
	}
	/** returns the current Connection status. Status can be three types:
	 * "STATUS_DISCONNECTED"
	 * "STATUS_CONNECTED"
	 * "STATUS_CONNECTED keep_alive_time_remaining"
	 */
	public String getConnectionStatus() {
		return connectionStatus;
	}
	/** returns maximum keep alive time remaining value */
	public int getKeepAliveMax() {
		return keepAliveMax;
	}
	/** set maximum keep alive time remaining value */
	public void setKeepAliveMax(int keepAliveMax) {
		this.keepAliveMax = keepAliveMax;
	}
	/** returns keep alive timer (how long to wait between keep alives) */
	public int getKeepAliveTimer() {
		return keepAliveTimer;
	}
	/** set keep alive timer (how long to wait between keep alives) */
	public void setKeepAliveTimer(int keepAliveTimer) {
		this.keepAliveTimer = keepAliveTimer;
	}
	//-----------------		Connection Settings		----------------------------//
	public void clearReceiveQueue() {
		comm.receiveQueueClear();
	}
	/** returns UDP connector */
	public UDPConnection getUDPComm() {
		return comm;
	}
	/**
	 * set the IP to connect to
	 * @return if IP was changed
	 */
	public boolean setSendIP(String IP) {
		if(!isStarted)
			if(!IP.equals("")) {
				this.sendIP = IP;
				comm.setIP(IP);
				return true;
			}
		return false;
	}
	/**
	 * set the port to receive packets from
	 * @return if receive Port was changed
	 */
	public boolean setReceivePort(int receivePort) {
		if(!isStarted)
		if(receivePort != 0) {
			this.receivePort = receivePort;
			comm.setReceivePort(receivePort);
			return true;
		}
		return false;
	}
	/**
	 * set the port to send packets to
	 * @return if send port was changed
	 */
	public boolean setSendPort(int sendPort) {
		if(!isStarted)
		if(sendPort != 0) {
			this.sendPort = sendPort;
			comm.setSendPort(sendPort);
			return true;
		}
		return false;
	}
	/**
	 * change all settings of the UDP Client. There are no partial changes here;
	 * either all worked, or none works.
	 * 
	 * requires the parameters to be valid: IP must be non-blank, and ports are
	 * positive.
	 * 
	 * @return if changes were made.
	 */
	public boolean setUDPSettings(String sendIP, int receivePort, int sendPort) {
		if(!isStarted)
			if(!(sendIP.equals("") && receivePort <= 0 && sendPort <= 0)) {
				setSendIP(sendIP);
				setReceivePort(receivePort);
				setSendPort(sendPort);
				return true;
			}
		return false;
	}
	/**
	 * start the connector, allowing the client to send and receive. Threads are
	 * not started, so use startThreads(). Also overwrites the connector settings.
	 * @return if connector started
	 */
	public boolean startUDPConnection() {
		if(!isStarted) {
			comm.startConnection();
			comm.startSending();
			comm.startReceiving();
			comm.addReceiveFunct(receiveFunct);
			isStarted = true;
			return true;
		}
		return false;
	}
	/**
	 * start the connector, allowing the client to send and receive. Threads are
	 * not started, so use startThreads(). Also overwrites the connector settings.
	 * @return if settings were changed and connector started.
	 */
	public boolean startUDPConnection(String IP, int receivePort, int sendPort) {
		if(setUDPSettings(IP, receivePort, sendPort))
			return startUDPConnection();
		else return false;
	}//StartUDPConnection method
	/**
	 * stop the connector. Once stopped, this client will not send or receive.
	 * @return if connector stopped.
	 */
	public boolean stopUDPConnection() {
		if(isStarted) {
			comm.stopConnection();
			isStarted = false;
			return true;
		} else return false;
	}
	/**
	 * send a UDP packet.
	 * @param sendBytes packet to be sent
	 * @return if packet was sent
	 */
	public boolean sendPacket(byte[] sendBytes) {
		if(isStarted && isConnected) {
			comm.sendBytes(sendBytes);
			return true;
		} else return false;
	}
	/**
	 * send a string.
	 * @return if packet was sent
	 */
	public boolean sendString(String sendString) {
		return sendPacket(sendString.getBytes());
	}
	//----------------------		Thread Control		------------------------//
	/**
	 * start threads. If threads are not made, them create them.
	 * @return if threads started.
	 */
	public boolean startThreads() {
		if(CThread == null) {
			CThread = new ConnectionThread();
			KAThread = new KeepAliveThread();
			CThread.start();
			KAThread.start();
			threadStarted = false;
		}
		if(!threadStarted) {
			CThread.pause = false;
			KAThread.pause = false;
			return threadStarted = true;
		}
		return false;
	}
	/**
	 * pause threads. 
	 * @return if pause worked.
	 */
	public boolean pauseThreads() {
		if(threadStarted) {
			CThread.pause = true;
			KAThread.pause = true;
			threadStarted = false;
			return true;
		}
		return false;
	}
	/**
	 * stop threads; kill them.
	 * @return if kill worked.
	 */
	public boolean stopThreads() {
		if(CThread != null) {
			CThread.pause = false;
			KAThread.pause = false;
			CThread.isRunning = false;
			KAThread.isRunning = false;
			threadStarted = false;
			CThread = null;
			KAThread = null;
			return true;
		}
		return false;
	}
	/**
	 * return the function running in the connectionThread.
	 */
	public ParamFunction getConnectionThreadFunct(int index) {
		return connectionThreadFuncts.get(index);
	}
	/**
	 * add connection function. As long as function is not null, it will work.
	 * Also, function cannot be changed while thread is running. Pause or kill them to
	 * make changes.
	 * @return if function was replaced
	 */
	public boolean addConnectionThreadFunct(ParamFunction connectionThreadFunct) {
		if(!threadStarted)
			if(connectionThreadFunct != null) {
				connectionThreadFuncts.add(connectionThreadFunct);
				return true;
			}
		return false;
	}
	public boolean removeConnectionThreadFunct(ParamFunction connectionThreadFunct) {
		if(!threadStarted)
			if(connectionThreadFunct != null) {
				connectionThreadFuncts.remove(connectionThreadFunct);
				return true;
			}
		return false;
	}
	/**
	 * return the function working in the receivePacketHandleThread.
	 */
	public ParamFunction getKeepAlivePacketFunct(int index) {
		return keepAlivePacketFuncts.get(index);
	}
	/**
	 * add receive packet function. As long as function is not null, it will work.
	 * Also, function cannot be changed while thread is running. Pause or kill them to
	 * make changes.
	 * @return if function was replaced successfully
	 */
	public boolean addKeepAlivePacketFunct(ParamFunction receivePacketFunct) {
		if(!threadStarted)
			if(receivePacketFunct != null) {
				this.keepAlivePacketFuncts.add(receivePacketFunct);
				return true;
			}
		return false;
	}
	public boolean removeKeepAlivePacketFunct(ParamFunction connectionThreadFunct) {
		if(!threadStarted)
			if(connectionThreadFunct != null) {
				connectionThreadFuncts.remove(connectionThreadFunct);
				return true;
			}
		return false;
	}
	/**
	 * return the function working in the receivePacketHandleThread.
	 */
	public ParamFunction getReceiveFunct(int index) {
		return comm.getReceiveFunct(index);
	}
	/**
	 * add receive packet function. As long as function is not null, it will work.
	 * Also, function cannot be changed while thread is running. Pause or kill them to
	 * make changes.
	 * @return if function was replaced successfully
	 */
	public boolean addReceiveFunct(ParamFunction receivePacketFunct) {
		return comm.addReceiveFunct(receivePacketFunct);
	}
	public boolean removeReceiveFunct(ParamFunction receivePacketFunct) {
		return comm.removeReceiveFunct(receivePacketFunct);
	}

	//-------------------		Threads			--------------------------------//
	/**
	 * Thread handling the connection status. Use connectionStatus to get the 
	 * status of the client. There are three possible values for connectionStatus:
	 * "STATUS_DISCONNECTED"
	 * "STATUS_CONNECTED"
	 * "STATUS_CONNECTED keep_alive_time_remaining"
	 */
	public class ConnectionThread extends Thread {
		public boolean isRunning = true, pause = true;
		public ConnectionThread() {}
		public void run() {
			//System.out.printf("connectThread Started\n");
			keepAlive = keepAliveMax;
			while(isRunning) {
				while(pause) {
					try { Thread.sleep(1);
					} catch (InterruptedException e1) { }
				}
				
				if(isStarted) {
					if(keepAlive > 0) {
						try { Thread.sleep(1);
						} catch (InterruptedException e1) { e1.printStackTrace(); }
						keepAlive--;
						connectionStatus = STATUS_CONNECTED + " " + keepAlive;
						if(keepAlive <= 0) {
							connectionStatus = STATUS_DISCONNECTED;
							isConnected = false;
						}
						if(connectionThreadFuncts.isEmpty()) 
							for(int i = 0; i < connectionThreadFuncts.size(); i++)
								connectionThreadFuncts.get(i).execute();
					} else {
						try { Thread.sleep(1);
						} catch (InterruptedException e1) { e1.printStackTrace(); }
					}
					if(gotPacket) {//got packet, reset timer
						gotPacket = false;
						keepAlive = keepAliveMax;
						isConnected = true;
						connectionStatus = STATUS_CONNECTED;
						if(connectionThreadFuncts.isEmpty()) 
							for(int i = 0; i < connectionThreadFuncts.size(); i++)
								connectionThreadFuncts.get(i).execute();
					}
				}//if connection started
			}
		connectionStatus = STATUS_DISCONNECTED;
		}//run, while running
	}//ConnectionThread class
	/**
	 * Thread handling the keep alive packets sent to 'ping' the other side.
	 * Keep alive packets are sent every keepAliveTimer milliseconds.
	 * If a packet is already sent, then the timer resets.
	 */
	public class KeepAliveThread extends Thread {
		public boolean isRunning = true, pause = true;
		int keepAliveCounter;
		public KeepAliveThread() { }
		public void run() {
			keepAliveCounter = 0;
			while(isRunning) {
				while(pause) {
					try { Thread.sleep(1);
					} catch (InterruptedException e1) { }
				}
				if(isStarted) {
					if(!comm.isSendEmpty()) keepAliveCounter = keepAliveTimer;
					else if(comm.isSendEmpty() && keepAliveCounter == 0) {
						comm.sendBytes(KEEP_ALIVE_PACKET);
						if(keepAlivePacketFuncts.isEmpty()) 
							for(int i = 0; i < keepAlivePacketFuncts.size(); i++)
								keepAlivePacketFuncts.get(i).execute();
						keepAliveCounter = keepAliveTimer;
					} else {
						keepAliveCounter--;
						try { Thread.sleep(1);
						} catch (InterruptedException e1) { }
					}
				} else {
					try { Thread.sleep(1);
					} catch (InterruptedException e1) { }
				}//if connection started
		}}//run, while running
	}//KeepAliveThread class
	/**
	 * function that updates the received string to the latest received string.
	 */
	public class ReceiveFunction implements ParamFunction {
		@Override
		public void execute() {
			receivePacket = comm.popReceiveQueue();
			//receivePacket = comm.getReceiveQueue(comm.getReceiveQueueSize()-1);
			
			//receiveString = Arrays.toString(receivePacket);
			receiveString = new String(receivePacket);
			System.out.printf("%s %d %d:%s:\n",sendIP,receivePort,sendPort,receiveString);
			gotPacket = true;
		}
	}
	
	
	
	
	public static void main(String[] a) {
		JFrame frameServer = new JFrame("TCP GUI");
			UDPClient1 udp = new UDPClient1("localhost",4000,4001);
		frameServer.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frameServer.pack();
		frameServer.setVisible(true);
	}
}
