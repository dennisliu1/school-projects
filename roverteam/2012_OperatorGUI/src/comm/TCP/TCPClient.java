
package Comm.TCP;

import Comm.ParamFunction;
import Comm.TCP.Conn.TCPConnection;
import java.util.ArrayList;

/**
 * Creates a TCP Client connection. By specifying the IP and Port of the 
 * server to connect to, the connection allows TCP communications. Also
 * runs a connection status thread that returns the current connection status.
 * 
 * To use:
 * Construct with TCPClient(String IP, int port). Try connection attempt with
 * startConnection(). Start the connection Status thread with startThread().
 * 
 * @version 1.0
 * @author Dennis Liu, YURT 2012
 */
public class TCPClient {
	//----------------------		CONSTANTS		----------------------------//
	/** Connection status for disconnected connection. */
	public static final String STATUS_DISCONNECTED = "disconnected";
	/** Connection status for connected connection. */
	public static final String STATUS_CONNECTED = "connected";
	//----------------------		VARIABLES		----------------------------//
	/** TCP client connection */
	private TCPConnection tcp;
	//----------------------		THREAD-RELATED		------------------------//
	/** Whether the thread is turned on. */
	private boolean threadOn;
	/** Thread updating the connection status of the client. */
	private ConnectionThread CThread;
	/** Holds the connection status. */
	private String connectionStatus;
	/** Holds the functions that run every iteration of ConnectionThread. */
	private ArrayList<ParamFunction> connectionThreadFuncts;
	//----------------------		CONSTRUCTOR		----------------------------//
	/**
	 * Construct a TCP Client connection.
	 * @param IP IP of the server to connect to
	 * @param port port number to connect to
	 */
	public TCPClient(String IP, int port) {
		//defaults
		tcp = new TCPConnection(IP, port);
		threadOn = false;
		CThread = null;
		connectionStatus = STATUS_DISCONNECTED;
		connectionThreadFuncts = new ArrayList<ParamFunction>();
	}
	//----------------------		ACCESSORS		----------------------------//
	/** Returns client IP. */
	public String getIP() {
		return tcp.getIP();
	}
	/** Sets the IP of the TCP Client. Does not work if IP is invalid(null or "")
	 *  or if connection is already made.
	 */
	public boolean setIP(String IP) {
		if(IP != null)
		if(!tcp.isSocketMade() && !IP.equals("")) {
			tcp.clientChangeSettings(IP, tcp.getPort());
			return true;
		} return false;
	}
	/** Returns client port number. */
	public int getPort() {
		return tcp.getPort();
	}
	/** Sets the port of the TCP Client. 
	 * Works for port > 0, and connection not made.
	 */
	public boolean setPort(int port) {
		if(!tcp.isSocketMade() && port > 0) {
			tcp.clientChangeSettings(tcp.getIP(), port);
			return true;
		} return false;
	}
	public void changeClientSettings(String IP, int port) {
		setIP(IP);
		setPort(port);
	}
	//----------------------		STATUS CHECKS		----------------------------//
	/** Returns the most current received string. */
	public String receiveString() {
		return tcp.receivedString();
	}
	/** Whether a connection is made. */
	public boolean isConnected() {
		return tcp.isSocketMade();
	}
	/** Whether threads are running. */
	public boolean isThreadStarted() {
		return threadOn;
	}
	/** Returns the connection Status. Returns:
	 * STATUS_DISCONNECTED - disconnected
	 * STATUS_CONNECTED - connected
	 */
	public String getConnectionStatus() {
		return connectionStatus;
	}
	//------------------		CONNECTIONS				--------------------//
	/** Try to make a connection. */
	public boolean startConnection() {
		if(!tcp.isSocketMade())
			return tcp.clientConnection();
		return false;
	}
	/** Stop the connection. */
	public boolean stopConnection() {
		if(tcp.isSocketMade()) 
			return tcp.closeConnection();
		return false;
	}
	/** Send a string out. The string should not be empty or null. 
	 * And client is connected.
	 */
	public boolean sendString(String s) {
		if(s != null) 
			if(!s.equals("") && tcp.isSocketMade()) {
				tcp.sendString(s);
				return true;
			}
		return false;
	}
	//------------------		THREADING				--------------------//
	/** Starts the connection thread. Creates it if it is not created. */
	public boolean startThread() {
		if(CThread == null) {
			CThread = new ConnectionThread();
			CThread.start();
			threadOn = true;
		}
		if(CThread.pause) {
			CThread.pause = false;
			return true;
		}
		return false;
	}
	/** pause the connection thread. */
	public boolean pauseThreads() {
		if(threadOn) {
			CThread.pause = true;
			return true;
		}
		return false;
	}
	/** stop the thread and kill it. */
	public boolean stopThreads() {
		if(threadOn) {
			CThread.pause = false;
			CThread.isRunning = false;
			CThread = null;
			return true;
		}
		return false;
	}
	//------------------		THREADS			----------------------------//
	/** Add a function to run in the connection thread. Does not work while thread is running */
	public boolean addConnectionThreadFunct(ParamFunction connectionThreadFunct) {
		if(!threadOn) {
			if(connectionThreadFunct != null) {
				connectionThreadFuncts.add(connectionThreadFunct);
				return true;
			}
		}
		return false;
	}
	/** Remove a function running in the thread. Does not work while thread is running. */
	public boolean removeConnectionThreadFunct(ParamFunction connectionThreadFunct) {
		if(!threadOn)
			if(connectionThreadFunct != null)
				return connectionThreadFuncts.remove(connectionThreadFunct);
		return false;
	}
	/** Get function index running in the thread. */
	public ParamFunction getConnectionThreadFunct(int index) {
		if(!threadOn) {
			if(!connectionThreadFuncts.isEmpty()) {
				return connectionThreadFuncts.get(index);
			}
		}
		return null;
	}
	/**
	 * Set the function to run at each iteration for the ReceiveThread
	 * @param receiveFunct function to run
	 */
	public boolean addReceiveFunct(ParamFunction receiveFunct) {
		return tcp.addReceiveFunct(receiveFunct);
	}
	public boolean removeReceiveFunct(ParamFunction receiveFunct) {
		return tcp.removeReceiveFunct(receiveFunct);
	}
	/**
	 * get a function running at each iteration for the ReceiveThread
	 * @return a function running in the receiveThread. Null if nothing set.
	 */
	public ParamFunction getReceiveFunct(int index) {
		return tcp.getReceiveFunct(index);
	}
	/** Thread that holds the current connection status. Used by GUIs.*/
	private class ConnectionThread extends Thread {
		public boolean isRunning = true, pause = true;
		private boolean wasConnected = false;///previous connection status
		public ConnectionThread() {
		}

		@Override
		public void run() {
		while(isRunning) {
			//pause
			while(pause) { try{ Thread.sleep(1); }catch(InterruptedException e1){} }
			if(tcp.isSocketMade() != wasConnected) {
				//update status
				if(tcp.isSocketMade() && !wasConnected) connectionStatus = STATUS_CONNECTED;
				else if(!tcp.isSocketMade() && wasConnected) connectionStatus = STATUS_DISCONNECTED;
				wasConnected = tcp.isSocketMade();
				//run functions
				if(!connectionThreadFuncts.isEmpty()) {
					for(int i = 0; i < connectionThreadFuncts.size(); i++)
					connectionThreadFuncts.get(i).execute();
				}
			}
			try{ Thread.sleep(1); }catch(InterruptedException e1){}
			//System.out.println(isConnected);
		}}
	}//ReceiveThread class
}//TCPClient class
