
package Comm.TCP.Conn;

import Comm.ParamFunction;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;

/**
 * Creates either a TCP client connection to a server, or opens a server port, and 
 * waits for a client to connect. When connected, any received strings are registered.
 * 
 * Notes:
 * - isClient: Client(true) or Server(false). The only difference between them is Server takes a ServerSocket.
 * - Functions starting with client/server only work for if it is a client/server.
 * - IP is only needed for a client, not a server.
 * - ServerConnection() starts listening for a connection. ClientConnection() tries to connect, and can fail.
 * - The ReceiveThread is created and started after successfully running *Connection. 
 * - The ReceiveThead is stopped and killed after stopping the connection.
 * - IP and Port can be changed with *ChangeSettings(). They cannot be changed while a connection is running.
 * - received Strings are stored in receivedString, but it will be replaced by a newer String coming in.(no buffer)
 * - you can enter a function into the ReceiveThread by adding a ParamFunction with addReceiveFunct().
 * - receivedString will not be updated while running your function, so use receivedString() inside the function to 
 * work with it.
 * 
 * to use:
 * client - construct with TCPConnection(IP, port), and run clientConnection().
 * server - construct with TCPConnection(port), and run serverConnection().
 * 
 * @version 1.0
 * @author Dennis Liu, YURT 2012
 */
public class TCPConnection {
	/** IP of the connection. For Client use only. */
	private String IP;
	/** port number of the connection. */ 
	private int port;
	/** whether the socket is created. */ 
	private boolean socketMade;
	/** whether this connection is a client(true) or server(false).*/
	private boolean isClient;
	/** socket used by the connection. */
	private Socket socket;
	/** handles sending strings out of the socket */
	private PrintWriter sendOutSocket;
	/** handles receiving strings coming to the socket */
	private BufferedReader readInSocket;
	/** creates a server socket. For use in server mode only. */
	private ServerSocket serverSocket;
	/** string holding what is received. overwrites with latest received string. */
	private String receiveString;
	/** A thread that receives strings from the socket. Always runs. */
	private ReceiveThread RThread;
	private ArrayList<ParamFunction> receiveFuncts;
	//------------------------		CONSTRUCTOR			------------------------//
	/**
	 * Create a default TCP connection, with no IP or port.
	 * Use *ChangeSettings() to overwrite defaults.
	 * @param isClient if this connection is a client(true) or a server(false).
	 */
	public TCPConnection(boolean isClient) {
		this("",0, isClient);
	}
	/**
	 * Create a client connection. Since IP is only required for clients, it 
	 * assumes this is a client.
	 * @param IP IP of the server to connect to.
	 * @param port port number to connect to.
	 */
	public TCPConnection(String IP, int port) {
		this(IP, port, true);
	}
	/**
	 * create a server connection. IP is not required.
	 * @param port port number server connection to listen to.
	 */
	public TCPConnection(int port) {
		this("", port, false);
	}
	/**
	 * create a TCP connection. IP is only used in the client version.
	 * @param IP IP of the server to connect to.
	 * @param port port number to connect or listen to.
	 * @param isClient whether this connection is a client(true) or server(false).
	 */
	public TCPConnection(String IP, int port, boolean isClient) {
		this.IP = IP;
		this.port = port;
		socket = null;
		sendOutSocket = null;
		readInSocket = null;
		serverSocket = null;
		socketMade = false;
		this.isClient = isClient;
		receiveString = "";
		receiveFuncts = new ArrayList<ParamFunction>();
	}
	//------------------------		ACCESSOR			------------------------//
	/**
	 * CLIENT TYPE ONLY
	 * returns the IP of the connection, only if the connection is a client type.
	 * If this is a server type, it will return an empty string.
	 * @return IP of the connection.
	 */
	public String getIP() {
		if(isClient) return IP;
		else return "";
	}
	/**
	 * BOTH
	 * @return the port connecting/listening to.
	 */
	public int getPort() {
		return port;
	}
	/**
	 * BOTH
	 * @return is socket created, and connection made.
	 */
	public boolean isSocketMade() {
		return socketMade;
	}
	/**
	 * BOTH
	 * @return whether this connection is a client(true) or server(false).
	 */
	public boolean isClient() {
		return isClient;
	}
	//--------------------------		CLIENT			------------------------//
	/**
	 * CLIENT TYPE ONLY
	 * change IP and port of the TCP connection. Cannot be changed if connection
	 * is made. Also, this method does not work if connection is not client type.
	 * @param IP IP to connect to
	 * @param port port to connect to
	 * @return if settings were changed
	 */
	public boolean clientChangeSettings(String IP, int port) {
		if(isClient && !socketMade) {
			this.IP = IP;
			this.port = port;
			return true;
		} else return false;
	}
	/**
	 * CLIENT TYPE ONLY
	 * Try and connect to a server, IP and port. If connection is a server type,
	 * returns false.
	 * @param IP IP to connect to
	 * @param port port to connect to
	 * @return if connection is made.
	 */
	public boolean clientConnection(String IP, int port) {
		if(isClient) {
			clientChangeSettings(IP, port);
			return clientConnection();
		} return false;
	}
	/**
	 * CLIENT TYPE ONLY
	 * create a connection with the server. Also starts the receive Thread to get
	 * strings from the server. makes a thread that tries to connect to server.
	 * @return connection successful or not.
	 */
	public boolean clientConnection() {
		if(isClient) {
			try {//make thread wait for a client.
				return socketConnection(new Socket(IP, port));
			} catch (IOException ex) { return false; }
		} return false;
	}
	//--------------------------		SERVER			------------------------//
	/**
	 * SERVER TYPE ONLY
	 * Change the port of the server connection. Cannot be changed if connection
	 * is made.
	 * @param port port number to change to.
	 * @return if change was successful.
	 */
	public boolean serverChangeSettings(int port) {
		if(!isClient && !socketMade) {
			this.port = port;
			return true;
		} else return false;
	}
	/**
	 * SERVER TYPE ONLY
	 * changes the port and opens the socket.
	 * @param port port to change to.
	 * @return if socket successfully opened.
	 */
	public boolean serverConnection(int port) {
		if(!isClient) {
			serverChangeSettings(port);//change port
			return serverConnection();//open socket
		} else return false;
	}
	/**
	 * SREVER TYPE ONLY
	 * opens a socket for a client to connect to.  Creates a thread to listen for
	 * the port.
	 * @return if socket successfully opened.
	 */
	public boolean serverConnection() {
		if(!isClient) {
			//make thread wait for a client.
			Thread t = new Thread() {
				public void run() {
					try {
						serverSocket = new ServerSocket(port);
						socketConnection(serverSocket.accept());
					} catch (IOException ex) { }
				}
			};
			t.start();
			for(int i = 0; i < 1000 && (serverSocket == null); i++)
				if(serverSocket != null) break;
				else System.out.println(i);
			return (serverSocket != null);
		} return false;
	}
	//----------------------		Connections			------------------------//
	/**
	 * finish a socket connection, by allowing send/receive from socket, and 
	 * starting the receive thread. If construction fails, all variables are reset
	 * to null.
	 * @param s socket to finish constructing.
	 * @return if method was successful.
	 */
	private boolean socketConnection(Socket s) {
		try {
			socket = s;
			//create send/receive cabibilities
			sendOutSocket = new PrintWriter(socket.getOutputStream(), true);
			readInSocket = new BufferedReader(new InputStreamReader(socket.getInputStream()));
			RThread = new ReceiveThread(this);//start receiveThread
			RThread.start();
			//System.out.println("client connected");
			return socketMade = true;
		} catch (UnknownHostException ex) {	} catch (IOException ex) { }
		//System.out.println("client connection broke");
		socket = null;
		sendOutSocket = null;
		readInSocket = null;
		return socketMade = false;
	}
	//--------------------------		BOTH			------------------------//
	/**
	 * BOTH
	 * send a string to destination. Will only work if Connection is made. 
	 * @param s String to send.
	 */
	public void sendString(String s) {
		if(socketMade) sendOutSocket.println(s);
	}
	
	/**
	 * BOTH
	 * Returns the most current received string. Note that this can still be called without
	 * a connection. receivedStrings are not kept, and are overwritten by new received Strings.
	 * @return the most current string received from the connection. Returns null if connection is lost.
	 */
	public String receivedString() {
		if(receiveString == null) return null;
		String s = receiveString.substring(0);
		//receiveString = "";
		return s;
	}
	/**
	 * BOTH
	 * closes the connections, and turns them to null. If there is no connection,
	 * returns false.
	 * @return if closing the connections succeeds, return true. else false.
	 */
	public boolean closeConnection() {
		if(socketMade) {
			try {
				//close all sockets and connections. ORDER MATTERS (apparently).
				sendOutSocket.close();
				readInSocket.close();
				socket.close();
				if(!isClient) {//for server, serverSocket needs to be closed too.
					serverSocket.close();
					serverSocket = null;
				}
				//reset all sockets and connectors to null.
				readInSocket = null;
				sendOutSocket = null;
				socket = null;
				socketMade = false;
				RThread.pause = false;//kill receiveThread
				RThread.isRunning = false;
				RThread = null;
				receiveString = "";
				return true;
			} catch (IOException ex) {
				return false;
			}
		} else return false;
	}
	//-----------------------		Thread-related			--------------------------------//
	/**
	 * Set the function to run at each iteration for the ReceiveThread
	 * @param receiveFunct function to run
	 */
	public boolean addReceiveFunct(ParamFunction receiveFunct) {
		if(!socketMade)
			return this.receiveFuncts.add(receiveFunct);
		return false;
	}
	public boolean removeReceiveFunct(ParamFunction receiveFunct) {
		if(!socketMade)
			return this.receiveFuncts.remove(receiveFunct);
		return false;
	}
	/**
	 * get a function running at each iteration for the ReceiveThread
	 * @return a function running in the receiveThread. Null if nothing set.
	 */
	public ParamFunction getReceiveFunct(int index) {
		if(receiveFuncts.isEmpty()) return null;
		else return receiveFuncts.get(index);
	}
	/**
	 * Thread that receives Strings from the socket.
	 * String is stored in variable receiveString.
	 */
	private class ReceiveThread extends Thread {
		public TCPConnection tcp;
		boolean isRunning = true, pause = false;
		
		public ReceiveThread(TCPConnection tcp) {
			this.tcp = tcp;
		}
		
		public void run() {
		while(isRunning) {
			try{
				//pause
				while(pause) { try{ Thread.sleep(1); }catch(InterruptedException e1){} }
				//if socket is made, try receiving string
				if(socketMade) tcp.receiveString = readInSocket.readLine();
				//run plugged in functions; garantees receiveString works
				if(!receiveFuncts.isEmpty()) {
					for(int i = 0; i < receiveFuncts.size(); i++) 
						receiveFuncts.get(i).execute();
				}
				try{ Thread.sleep(1); }catch(InterruptedException e1){}
			} catch(IOException ex){}
		}}
	}//ConnectionInfo class
}//class TCPConnection
