/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Comm.UDP;

import Comm.ParamFunction;
import Comm.Test.UDPClientGUI;
import java.util.TreeMap;
import javax.swing.JFrame;

/**
 *
 * @version 1.0
 * @author Dennis Liu, YURT 2012
 */
public class UDPServer {
	private TreeMap<UDPInfo,UDPClient> comm;
	
	public UDPServer() {
		comm = new TreeMap<UDPInfo,UDPClient>();
	}
	//----------------------		connecting			------------------------//
	public int size() {
		return comm.size();
	}
	//String IP, int receivePort, int sendPort
	/**
	 * start a connection with IP, receivePort, sendPort
	 * @param IP IP of the UDP connection
	 * @param receivePort port to receive data from 
	 * @param sendPort port to send data to
	 * @return if connection is started
	 */
	public boolean startConnection(String IP, int receivePort, int sendPort) {
		return startConnection(new UDPInfo(IP, receivePort, sendPort));
	}
	/**
	 * start a connection with IP, receivePort, sendPort
	 * @param info IP, receivePort, sendPort to connect to
	 * @return start the UDP connection and start the required threads
	 */
	public boolean startConnection(UDPInfo info) {
		if(comm.get(info) != null) {
			comm.get(info).startUDPConnection();
			comm.get(info).startThreads();
			return true;
		}
		return false;
	}
	/**
	 * Stop the connection.
	 * @param IP IP of the UDP connection
	 * @param receivePort port to receive data from 
	 * @param sendPort port to send data to
	 * @return if connection is stopped
	 */
	public boolean stopConnection(String IP, int receivePort, int sendPort) {
		return stopConnection(new UDPInfo(IP, receivePort, sendPort));
	}
	/**
	 * Stop the UDP connection.
	 * @param info IP, receivePort, sendPort to connect to
	 * @return stop the UDP connection and stop the required threads
	 */
	public boolean stopConnection(UDPInfo info) {
		if(comm.get(info) != null) {
			comm.get(info).stopThreads();
			comm.get(info).stopUDPConnection();
			return true;
		}
		return false;
	}
	//----------------------		FUNCTIONS			------------------------//
	/**
	 * add a function to run in the connection status Thread of the IP/port.
	 * @param info IP/port to add function to
	 * @param funct function to add
	 * @return if function was added
	 */
//	public boolean addConnectionFunct(UDPInfo info, ParamFunction funct) {
//		if(comm.get(info) != null) {
//			return comm.get(info).addConnectionThreadFunct(funct);
//		}
//		return false;
//	}
	/**
	 * removes a specific function running in the connection status Thread of the IP/port.
	 * @param port IP/port to remove function
	 * @param funct function to remove
	 * @return if function is removed
	 */
//	public boolean removeConnectionFunct(UDPInfo info, ParamFunction funct) {
//		if(comm.get(info) != null) {
//			return comm.get(info).removeConnectionThreadFunct(funct);
//		}
//		return false;
//	}
	/**
	 * get the function at index of the IP/port.
	 * @param info IP/port to get function from
	 * @param index index of the function to get
	 * @return function at the index of the port
	 */
//	public ParamFunction getConnectionFunct(UDPInfo info, int index) {
//		if(comm.get(info) != null) {
//			return comm.get(info).getConnectionThreadFunct(index);
//		}
//		return null;
//	}
	/**
	 * add a function to run in the receive Thread of the IP/port.
	 * @param info IP/port to add function to
	 * @param funct function to add
	 * @return if function was added
	 */
//	public boolean addKeepAliveFunct(UDPInfo info, ParamFunction funct) {
//		if(comm.get(info) != null) {
//			return comm.get(info).addKeepAlivePacketFunct(funct);
//		}
//		return false;
//	}
	/**
	 * removes a specific function running in the receive Thread of the IP/port.
	 * @param port IP/port to remove function
	 * @param funct function to remove
	 * @return if function is removed
	 */
//	public boolean removeKeepAliveFunct(UDPInfo info, ParamFunction funct) {
//		if(comm.get(info) != null) {
//			return comm.get(info).removeKeepAlivePacketFunct(funct);
//		}
//		return false;
//	}
	/**
	 * get the function at index of the IP/port.
	 * @param info IP/port to get function from
	 * @param index index of the function to get
	 * @return function at the index of the port
	 */
//	public ParamFunction getKeepAliveFunct(UDPInfo info, int index) {
//		if(comm.get(info) != null) {
//			return comm.get(info).getKeepAlivePacketFunct(index);
//		}
//		return null;
//	}
	/**
	 * get the TCP connector. 
	 * @param info info to get.
	 * @return connection port.
	 */
	public UDPClient get(UDPInfo info) {
		if(comm.containsKey(info)) {
			return comm.get(info);
		}
		return null;
	}
	public UDPClient get(String IP, int receivePort, int sendPort) {
		UDPInfo info = new UDPInfo(IP, receivePort, sendPort);
		return get(info);
	}
	//----------------------		Allocation			------------------------//
	public boolean allocateSocket(String IP, int receivePort, int sendPort) {
		UDPInfo info = new UDPInfo(IP, receivePort, sendPort);
		return allocateSocket(info);
	}
	public boolean allocateSocket(UDPInfo info) {
		if(comm.get(info) == null) {
			comm.put(info,new UDPClient(info.IP, info.receivePort, info.sendPort));
			return true;
		}
		return false;
	}
	public boolean deallocateSocket(UDPInfo info) {
		if(comm.get(info) != null) {
			comm.get(info).stopUDPConnection();
			comm.remove(info);
			return true;
		} 
		return false;
	}
	//----------------------		MAIN			------------------------//
	public static void main(String[] a) {
		UDPServer server = new UDPServer();
		server.allocateSocket("localhost", 4000, 4001);
		server.startConnection("localhost", 4000, 4001);
		
		
		JFrame frameServer = new JFrame("TCP GUI");
			
		frameServer.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frameServer.pack();
		frameServer.setVisible(true);
	}
}//UDPServer class
