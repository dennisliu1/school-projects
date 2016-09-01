/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Comm.UDP;

/**
 *
 * @author
 * Admin
 */
public class UDPInfo implements Comparable {
	public String IP;
	public int receivePort;
	public int sendPort;
	public UDPInfo(String IP, int receivePort, int sendPort) {
		this.IP = IP;
		this.receivePort = receivePort;
		this.sendPort = sendPort;
	}
	@Override
	public int compareTo(Object arg0) {
		return IP.compareTo(((UDPInfo)arg0).IP)+Math.abs(((UDPInfo)arg0).receivePort - receivePort)+Math.abs(((UDPInfo)arg0).sendPort - sendPort);
	}
	@Override
	public boolean equals(Object arg0) {
		return (IP.equals(((UDPInfo) arg0).IP) && receivePort == ((UDPInfo) arg0).receivePort && sendPort == ((UDPInfo) arg0).sendPort );
	}
	@Override
	public String toString() {
		return IP+" "+receivePort+" "+sendPort;
	}
}
