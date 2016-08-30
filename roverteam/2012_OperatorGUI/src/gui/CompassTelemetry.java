package gui;

import gui.component.GUIComponent;

import javax.swing.JFrame;

import comm.TCP.TCPClient;

/**
 * This is the controller for the TCP Connection for the compass/client telemetry. It asks for info,
 * parses the message back, and updates the displays of the components.
 * @author dennis
 *
 */
public class CompassTelemetry {
	TCPClient client;
	long[] degree;
	double[] tiltx;
	double[] tilty;
	private SendThread sendThread;
	
	GUIComponent compassPanel, tiltPanel;
	
	public CompassTelemetry(TCPClient client, long[] degree, double[] tiltx, double[] tilty) {
		this.client = client;
		this.degree = degree;
		this.tiltx = tiltx;
		this.tilty = tilty;
		sendThread = new SendThread();
		sendThread.start();
	}
	public void setCompassPanel(GUIComponent compassPanel) {
		this.compassPanel = compassPanel;
	}
	public void setTiltPanel(GUIComponent tiltPanel) {
		this.tiltPanel = tiltPanel;
	}
	
	public class SendThread extends Thread {
		boolean isRunning = true, pause = false;
		String receiveStr = "", prevStr = "";
		
		public void run() {
			while(isRunning) {
				while(pause) try { Thread.sleep(1); } catch (InterruptedException e) { }
				if(client.getConnectionStatus().equals(TCPClient.STATUS_CONNECTED)) {
					client.sendString("G");
				}
				receiveStr = client.receiveString();
				if(!receiveStr.equals(prevStr)) {
					prevStr = receiveStr;
					String s = prevStr;
					if(prevStr.indexOf(' ') >= 0) {
						if(degree != null) {
							degree[0] = Long.parseLong(s.substring(0,s.indexOf(' ')));
							if(compassPanel != null) compassPanel.updateDisplay();
						}
						s = s.substring(s.indexOf(' ')+1);
						if(tiltx != null && tilty != null) {
							tiltx[0] = Long.parseLong(s.substring(0,s.indexOf(' ')));
							s = s.substring(s.indexOf(' ')+1);
							tilty[0] = Long.parseLong(s);
							if(tiltPanel != null) tiltPanel.updateDisplay();
						}
						
					}
					System.out.printf("got:%s %d\n", prevStr, degree[0]);
				}
				try { Thread.sleep(200); } catch (InterruptedException e) { }
			}
		}
	}//SendThread class
	
	public static void main(String[] args) {
		TCPClient client = new TCPClient("192.168.80.121", 30150);
		TCPController gui = new TCPController(client);
			CompassTelemetry tele = new CompassTelemetry(client, new long[1], new double[1], new double[1]);
		JFrame frame = new JFrame("TCP GUI");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.add(gui);
		frame.pack();
		frame.setVisible(true);
	}
}//CompassTelemetry class
