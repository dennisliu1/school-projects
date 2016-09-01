package GUI;

import javax.swing.JFrame;

import Comm.TCP.TCPClient;

public class CompassTelemetry {
	
	TCPClient client;
	long[] degree;
	double[] tiltx;
	double[] tilty;
	private SendThread sendThread;
	
	public CompassTelemetry(TCPClient client, long[] degree, double[] tiltx, double[] tilty) {
		this.client = client;
		this.degree = degree;
		this.tiltx = tiltx;
		this.tilty = tilty;
		sendThread = new SendThread();
		sendThread.start();
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
						degree[0] = Long.parseLong(s.substring(0,s.indexOf(' ')));
						s = s.substring(s.indexOf(' ')+1);
						tiltx[0] = Long.parseLong(s.substring(0,s.indexOf(' ')));
						s = s.substring(s.indexOf(' ')+1);
						tilty[0] = Long.parseLong(s);
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
