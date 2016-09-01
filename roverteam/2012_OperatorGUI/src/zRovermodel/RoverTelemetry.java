/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package zRovermodel;

import Comm.ParamFunction;
import Comm.Test.UDPClientGUI;
import Comm.UDP.UDPClient;
import java.util.ArrayList;
import java.util.TreeMap;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JFrame;

/**
 * THIS IS A TEST CLASS FOR ClientTelemetry ONLY. It also programs the example
 * results to come back from the rover, and what the expected received packets are.
 * This class creates a UDP connection with the communications protocol,
 * and processes the packets being sent/received. This means this class handles
 * both sending and receiving on the client end. This is where telemetry update
 * requests are placed and where those updates are received and processed. That
 * means the rover model is passed to this class so that it can be synchronized.
 * 
 * A note about the protocol being used:
 * r.DID.FID
 * - This is the packet to send to ask for the value of Device(DID)'s Field ID(FID)'s value.
 * r.DID.FID.VAL
 * - This is the packet that the rover sends back from a read request.
 * w.DID.FID.VAL
 * - This is a write request. No affirmation yet, to be decided.
 * 
 * @author dennis
 */
public class RoverTelemetry {
	public static final int DID_FRONTLEFT = 21;
	public static final int MODEL_FRONTLEFT = 0;
	
	public static final int FID_FRONTLEFT_CLIENTID = 0;
	public static final int FID_FRONTLEFT_POWER = 11;
	public static final int FID_FRONTLEFT_BRAKE = 12;
	public static final int FID_FRONTLEFT_DRIVERVCC = 15;
	public static final int FID_FRONTLEFT_LOADCURRENT = 16;
	public static final int FID_FRONTLEFT_TIMER = 17;
	
	private UDPClient client;
	private ReceiveParamFunction receiveFunct;
	private YURTRover rover;
	private ArrayList<ArrayList<Integer>> roverList;
	private RoverThread roverThread;
	
	public RoverTelemetry(UDPClient client) {
		this.client = client;
		//client.startUDPConnection();
		//client.startThreads();
		rover = YURTRover.buildRoverModel();
		
		receiveFunct = new ReceiveParamFunction();
		client.addReceiveFunct(receiveFunct);

		roverThread = new RoverThread();
		roverThread.start();
	}
	
	public class ReceiveParamFunction implements ParamFunction {
		String s,sORI;
		String[] s2 = new String[4];
		//I know that the receive string is updated before calling this function, so
		//I can use it.
		@Override
		public void execute() {
			s = client.getReceiveString();
			sORI = s.substring(0);
			//PROCESS THE RECEIVED STRING HERE
			//split apprently does not work, so wrote it manually.
			s = s.concat(".");
			s2[0] = s.substring(0, 1);
			s = s.substring(1);
			for(int i = 1; i < 4; i++) {
				if(s.indexOf('.') != -1) {
					s2[i] = s.substring(0, s.indexOf('.'));
					s = s.substring(s.indexOf('.')+1);
				} else break;
			}
			//System.out.printf("Rover got:%s\n", sORI);
			if(s2[0] != null) {
				if(s2[0].equals("r")) {
					int device = -1, field = -1;
					device = rover.getDeviceID(Integer.parseInt(s2[1]));
					field = rover.getFieldID(device, Integer.parseInt(s2[2]));
					System.out.printf("Rover sends:%d %d %s\n", device, field, client.getPacketAddress());
					if(field >= 0) {
						client.sendPacket(sORI+"."+rover.getComponents().get(device).getComponents().get(field).getData());
						//System.out.printf("Rover sends:%s\n", sORI+"."+rover.getComponents().get(device).getComponents().get(field).getData());
					}
				}
			}
			//PROCESS THE RECEIVED STRING HERE
		}
	}//ReceiveParamFunction class

	public class RoverThread extends Thread {
		boolean isRunning;
		int counter, defaultCounter;
		int val = 0;
		
		public RoverThread() {
			isRunning = true;
			defaultCounter = 1000;
			counter = defaultCounter;
		}
		
		public void run() {
			while(isRunning) {
				if(val > 100) val = 0;
				else val += 10;
				int device = -1, field = -1;
				device = rover.getDeviceID(21);
				field = rover.getFieldID(device, 12);
				if(field >= 0) {
					rover.getComponents().get(device).getComponents().get(field).setData(""+val);
//					if(client.sendString(rover.getComponents().get(device).getComponents().get(field).getData()))
//					;	
						//System.out.printf("Rover value:%d %d %d\n", device, field, val);
				}
				try { Thread.sleep(1000); } catch (InterruptedException ex) { }//sleep 1 s
			}
		}
	}//RoverThread
	
	public static void main(String[] a) {
		JFrame frameServer3 = new JFrame("UDP Rover");
			UDPClient client3 = new UDPClient("localhost", 4001, 4001);
			UDPClientGUI udp3 = new UDPClientGUI(client3);
			frameServer3.add(udp3);
			RoverTelemetry rover = new RoverTelemetry(client3);
		frameServer3.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frameServer3.pack();
		frameServer3.setVisible(true);
	}
}