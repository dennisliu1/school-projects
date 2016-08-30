package baseOperations;


import java.awt.FlowLayout;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JFrame;

import comm.ParamFunction;
import comm.Test.UDPClientGUI;
import comm.UDP.UDPClient;

import zRovermodel.ClientTelemetry;
import zRovermodel.Component;
import zRovermodel.Robot;
import zRovermodel.SerializedComponent;
import zRovermodel.RoverTelemetry;
import zRovermodel.YURTRover;

/**
 * This class creates a UDP connection with the communications protocol,
 * and processes the packets being sent/received. This means this class handles
 * both sending and receiving on the client end. This is where telemetry update
 * requests are placed and where those updates are received and processed. That
 * means the rover model is passed to this class so that it can be synchronized.
 * 
 * A note about the protocol being used:
 * rDID.FID
 * - This is the packet to send to ask for the value of Device(DID)'s Field ID(FID)'s value.
 * rDID.FID.VAL
 * - This is the packet that the rover sends back from a read request.
 * wDID.FID.VAL
 * - This is a write request. No affirmation yet, to be decided.
 * 
 * connect to basestation with wire, set udp to its ip: 192.168.80.20 port 5000,5000
 * basestation should run telemetrysplitter, and set its own ip 192.168.80.20 port 5000,5000 for the split part.
 * 
 * @author dennis
 */
public class TelemetrySplitter {
	private ClientTelemetry client;
	private ArrayList<ArrayList<double[]>> telemetry;
	
	private UDPClient receiver;
	private SplitterParamFunction splitFunct;
	
	public TelemetrySplitter(ClientTelemetry client, UDPClient receiver) {
		this.client = client;
		this.receiver = receiver;
		splitFunct = new SplitterParamFunction();
		receiver.addReceiveFunct(splitFunct);
	}
	
	public class SplitterParamFunction implements ParamFunction {
		String s,sORI;
		String[] s2 = new String[4];
		
		@Override
		public void execute() {
			//I know I call the UDPClient's paramfunction, which sets receiveString and receiveAddress. So I can use them.
			InetAddress addr = receiver.getReceiveAddress();
			int port = receiver.getReceivePort();
			
			s = receiver.getReceiveString();
			sORI = s.substring(0);
			if(sORI.startsWith("time=")) {
				if(client.time != null) try {
					String sendStr = "time="+client.time[0];
					if(addr.equals(InetAddress.getByName("localhost"))) receiver.sendPacket(sendStr, addr, port-1);
					else receiver.sendPacket(sendStr, addr, port);
				} catch(Exception e) {}
			} else if(sORI.startsWith("Quality=")) {
				if(client.time != null) try {
					String sendStr = "Quality="+client.signal[0];
					if(addr.equals(InetAddress.getByName("localhost"))) receiver.sendPacket(sendStr, addr, port-1);
					else receiver.sendPacket(sendStr, addr, port);
				} catch(Exception e) {}
			} else {
				s = s.concat(".");
				//System.out.printf("Test Send:%s \n", sORI);
				//split apprently does not work, so wrote it manually.
				for(int i = 0; i < s2.length; i++)
					s2[i] = "";
				for(int i = 0; i < 4; i++) {
					if(s.indexOf('.') != -1) {
						s2[i] = s.substring(0, s.indexOf('.'));
						s = s.substring(s.indexOf('.')+1);
					} else break;
				}
				//PROCESS THE RECEIVED STRING HERE
				if(s2[0] != null) {
					if(s2[0].equals("r")) {
						int device = -1, field = -1;
						device = client.rover.getDeviceID(Integer.parseInt(s2[1]));
						field = client.rover.getFieldID(device, Integer.parseInt(s2[2]));
						if(field >= 0) {
							if(client.rover.getComponents().size() >= device) {
								if(client.rover.getComponents().get(device).getComponents().size() >= field) {
									String sendStr = "r"+client.rover.getComponents().get(device).getIdentifier()+"."+client.rover.getComponents().get(device).getComponents().get(field).getIdentifier()+"."+client.rover.getComponents().get(device).getComponents().get(field).getData();
									try {
										if(addr.equals(InetAddress.getByName("localhost"))) receiver.sendPacket(sendStr, addr, port-1);
										else receiver.sendPacket(sendStr, addr, port);
									} catch (UnknownHostException e) { }
									System.out.printf("Test Send:%s %s %d\n", "r"+client.rover.getComponents().get(device).getIdentifier()+"."+client.rover.getComponents().get(device).getComponents().get(field).getIdentifier()+"."+client.rover.getComponents().get(device).getComponents().get(field).getData(), addr.getHostAddress(), port);
								}
							}
						}//field >= 0
					}//read section
					for(int i = 0; i < s2.length; i++)
						s2[i] = "";
				}//if format matches
			}
		}//execute()
	}//ReceiveParamFunction class	

	public static final int DEFAULT_UNUSED_RESET_TIME = -1;
	public static final int DEFAULT_UNUSED_READ_TIME = -1;
	public static final int DEFAULT_RESET_TIME = 10000;
	public static final int DEFAULT_READ_TIME = 1000;

	public static void main(String[] a) {
		
		JFrame frameServer2 = new JFrame("UDP Splitter");
		frameServer2.setLayout(new FlowLayout());
			UDPClient udpclient = new UDPClient("localhost", 4000, 4001);
			UDPClientGUI udp2 = new UDPClientGUI(udpclient);
			frameServer2.add(udp2);
			UDPClient receiver = new UDPClient("localhost", 5001, 5000);
			UDPClientGUI rGUI = new UDPClientGUI(receiver);
			frameServer2.add(rGUI);
			YURTRover r = YURTRover.defaultRoverModel();// builds a rover model for the telemetry model to use
			
			ArrayList<SerializedComponent> roverComponents = new ArrayList<SerializedComponent>();
			roverComponents.add(new YURTRover("Front22", 22, Component._dataType._int, "21"));
			roverComponents.add(new YURTRover("Rear23", 23, Component._dataType._int, "22"));
			roverComponents.add(new YURTRover("extr12", 12, Component._dataType._int, "23"));
			roverComponents.add(new YURTRover("extr14", 14, Component._dataType._int, "110"));
			roverComponents.add(new YURTRover("extr15", 15, Component._dataType._int, "12"));
			
			for(int i = 0; i < roverComponents.size(); i++) {
				//Motor 1
				roverComponents.get(i).addComponent(new YURTRover("Client ID", 0, Component._dataType._int, "-1", Robot.READ_ONLY));// Device ID
				roverComponents.get(i).addComponent(new YURTRover("Power", 11, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
				roverComponents.get(i).addComponent(new YURTRover("Brake", 12, Component._dataType._int, "1", Robot.READ_WRITE, 0, 1));// [0,1]
				roverComponents.get(i).addComponent(new YURTRover("Position", 13, Component._dataType._int, "", Robot.READ_WRITE, Integer.MIN_VALUE, Integer.MAX_VALUE));// [0,1]
				roverComponents.get(i).addComponent(new YURTRover("Brake", 14, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
				roverComponents.get(i).addComponent(new YURTRover("Driver Vcc", 15, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
				roverComponents.get(i).addComponent(new YURTRover("Load Current", 16, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
				roverComponents.get(i).addComponent(new YURTRover("Timer", 17, Component._dataType._int, "", Robot.READ_WRITE, 0, Integer.MAX_VALUE));// [0, 2^32-1]
				//Motor 2
				roverComponents.get(i).addComponent(new YURTRover("Power", 21, Component._dataType._int, "0", Robot.READ_WRITE, -128d, 127d));// [-128,127]
				roverComponents.get(i).addComponent(new YURTRover("Brake", 22, Component._dataType._int, "1", Robot.READ_WRITE, 0, 1));// [0,1]
				roverComponents.get(i).addComponent(new YURTRover("Position", 23, Component._dataType._int, "", Robot.READ_WRITE, Integer.MIN_VALUE, Integer.MAX_VALUE));// [0,1]
				roverComponents.get(i).addComponent(new YURTRover("Brake", 24, Component._dataType._int, "", Robot.READ_ONLY, 0, 7));
				roverComponents.get(i).addComponent(new YURTRover("Driver Vcc", 25, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
				roverComponents.get(i).addComponent(new YURTRover("Load Current", 26, Component._dataType._int, "", Robot.READ_ONLY, 0, 1023));// [0, 1023]
				roverComponents.get(i).addComponent(new YURTRover("Timer", 27, Component._dataType._int, "", Robot.READ_WRITE, 0, Integer.MAX_VALUE));// [0, 2^32-1]
				
				for(int j = 0; j < roverComponents.get(i).getComponents().size(); j++) {
					((SerializedComponent)roverComponents.get(i).getComponents().get(j)).setResetTimer(DEFAULT_RESET_TIME);
					((SerializedComponent)roverComponents.get(i).getComponents().get(j)).setReadTimer(DEFAULT_UNUSED_READ_TIME);
				}
				r.addComponent(roverComponents.get(i));
			}
						
			((SerializedComponent)roverComponents.get(0).getComponents().get(0)).setResetTimer(DEFAULT_RESET_TIME);
			((SerializedComponent)roverComponents.get(0).getComponents().get(0)).setReadTimer(DEFAULT_READ_TIME);
			((SerializedComponent)roverComponents.get(1).getComponents().get(0)).setResetTimer(DEFAULT_RESET_TIME);
			((SerializedComponent)roverComponents.get(1).getComponents().get(0)).setReadTimer(DEFAULT_READ_TIME);
			((SerializedComponent)roverComponents.get(2).getComponents().get(0)).setResetTimer(DEFAULT_RESET_TIME);
			((SerializedComponent)roverComponents.get(2).getComponents().get(0)).setReadTimer(DEFAULT_READ_TIME);
			((SerializedComponent)roverComponents.get(3).getComponents().get(0)).setResetTimer(DEFAULT_RESET_TIME);
			((SerializedComponent)roverComponents.get(3).getComponents().get(0)).setReadTimer(DEFAULT_READ_TIME);
			((SerializedComponent)roverComponents.get(4).getComponents().get(0)).setResetTimer(DEFAULT_RESET_TIME);
			((SerializedComponent)roverComponents.get(4).getComponents().get(0)).setReadTimer(DEFAULT_READ_TIME);
			
			ClientTelemetry client = new ClientTelemetry(udpclient, r);
			TelemetrySplitter splitter = new TelemetrySplitter(client, receiver);
			ArrayList<double[]>  speeds = new ArrayList<double[]>();
			for(int i = 0; i < 4; i++) speeds.add(new double[1]);
			client.addTelemetry(22,21, speeds.get(0));
			client.setTelemetryReadTime(22, 11, 1000);
			client.addTelemetry(23,21, speeds.get(1));
			client.setTelemetryReadTime(22, 21, 1000);
			client.addTelemetry(22,11, speeds.get(2));
			client.setTelemetryReadTime(23, 11, 1000);
			client.addTelemetry(23,11, speeds.get(3));
			client.setTelemetryReadTime(23, 21, 1000);
			
			
//			client.buildNames();
//			client.buildStatus();
//			client.buildVoltages();
			long[] time = new long[1];
			client.addTimeTelemetry(time);
			client.sendTime = false;
			String[] signal = new String[1];
			signal[0] = "11/12";
			client.addSignalTelemetry(signal);
//			client.addTelemetry(21,11, new double[1]);
//			client.setTelemetryReadTime(21, 11, 33);
//			client.addTelemetry(21,21, new double[1]);
//			client.setTelemetryReadTime(21, 21, 33);
//			client.addTelemetry(22,11, new double[1]);
//			client.setTelemetryReadTime(22, 11, 33);
//			client.addTelemetry(22,21, new double[1]);
//			client.setTelemetryReadTime(22, 21, 33);
			
			
			
			//tele2.addTelemetry(21, 0, new double[1]);
			//ArrayList<double[]> arr = tele2.buildStatus();
		frameServer2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frameServer2.pack();
		frameServer2.setVisible(true);
		
//		JFrame frameServer3 = new JFrame("UDP Rover");
//			UDPClient client3 = new UDPClient("localhost", 4001, 4000);
//			UDPClientGUI udp3 = new UDPClientGUI(client3);
//			frameServer3.add(udp3);
//			RoverTelemetry rover = new RoverTelemetry(client3);
//		frameServer3.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//		frameServer3.pack();
//		frameServer3.setVisible(true);
		
//		JFrame frameServer = new JFrame("UDP GUI");
//			UDPClient client = new UDPClient("localhost", 5000, 5001);
//			UDPClientGUI udp = new UDPClientGUI(client);
//			frameServer.add(udp);
//			YURTRover r2 = YURTRover.buildRoverModel();
//			ClientTelemetry tele = new ClientTelemetry(client, r2);
//			tele.addTelemetry(21, 12, new double[1]);
//		frameServer.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//		frameServer.pack();
//		frameServer.setVisible(true);
	}
}//ClientTelemetry class
