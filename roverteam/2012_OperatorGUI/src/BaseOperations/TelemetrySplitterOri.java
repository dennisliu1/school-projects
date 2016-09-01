package BaseOperations;

import Comm.ParamFunction;
import Comm.Test.UDPClientGUI;
import Comm.UDP.UDPClient;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JFrame;

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
 * r.DID.FID
 * - This is the packet to send to ask for the value of Device(DID)'s Field ID(FID)'s value.
 * r.DID.FID.VAL
 * - This is the packet that the rover sends back from a read request.
 * w.DID.FID.VAL
 * - This is a write request. No affirmation yet, to be decided.
 * 
 * @author dennis
 */
public class TelemetrySplitterOri {
	public static final int DEFAULT_RESET_TIME = 3000;
	public static final int DEFAULT_READ_TIME = 1000;
	private UDPClient client;
	private ReceiveParamFunction receiveFunct;
	private SendThread sendThread;
	private ResetThread resetThread;
	public YURTRover rover;
	private ArrayList<ArrayList<double[]>> telemetry;
	private long time[];
	
	public TelemetrySplitterOri(UDPClient client, YURTRover rover) {
		this.client = client;
		this.rover = rover;
		receiveFunct = new ReceiveParamFunction();
		client.addReceiveFunct(receiveFunct);
		telemetry = new ArrayList<ArrayList<double[]>>();//build telemetry list
		for(int i = 0; i < rover.getComponents().size(); i++) {
			telemetry.add(new ArrayList<double[]>());
			for(int j = 0; j < rover.getComponents().get(i).getComponents().size(); j++) {
				telemetry.get(i).add(null);//add placeholder
				((SerializedComponent)rover.getComponents().get(i).getComponents().get(j)).setTime(System.currentTimeMillis());
				((SerializedComponent)rover.getComponents().get(i).getComponents().get(j)).setResetTimer(-1);//set default reset timer
				((SerializedComponent)rover.getComponents().get(i).getComponents().get(j)).setReadTimer(-1);//set default read timer
			}
		}
		sendThread = new SendThread();
		sendThread.start();
		resetThread = new ResetThread();
		resetThread.start();
	}
	
	public ArrayList<double[]> buildStatus() {
		ArrayList<double[]> arr = new ArrayList<double[]>();
		for(int i = 0; i < getNumComponents(); i++) {
			arr.add(new double[1]);
		}
		for(int i = 0; i < rover.getComponents().size(); i++) {
			addTelemetry(rover.getComponents().get(i).getIdentifier(), 0, arr.get(i));
		}
		return arr;
	}
	public ArrayList<String> buildNames() {
		ArrayList<String> arr = new ArrayList<String>();
		for(int i = 0; i < rover.getComponents().size(); i++) {
			arr.add(rover.getComponents().get(i).getName());
		}
		return arr;
	}
	public ArrayList<double[]> buildVoltages() {
		ArrayList<double[]> arr = new ArrayList<double[]>();
		for(int i = 0; i < getNumComponents(); i++) {
			arr.add(new double[1]);
		}
		
//		int device = rover.getDeviceID( );
//		int field = rover.getFieldID(device, );
//		addTelemetry(rover.getComponents().get(device).getIdentifier(), field, arr.get(device));
		
		//device = rover.getDeviceID();
		//field = rover.getFieldID(device, );
		return arr;
	}
	
	public int getNumComponents() {
		return rover.getComponents().size();
	}
	public int getNumFields(int did) {
		return rover.getComponents().get(rover.getDeviceID(did)).getComponents().size();
	}
	
	public void addTimeTelemetry(long[] time) {
		this.time = time;
	}
	public boolean addTelemetry(int did, int fid, double[] arr) {
		int device = -1, field = -1;
		device = rover.getDeviceID(did);
		field = rover.getFieldID(device, fid);
		//telemetry.get(did).remove(fid);
		if(device >= 0 && field >= 0) {
			telemetry.get(device).remove(field);
			telemetry.get(device).add(field, arr);
			((SerializedComponent)rover.getComponents().get(device).getComponents().get(field)).setResetTimer(DEFAULT_RESET_TIME);
			((SerializedComponent)rover.getComponents().get(device).getComponents().get(field)).setReadTimer(DEFAULT_READ_TIME);
			return true;
		} return false;
	}
	
	public class ReceiveParamFunction implements ParamFunction {
		String s,sORI;
		String[] s2 = new String[4];
		//I know that the receive string is updated before calling this function, so I can use it.
		@Override
		public void execute() {
			s = client.getReceiveString();
			sORI = s.substring(0);
			s = s.concat(".");
			//split apprently does not work, so wrote it manually.
			for(int i = 0; i < 4; i++) {
				if(s.indexOf('.') != -1) {
					s2[i] = s.substring(0, s.indexOf('.'));
					s = s.substring(s.indexOf('.')+1);
				} else break;
			}
			//PROCESS THE RECEIVED STRING HERE
			if(s2[0] != null) {
				if(s2[0].startsWith("time:")) {
					if(time != null) time[0] = Long.parseLong(s2[0].substring(5));
				}
				if(s2[0].equals("r")) {
					int device = -1, field = -1;
					device = rover.getDeviceID(Integer.parseInt(s2[1]));
					field = rover.getFieldID(device, Integer.parseInt(s2[2]));
					System.out.printf("splitter got %s\n", sORI);
					if(field >= 0 && s2[3].length() > 0) {
						rover.getComponents().get(device).getComponents().get(field).setData(s2[3]);
						((SerializedComponent)rover.getComponents().get(device).getComponents().get(field)).setTime(System.currentTimeMillis());
						if(telemetry.size() >= device) {
							if(telemetry.get(device).size() >= field) {
								if(telemetry.get(device).get(field) != null) {
									System.out.printf("splitter updates %s %s: %s\n", s2[1],s2[2],s2[3]);
									telemetry.get(device).get(field)[0] = Double.parseDouble(rover.getComponents().get(device).getComponents().get(field).getData());
								}
							}
						}
//						System.out.printf("Client get %s:r.%s.%s.%s:\n", sORI,
//							rover.getComponents().get(device).getIdentifier(),
//							rover.getComponents().get(device).getComponents().get(field).getIdentifier(),
//							rover.getComponents().get(device).getComponents().get(field).getData());
						// if(s2[1].equals("12") && s2[2].equals("11")) {
						// 	double num = Integer.parseInt(rover.getComponents().get(device).getComponents().get(field).getData());
						// 	System.out.printf("Speed1: %f\n", num);
						// } else if(s2[1].equals("12") && s2[2].equals("21")) {
						// 	double num = Integer.parseInt(rover.getComponents().get(device).getComponents().get(field).getData());
						// 	System.out.printf("Speed2: %f\n", num);
						// } else if(s2[1].equals("12") && s2[2].equals("14")) {
						// 	double voltage = Integer.parseInt(rover.getComponents().get(device).getComponents().get(field).getData())/1023.0 * 5.0 *(2.7+10)/2.7;
						// 	System.out.printf("voltage: %f\n", voltage);
						// } else if(s2[1].equals("12") && s2[2].equals("15")) {
						// 	double voltage = Integer.parseInt(rover.getComponents().get(device).getComponents().get(field).getData())/1023.0 * 5.0 *(2.7+10)/2.7;
						// 	System.out.printf("current: %f\n", voltage);
						// }
					}//field >= 0
				}//read section
			}//if format matches
		}//execute()
	}//ReceiveParamFunction class
	
	
	/**
	 * Does the sending of the 
	 */
	public class SendThread extends Thread {
		boolean isRunning;
		int counter, defaultCounter;
		
		public SendThread() {
			isRunning = true;
			defaultCounter = 1000;
			counter = defaultCounter;
		}
		
		public void run() {
			int speed = 0, dir = 1;
			while(isRunning) {
				if(client.isConnected()) {
					if(counter%1000 == 0) {//check every second
						//voltage/1023.0 * 5.0 *(2.7+10)/2.7
						Calendar c = Calendar.getInstance();
						c.setTime(new Date(System.currentTimeMillis()));
						for(int i = 0; i < rover.getComponents().size(); i++) {
							for(int j = 0; j < rover.getComponents().get(i).getComponents().size(); j++) {
								if(((SerializedComponent)rover.getComponents().get(i).getComponents().get(j)).isPastReadTime(c)) {
									if(client.sendPacket("r."+rover.getComponents().get(i).getIdentifier()+"."+rover.getComponents().get(i).getComponents().get(j).getIdentifier())) {
										setTelemetryData(i,j);
										//System.out.printf("GUI Sends %s\n", "r."+rover.getComponents().get(i).getIdentifier()+"."+rover.getComponents().get(i).getComponents().get(j).getIdentifier());
									}
								}
						}}
					}//check every second
				}
				try { Thread.sleep(1); } catch (InterruptedException ex) { }//sleep 1 ms
				if(counter <= 1) counter = defaultCounter;
				else counter--;
			}//loop
		}//run method
	}//SendThread class
	public class ResetThread extends Thread {
		boolean isRunning;
		int counter, defaultCounter;
		
		public ResetThread() {
			isRunning = true;
			defaultCounter = 1000;
			counter = defaultCounter;
		}
		
		public void run() {
			int speed = 0, dir = 1;
			while(isRunning) {
				if(counter%1000 == 0) {
					Calendar c = Calendar.getInstance();
					c.setTime(new Date(System.currentTimeMillis()));
					//get
					for(int i = 0; i < rover.getComponents().size(); i++) {
						for(int j = 0; j < rover.getComponents().get(i).getComponents().size(); j++) {
							if(((SerializedComponent)rover.getComponents().get(i).getComponents().get(j)).isPastResetTime(c)) {
								rover.getComponents().get(i).getComponents().get(j).setData("0");
								setTelemetryData(i,j);
							}
						}
					}
				}
				try { Thread.sleep(1); } catch (InterruptedException ex) { }//sleep 1 ms
				if(counter <= 1) counter = defaultCounter;
				else counter--;
			}
		}
	}//ResetThread class
	public void setTelemetryData(int i, int j) {
		if(telemetry.size() >= i) {
			if(telemetry.get(i).size() >= j) {
				if(telemetry.get(i).get(j) != null) {
					telemetry.get(i).get(j)[0] = Double.parseDouble(rover.getComponents().get(i).getComponents().get(j).getData());
				}
			}
		}
	}//setTelemetryData method
	
	public static void main(String[] a) {
		JFrame frameServer2 = new JFrame("UDP Client");
			UDPClient client2 = new UDPClient("localhost", 4000, 4000);
			UDPClientGUI udp2 = new UDPClientGUI(client2);
			frameServer2.add(udp2);
			YURTRover r = YURTRover.buildRoverModel();
			TelemetrySplitterOri tele2 = new TelemetrySplitterOri(client2, r);
			tele2.addTelemetry(21, 12, new double[1]);
			//ArrayList<double[]> arr = tele2.buildStatus();
		frameServer2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frameServer2.pack();
		frameServer2.setVisible(true);
		
		JFrame frameServer3 = new JFrame("UDP Rover");
			UDPClient client3 = new UDPClient("localhost", 4000, 4000);
			UDPClientGUI udp3 = new UDPClientGUI(client3);
			frameServer3.add(udp3);
			RoverTelemetry rover = new RoverTelemetry(client3);
		frameServer3.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frameServer3.pack();
		frameServer3.setVisible(true);
	}
}//ClientTelemetry class















//	public class SplitterParamFunction implements ParamFunction {
//		String s,sORI;
//		String[] s2 = new String[4];
//		
//		@Override
//		public void execute() {
//			//I know I call the UDPClient's paramfunction, which sets receiveString and receiveAddress. So I can use them.
//			InetAddress addr = receiver.getReceiveAddress();
//			int port = receiver.getReceivePort();
//			
//			s = client.getReceiveString();
//			sORI = s.substring(0);
//			s = s.concat(".");
//			System.out.printf("Test Send:%s \n", sORI);
//			//split apprently does not work, so wrote it manually.
//			for(int i = 0; i < 4; i++) {
//				if(s.indexOf('.') != -1) {
//					s2[i] = s.substring(0, s.indexOf('.'));
//					s = s.substring(s.indexOf('.')+1);
//				} else break;
//			}
//			//PROCESS THE RECEIVED STRING HERE
//			if(s2[0] != null) {
//				if(s2[0].startsWith("time:")) {
//					if(time != null) receiver.sendPacket("time:"+time[0], addr.getHostAddress(), port);
//				}
//				if(s2[0].equals("r")) {
//					int device = -1, field = -1;
//					device = rover.getDeviceID(Integer.parseInt(s2[1]));
//					field = rover.getFieldID(device, Integer.parseInt(s2[2]));
//					if(field >= 0) {
//						if(rover.getComponents().size() >= device) {
//							if(rover.getComponents().get(device).getComponents().size() >= field) {
//								receiver.sendPacket(sORI, addr, port);
//								System.out.printf("Test Send:%s %s %d\n", sORI+"."+rover.getComponents().get(device).getComponents().get(field).getData(), addr.getHostAddress(), port);
//							}
//						}
//					}//field >= 0
//				}//read section
//			}//if format matches
//		}//execute()
//	}//ReceiveParamFunction class
