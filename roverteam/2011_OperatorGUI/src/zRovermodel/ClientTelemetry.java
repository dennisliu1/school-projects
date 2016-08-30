/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package zRovermodel;

import gui.OperatorGUI;
import gui.component.GUIComponent;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JFrame;

import comm.ParamFunction;
import comm.Test.UDPClientGUI;
import comm.UDP.UDPClient;

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
public class ClientTelemetry {
	public static int NUM_READS = 5;
	public static final int DEFAULT_RESET_TIME = 9000;
	public static final int DEFAULT_READ_TIME = 3000;
	private UDPClient client;
	private ReceiveParamFunction receiveFunct;
	private SendThread sendThread;
	private ResetThread resetThread;
	private TimeThread timeThread;
	public YURTRover rover;
	private ArrayList<ArrayList<double[]>> telemetry;
	public long time[];
	public String signal[];
	public boolean sendTime = true;
	private ArrayList<ArrayList<GUIComponent>> components;
	private GUIComponent signalComponent, timeComponent,tableComponent,tableComponentTwo;
	private OperatorGUI gui;
	
	public ClientTelemetry(UDPClient client, YURTRover rover) {
		this.client = client;
		this.rover = rover;
		receiveFunct = new ReceiveParamFunction();
		client.addReceiveFunct(receiveFunct);
		telemetry = new ArrayList<ArrayList<double[]>>();//build telemetry list
		components = new ArrayList<ArrayList<GUIComponent>>();
		for(int i = 0; i < rover.getComponents().size(); i++) {
			telemetry.add(new ArrayList<double[]>());
			components.add(new ArrayList<GUIComponent>());
			for(int j = 0; j < rover.getComponents().get(i).getComponents().size(); j++) {
				telemetry.get(i).add(null);//add placeholder
				components.get(i).add(null);
				((SerializedComponent)rover.getComponents().get(i).getComponents().get(j)).setTime(System.currentTimeMillis());
			}
		}
		
		sendThread = new SendThread();
		sendThread.start();
		timeThread = new TimeThread();
		timeThread.start();
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
		for(int i = 0; i < getNumComponents()*2; i++) {
			arr.add(new double[1]);
		}
//		for(int i = 0; i < rover.getComponents().size(); i++) {
//			addTelemetry(rover.getComponents().get(i).getIdentifier(), 15, arr.get(i));
//			addTelemetry(rover.getComponents().get(i).getIdentifier(), 25, arr.get(i));
//		}
		return arr;
	}
	public ArrayList<double[]> buildCurrents() {
		ArrayList<double[]> arr = new ArrayList<double[]>();
		for(int i = 0; i < getNumComponents()*2; i++) {
			arr.add(new double[1]);
		}
//		for(int i = 0; i < rover.getComponents().size(); i++) {
//			addTelemetry(rover.getComponents().get(i).getIdentifier(), 16, arr.get(i));
//			addTelemetry(rover.getComponents().get(i).getIdentifier(), 26, arr.get(i));
//		}
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
	public void addSignalTelemetry(String[] signal) {
		this.signal = signal;
	}
	public void setSignalComponent(GUIComponent signal) {
		this.signalComponent = signal;
	}
	public void setTimerComponent(GUIComponent timer) {
		this.timeComponent = timer;
	}
	public void setTableComponent(GUIComponent timer) {
		this.tableComponent = timer;
	}
	public void setTableTwoComponent(GUIComponent timer) {
		this.tableComponentTwo = timer;
	}
	public void setOperatorGUI(OperatorGUI gui) {
		this.gui = gui;
	}
	public boolean addTelemetry(int did, int fid, double[] arr) {
		int device = -1, field = -1;
		device = rover.getDeviceID(did);
		field = rover.getFieldID(device, fid);
		//telemetry.get(did).remove(fid);
		if(device >= 0 && field >= 0) {
			telemetry.get(device).remove(field);
			telemetry.get(device).add(field, arr);
			return true;
		} return false;
	}
	public boolean setTelemetryResetTime(int did, int fid, int resetTime) {
		int device = -1, field = -1;
		device = rover.getDeviceID(did);
		if(device >= 0) field = rover.getFieldID(device, fid);
		//telemetry.get(did).remove(fid);
		if(device >= 0 && field >= 0) {
			((SerializedComponent)rover.getComponents().get(device).getComponents().get(field)).setResetTimer(resetTime);
			return true;
		} return false;
	}
	public boolean setTelemetryReadTime(int did, int fid, int readTime) {
		int device = -1, field = -1;
		device = rover.getDeviceID(did);
		if(device >= 0) field = rover.getFieldID(device, fid);
		//telemetry.get(did).remove(fid);
		if(device >= 0 && field >= 0) {
			((SerializedComponent)rover.getComponents().get(device).getComponents().get(field)).setReadTimer(readTime);
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
			System.out.printf("Client get %s\n", sORI);
			
			if(sORI.startsWith("time=")) {
				if(time != null) try {
					time[0] = Long.parseLong(sORI.split("=")[1]);
					if(timeComponent != null)
						timeComponent.updateDisplay();
				} catch(Exception e) {}
			} else if(sORI.startsWith("Quality=")) {
//				System.out.printf("q:%s\n", sORI.split("=")[1]);
				if(signal != null) try {
					signal[0] = sORI.split("=")[1];
					if(signalComponent != null)
						signalComponent.updateDisplay();
				} catch(Exception e) {}
			} else if(sORI.startsWith("GUI=Video.left")) {
				if(gui != null) {
					gui.moveLeft();
				}
			} else if(sORI.startsWith("GUI=Video.right")) {
				if(gui != null) {
					gui.moveRight();
				}
			} else if(sORI.startsWith("GUI=Video.start")) {
				if(gui != null) {
					gui.startVideo();
				}
			} else if(sORI.startsWith("GUI=Video.stop")) {
				if(gui != null) {
					gui.stopVideo();
				}
			} else {
				//split apprently does not work, so wrote it manually.
				s = s.concat(".");
				s2[0] = s.substring(0,1);
				s = s.substring(1);
				for(int i = 1; i < 4; i++) {
					if(s.indexOf('.') != -1) {
						s2[i] = s.substring(0, s.indexOf('.'));
						s = s.substring(s.indexOf('.')+1);
					} else break;
				}
				//PROCESS THE RECEIVED STRING HERE
				if(s2[0] != null) {
					if(s2[0].equals("r")) {
						//System.out.printf("Client get %s:%s:%s:%s:\n", sORI,s2[0],s2[1],s2[2]);
						int device = -1, field = -1;
						try {
							device = rover.getDeviceID(Integer.parseInt(s2[1]));
						} catch(Exception e){ device = -1; System.out.printf("Bad message device:%s:", sORI); }
						try {
							field = rover.getFieldID(device, Integer.parseInt(s2[2]));
							
						} catch (Exception e) { field = -1; System.out.printf(" Bad message field:%s:", sORI); }
						//System.out.printf("Client get %s\n", sORI);
						if(s2[3] != null)
						if(device >= 0 && field >= 0 && s2[3].length() > 0) {
							rover.getComponents().get(device).getComponents().get(field).setData(s2[3]);
							((SerializedComponent)rover.getComponents().get(device).getComponents().get(field)).setTime(System.currentTimeMillis());
							setTelemetryData(device,field);
							if(tableComponent != null)
								tableComponent.updateDisplay();
							if(tableComponentTwo != null)
								tableComponentTwo.updateDisplay();
						}//field >= 0
					}//read section
				}//if format matches
				for(int i = 0; i < s2.length; i++)
					s2[i] = "";
			}
		}//execute()
	}//ReceiveParamFunction class
	
	
	/**
	 * Does the sending of the requests for telemetry updates. 
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
			Calendar c = Calendar.getInstance();
			while(isRunning) {
				if(client.threadStarted) {
					//voltage/1023.0 * 5.0 *(2.7+10)/2.7
					c.setTime(new Date(System.currentTimeMillis()));
					for(int i = 0; i < rover.getComponents().size(); i++) {
						for(int j = 0; j < rover.getComponents().get(i).getComponents().size(); j++) {
							if(((SerializedComponent)rover.getComponents().get(i).getComponents().get(j)).isPastReadTime(c)) {
								if(client.sendPacket("r"+rover.getComponents().get(i).getIdentifier()+"."+rover.getComponents().get(i).getComponents().get(j).getIdentifier())) {
									System.out.printf("GUI Sends %s\n", "r"+rover.getComponents().get(i).getIdentifier()+"."+rover.getComponents().get(i).getComponents().get(j).getIdentifier());
								}
								try { 
									Thread.sleep(1000/NUM_READS);
									if(counter < 1000/NUM_READS) counter = defaultCounter;
									else counter -= 1000/NUM_READS;
								} catch (InterruptedException ex) { }//sleep 1 ms
								c.setTime(new Date(System.currentTimeMillis()));
							}
					}}
				}
				try { Thread.sleep(1); } catch (InterruptedException ex) { }//sleep 1 ms
				if(counter <= 1) counter = defaultCounter;
				else counter--;
			}//loop
		}//run method
	}//SendThread class
	public class TimeThread extends Thread {
		boolean isRunning;
		int counter, defaultCounter;
		
		public TimeThread() {
			isRunning = true;
			defaultCounter = 1000;
			counter = defaultCounter;
		}
		
		public void run() {
			int speed = 0, dir = 1;
			Calendar c = Calendar.getInstance();
			while(isRunning) {
				if(client.threadStarted) {
					if(counter == 0 && time != null && sendTime) {
						client.sendPacket("time=");
						client.sendPacket("Quality=");
						System.out.printf("GUI Sends %s\n", "time=");
					}
				}
				try { Thread.sleep(1); } catch (InterruptedException ex) { }//sleep 1 ms
				if(counter < 0) counter = defaultCounter;
				else counter--;
			}//loop
		}//run method
	}//timeThread class
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
								rover.getComponents().get(i).getComponents().get(j).setData("-1");
								setTelemetryData(i,j);
								if(tableComponent != null)
									tableComponent.updateDisplay();
								if(tableComponentTwo != null)
									tableComponentTwo.updateDisplay();
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
			UDPClient client2 = new UDPClient("localhost", 4000, 4001);
			UDPClientGUI udp2 = new UDPClientGUI(client2);
			frameServer2.add(udp2);
			YURTRover r = YURTRover.buildRoverModel();
			ClientTelemetry tele2 = new ClientTelemetry(client2, r);
			tele2.buildStatus();
		frameServer2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frameServer2.pack();
		frameServer2.setVisible(true);
		
		JFrame frameServer3 = new JFrame("UDP Rover");
			UDPClient client3 = new UDPClient("localhost", 4001, 4000);
			UDPClientGUI udp3 = new UDPClientGUI(client3);
			frameServer3.add(udp3);
			RoverTelemetry rover = new RoverTelemetry(client3);
		frameServer3.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frameServer3.pack();
		frameServer3.setVisible(true);
	}
}//ClientTelemetry class