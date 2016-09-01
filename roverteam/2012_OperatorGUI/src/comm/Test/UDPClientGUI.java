/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Comm.Test;

import Comm.UDP.UDPClient;
import Comm.*;
import Comm.UDP.Conn.UDPConnection;
import Comm.UDP.UDPServer;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Arrays;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextArea;

/**
 *
 * @author Admin
 */
public class UDPClientGUI extends JPanel {
	public static final byte[] KEEP_ALIVE_PACKET = new byte[]{127,127,127,2};
	
	UDPClient shared;

	JPanel self = this;
	JPanel[] panels;
	JLabel ipportLabel;
	JTextArea IPTextArea, receivePortTextArea, SendIPTextArea, sendPortTextArea;
	JButton connectButton;
	JTextArea statusTextArea;
	JTextArea receiveTextArea, echoTextArea;
	JLabel sendLabel;
	JTextArea sendStringTextArea;
	JTextArea[] sendTextAreas;
	JButton sendButton;
	ConnectActionListener CAL;
	SendActionListener SAL;
	
	ConnectionStatusThread CST;
	ProcessReceivePackets processPackets;
	
	public UDPClientGUI() {
		this("", 0, 0);
	}
	public UDPClientGUI(String IP, int receivePort, int sendPort) {
		this(new UDPClient(IP, receivePort, sendPort));
	}
	public UDPClientGUI(UDPClient shared) {
		//X:471 Y:128
		super();
		this.shared = shared;
		
		panels = new JPanel[3];
		for(int i = 0; i < panels.length; i++) panels[i] = new JPanel();
		ipportLabel = new JLabel("IP/port");
		IPTextArea = new JTextArea(1, 10);
		IPTextArea.setText(shared.getSendIP());
		receivePortTextArea = new JTextArea(1, 5);
		receivePortTextArea.setText(""+shared.getReceivePort());
		sendPortTextArea = new JTextArea(1, 5);
		sendPortTextArea.setText(""+shared.getSendPort());
		SendIPTextArea = new JTextArea(1, 10);
		//SendIPTextArea.setText(shared.getSendIP());
		connectButton = new JButton("start");
		CAL = new ConnectActionListener(shared, IPTextArea, receivePortTextArea,sendPortTextArea, connectButton);
		connectButton.addActionListener(CAL);
		statusTextArea = new JTextArea(1, 8);
		statusTextArea.setText("disconnected");
		receiveTextArea = new JTextArea(1, 10);
		echoTextArea = new JTextArea(1, 10);
		sendLabel = new JLabel("send");
		sendTextAreas = new JTextArea[3];
		for(int i = 0; i < sendTextAreas.length; i++) sendTextAreas[i] = new JTextArea(1, 3);
		sendStringTextArea = new JTextArea(1, 15);
		
		sendButton = new JButton("send");
		SAL = new SendActionListener(shared, sendTextAreas);
		sendButton.addActionListener(SAL);
		
		panels[0].add(ipportLabel);
		panels[0].add(IPTextArea);
		panels[0].add(receivePortTextArea);
		panels[0].add(sendPortTextArea);
		panels[0].add(connectButton);
		panels[1].add(statusTextArea);
		panels[1].add(receiveTextArea);
		panels[1].add(sendLabel);
//		for(int i = 0; i < sendTextAreas.length; i++) panels[2].add(sendTextAreas[i]);
		panels[1].add(sendStringTextArea);
		panels[1].add(sendButton);
		
		this.setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
		for(int i = 0; i < panels.length; i++) this.add(panels[i]);
		
		CST = new ConnectionStatusThread();
		CST.start();
		//this.setPreferredSize(new Dimension(471,128));
		this.setMaximumSize(new Dimension(471,128));
		processPackets = new ProcessReceivePackets();
		shared.addReceiveFunct(processPackets);
		
		//shared.startUDPConnection();
		//shared.startThreads();
	}
	
	private class ConnectActionListener implements ActionListener {
		JTextArea IPTextArea, receivePortTextArea, sendPortTextArea;
		JButton connectButton;
		private UDPClient shared;
		
		public ConnectActionListener(UDPClient shared, JTextArea IPTextArea,JTextArea receivePortTextArea,JTextArea sendPortTextArea, JButton connectButton) {
			this.shared = shared;
			this.IPTextArea = IPTextArea;
			this.receivePortTextArea = receivePortTextArea;
			this.sendPortTextArea = sendPortTextArea;
			this.connectButton = connectButton;
		}
		@Override
		public void actionPerformed(ActionEvent ae) {//&& shared.connectionStatus().equals(UDPClient.STATUS_DISCONNECTED)
			if(connectButton.getText().equals("start")) {
				if(!(IPTextArea.getText().equals("") && 
						receivePortTextArea.getText().equals("") && 
						sendPortTextArea.getText().equals(""))) {
					shared.setUDPSettings(IPTextArea.getText(), 
							Integer.parseInt(receivePortTextArea.getText()), 
							Integer.parseInt(sendPortTextArea.getText()));
				}
				shared.setUDPSettings(IPTextArea.getText(), 
						Integer.parseInt(receivePortTextArea.getText()), 
						Integer.parseInt(sendPortTextArea.getText()));
				shared.startUDPConnection();
				shared.startThreads();
				connectButton.setText("stop");
				System.out.printf("connect:%s\n", shared.connectionStatus());
				System.out.printf("connect: receive-%s send-%d\n", shared.getReceivePort(), shared.getSendPort());
			} else if(connectButton.getText().equals("stop")) {
				shared.stopThreads();
				shared.stopUDPConnection();
				connectButton.setText("start");
				System.out.printf("disconnect:%s\n", shared.connectionStatus());
				System.out.printf("disconnect: receive-%s send-%d\n", shared.getReceivePort(), shared.getSendPort());
			}
		}//actionPerformed method
	}//ConnectActionListener class
	private class SendActionListener implements ActionListener {
		private UDPClient shared;
		private JTextArea[] sendTextAreas;
		public SendActionListener(UDPClient shared, JTextArea[] sendTextAreas) {
			this.shared = shared;
			this.sendTextAreas = sendTextAreas;
		}
 		@Override
		public void actionPerformed(ActionEvent ae) {
			shared.sendPacket(sendStringTextArea.getText().getBytes());
		}
	}//SendActionListener class
	
	//add to UDPClient; get its received string
	public class ProcessReceivePackets implements ParamFunction {
		@Override
		public void execute() {
			receiveTextArea.setText(shared.getReceiveString());
		}
	}
	public class ConnectionStatusThread extends Thread {
			public boolean isRunning = true, pause = false;
			public ConnectionStatusThread() {
			}
			public void run() {
				while(isRunning) {
					while(pause) {
						try { Thread.sleep(1);
						} catch (InterruptedException e1) { }
					}
					if(shared.isClientStarted()) {
						if(!statusTextArea.getText().equals(shared.getConnectionStatus())) 
							statusTextArea.setText(shared.getConnectionStatus());
					}
					try { Thread.sleep(1); } catch (InterruptedException e1) { }
			}}//run, while running
		}//ConnectionThread class
	
	public static void main(String[] a) {
		JFrame frameServer = new JFrame("UDP GUI");
			//UDPClientGUI udp = new UDPClientGUI("192.168.0.147", 4001, 4000);
			UDPClientGUI udp = new UDPClientGUI("localhost", 4001, 4000);
			frameServer.add(udp);
		frameServer.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frameServer.pack();
		System.out.printf("X:%d Y:%d\n", frameServer.getSize().width, frameServer.getSize().height);
		frameServer.setVisible(true);
		
		JFrame frameServer2 = new JFrame("UDP GUI2");
			UDPClientGUI udp2 = new UDPClientGUI("localhost", 4000, 4001);
			frameServer2.add(udp2);
		frameServer2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frameServer2.pack();
		frameServer2.setVisible(true);
	}
}//UDPTestServer class
