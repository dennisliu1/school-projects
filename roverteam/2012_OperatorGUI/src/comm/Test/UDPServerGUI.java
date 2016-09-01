/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Comm.Test;

import Comm.UDP.UDPClient;
import Comm.UDP.UDPServer;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;

/**
 *
 * @author Admin
 */
public class UDPServerGUI extends JPanel {
	public ArrayList<UDPClientGUI> clientPanels;
	public UDPServer server;
	private int counter;
	
	JFrame frame;
	public JPanel panel1;
	JLabel portLabel;
	JTextArea IPTextArea;
	JTextArea receivePortTextArea;
	JTextArea sendPortTextArea;
	JButton makeButton;
	CreateConnectionActionListener CCAL;
	JPanel panel2;
	JScrollPane scrollPane;
	JPanel self = this;
	
	public UDPServerGUI(UDPServer server) {
		super();
		this.setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
		clientPanels = new ArrayList<UDPClientGUI>();
		//server = new UDPServer();
		this.server = server;
		counter = 1;
		
		panel1 = new JPanel();
		portLabel = new JLabel("port");
		IPTextArea = new JTextArea(1, 10);
		IPTextArea.setText("localhost");
		receivePortTextArea = new JTextArea(1, 5);
		receivePortTextArea.setText("4006");
		sendPortTextArea = new JTextArea(1, 5);
		sendPortTextArea.setText("4005");
		makeButton = new JButton("make socket");
		CCAL = new CreateConnectionActionListener(IPTextArea, receivePortTextArea,sendPortTextArea , makeButton);
		makeButton.addActionListener(CCAL);
		panel1.add(portLabel);
		panel1.add(IPTextArea);
		panel1.add(receivePortTextArea);
		panel1.add(sendPortTextArea);
		panel1.add(makeButton);
		panel2 = new JPanel();
		panel2.setLayout(new BoxLayout(panel2, BoxLayout.Y_AXIS));
		panel2.setPreferredSize(new Dimension(500, 500));
		scrollPane = new JScrollPane(panel2);
		
		
		this.add(panel1);
		this.add(scrollPane);
		
//		CThread = new ConnectionThread(tcp, connectionLabel);
//		CThread.start();
//		RThread = new ReceiveThread(tcp, receiveTextArea);
//		RThread.start();
	}
	
	public void addConnection(String IP, int receivePort, int sendPort) {
		addConnection(counter, IP, receivePort, sendPort);
		counter++;
	}
	public void addConnection(int num, String IP, int receivePort, int sendPort) {
//		if(server.get(IP, receivePort, sendPort) == null) System.out.println("null");
//		else System.out.println(server.get(IP, receivePort, sendPort).toString());
		if(server.get(IP, receivePort, sendPort) == null) {
			server.allocateSocket(IP, receivePort, sendPort);
		} if(server.size() != clientPanels.size()) {
			clientPanels.add(new UDPClientGUI(server.get(IP, receivePort, sendPort)));
			panel2.add(clientPanels.get(clientPanels.size()-1));
		}
	}
	//--------------------------		Classes		----------------------------//
	public class CreateConnectionActionListener implements ActionListener {
		JTextArea IPTextArea;
		JTextArea receivePortTextArea;
		JTextArea sendPortTextArea;
		JButton button;
		String IP;
		int receivePort, sendPort;
		public CreateConnectionActionListener(JTextArea IPTextArea, JTextArea receivePortTextArea, JTextArea sendPortTextArea, JButton button) {
			this.IPTextArea = IPTextArea;
			this.receivePortTextArea = receivePortTextArea;
			this.sendPortTextArea = sendPortTextArea;
			this.button = button;
		}
		@Override
		public void actionPerformed(ActionEvent arg0) {
			try {
				receivePort = Integer.parseInt(receivePortTextArea.getText());
				sendPort = Integer.parseInt(sendPortTextArea.getText());
				IP = IPTextArea.getText();
				addConnection(IP, receivePort, sendPort);
				self.revalidate();
			} catch(Exception e) {}
		}
	}//SendActionListener class
	
	public class ConnectionThread extends Thread {
		UDPClient alloc;
		JLabel text;
		boolean isRunning = true, pause = false, wasConnected = false;
		public ConnectionThread(UDPClient alloc, JLabel text) {
			this.alloc = alloc;
			this.text = text;
		}
		
		public void run() {
			while(isRunning) {
				while(pause) { try{ Thread.sleep(1); }catch(InterruptedException e1){} }
				if(alloc.isConnected() != wasConnected) {
					if(alloc.isConnected() && !wasConnected) text.setText("connected");
					else if(!alloc.isConnected() && wasConnected) text.setText("disconnected");
					wasConnected = alloc.isConnected();
				}
				try{ Thread.sleep(1); }catch(InterruptedException e1){}
				//System.out.println(isConnected);
			}
		}
	}//ReceiveThread class
	
	public class ReceiveThread extends Thread {
		UDPClient alloc;
		JTextArea text;
		String temp, s;
		boolean isRunning = true, pause = false;
		public ReceiveThread(UDPClient alloc, JTextArea text, String s) {
			this.alloc = alloc;
			this.text = text;
			this.s = s;
		}
		
		public void run() {
			while(isRunning) {
				while(pause) { try{ Thread.sleep(1); }catch(InterruptedException e1){} }
				if(alloc.isConnected()) {
					temp = alloc.getReceiveString();
					if(!temp.equals("")) {
						s = temp;
						text.setText(s);
					}
				}
				try{ Thread.sleep(1); }catch(InterruptedException e1){}
				//System.out.println(isConnected);
			}
		}
	}//ReceiveThread class
	//---------------------------		Main		----------------------------//
	public static void main(String[] a) {
		JFrame frameServer = new JFrame("TCP GUI");
			UDPServerGUI server = new UDPServerGUI(new UDPServer());
			server.addConnection("localhost", 4001, 4000);
			//server.addConnection("192.168.0.147", 4001, 4000);
		frameServer.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frameServer.add(server);
		frameServer.setSize(800, 500);
		frameServer.setVisible(true);
	}
}
