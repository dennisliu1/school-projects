package comm.Test;

import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.*;

import comm.ParamFunction;
import comm.TCP.TCPClient;

/**
 *
 * @version 1.0
 * @author Dennis Liu, YURT 2012
 */
public class TCPClientGUI extends JPanel {
	TCPClient client;
	
	JPanel[] panels;
	JLabel portLabel;
	JTextArea IPTextArea, portTextArea;
	JButton connectButton;
	ConnectActionListener CAL;
	JLabel connectionLabel;
	JTextArea receiveTextArea;
	JTextArea sendTextArea;
	JButton sendButton;
	SendActionListener SAL;
	
	ConnectionThread CThread;
	
	public TCPClientGUI(TCPClient client) {
		super();
		this.client = client;
		
		panels = new JPanel[3];
		for(int i = 0; i < panels.length; i++) panels[i] = new JPanel();
		//panel 1: connections
		portLabel = new JLabel("IP/port");
		IPTextArea = new JTextArea(1, 5);
		IPTextArea.setText(client.getIP());
		portTextArea = new JTextArea(1, 5);
		portTextArea.setText(client.getPort()+"");
		connectButton = new JButton("connect");
		CAL = new ConnectActionListener(IPTextArea, portTextArea, connectButton);
		connectButton.addActionListener(CAL);
		connectionLabel = new JLabel("disconnected");
		//panel 2: receive strings
		receiveTextArea = new JTextArea(1, 20);
		//panel 3: send strings
		sendTextArea = new JTextArea(1, 20);
		sendButton = new JButton("send");
		SAL = new SendActionListener(sendTextArea, sendButton);
		sendButton.addActionListener(SAL);
		//connect together
		panels[0].add(portLabel);
		panels[0].add(IPTextArea);
		panels[0].add(portTextArea);
		panels[0].add(connectButton);
		panels[0].add(connectionLabel);
		panels[1].add(receiveTextArea);
		panels[2].add(sendTextArea);
		panels[2].add(sendButton);
								
		this.setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
		for(int i = 0; i < panels.length; i++) this.add(panels[i]);
		
		client.addReceiveFunct(new ReceiveFunct());
		client.startThread();
		
		CThread = new ConnectionThread();
		CThread.start();
		//X:342 Y:128
		this.setMaximumSize(new Dimension(342,128));
	}
	//--------------------		Supprting Classes		------------------------//
	public class ConnectActionListener implements ActionListener {
		JTextArea IPtext;
		JTextArea Porttext;
		JButton button;
		public ConnectActionListener(JTextArea IPtext, JTextArea Porttext, JButton button) {
			this.IPtext = IPtext;
			this.Porttext = Porttext;
			this.button = button;
		}
		@Override
		public void actionPerformed(ActionEvent arg0) {
			if(button.getText().equals("connect")) {
				if(!(IPtext.getText().equals("") || Porttext.getText().equals(""))) 
					client.changeClientSettings(IPtext.getText(), Integer.parseInt(Porttext.getText()));
				if(!client.isConnected()) 
					if(client.startConnection()) button.setText("disconnect");
			} else if(client.isConnected()) {
				if(client.stopConnection())
					button.setText("connect");
			}
		}
	}//SendActionListener class
	
	public class SendActionListener implements ActionListener {
		JTextArea sendTextArea;
		JButton button;
		public SendActionListener(JTextArea sendTextArea, JButton button) {
			this.sendTextArea = sendTextArea;
			this.button = button;
		}
		@Override
		public void actionPerformed(ActionEvent arg0) {
			String s = sendTextArea.getText();
			if(!s.equals("") && client.isConnected()) client.sendString(s);
		}
	}//SendActionListener class
	
	public class ConnectionThread extends Thread {
		public boolean isRunning = true, pause = false;
		public ConnectionThread() {
		}

		public void run() {
			while(isRunning) {
				while(pause) { try{ Thread.sleep(1); }catch(InterruptedException e1){} }
				if(!client.getConnectionStatus().equals(connectionLabel.getText())) {
					connectionLabel.setText(client.getConnectionStatus());
				}
				try{ Thread.sleep(1); }catch(InterruptedException e1){}
				//System.out.println(isConnected);
			}
		}
	}//ReceiveThread class
	
	public class ReceiveFunct implements ParamFunction {
		@Override
		public void execute() {
			receiveTextArea.setText(client.receiveString());
		}
	}
	//----------------------		Main		--------------------------------//
	public static void main(String[] a) {
//		JFrame frameServer = new JFrame("TCP GUI");
//			TCPServerGUI server = new TCPServerGUI(frameServer);
//			server.addConnection(6000);
//		frameServer.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//		frameServer.add(server);
//		frameServer.setSize(800, 500);
//		frameServer.setVisible(true);
		
		TCPClientGUI gui = new TCPClientGUI(new TCPClient("localhost", 6000));
		JFrame frame = new JFrame("TCP GUI");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.add(gui);
		frame.pack();
		frame.setVisible(true);
		System.out.printf("X:%d Y:%d\n", frame.getSize().width, frame.getSize().height);
	}
}
