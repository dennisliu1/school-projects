package GUI;

import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

import Comm.UDP.UDPClient;

public class TimerSendPanel extends JPanel {
	public static final String DEFAULT_START_STRING = "Start";
	public static final String DEFAULT_STOP_STRING = "Stop";
	public static final String DEFAULT_RESET_STRING = "Reset";
	public static final String DEFAULT_SEND_STRING = "Send";
	public static final String DEFAULT_NO_SEND_STRING = "No Send";
	private JPanel groups[];
	private JLabel formatLabel;
	private JTextField currentTime;
	private JTextField resetTo;
	private JButton ssButton;
	private JButton resetButton;
	private JTextField IPField,portField;
	private JButton sendButton;
	private UDPClient client;
	private long time;
	
	public TimerSendPanel(UDPClient client) {
		this.client = client;
		this.setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
		groups = new JPanel[3];
		for(int i = 0; i < groups.length; i++) {
			groups[i] = new JPanel();
			groups[i].setLayout(new FlowLayout());
			this.add(groups[i]);
		}
		formatLabel = new JLabel("HH:MM:SS");
		currentTime = new JTextField(10);
		currentTime.setEditable(false);
		resetTo = new JTextField(10);
		ssButton = new JButton(DEFAULT_START_STRING);
		resetButton = new JButton(DEFAULT_RESET_STRING);
		IPField = new JTextField(10);
		portField = new JTextField(5);
		sendButton = new JButton(DEFAULT_SEND_STRING);

		groups[0].add(formatLabel);
		groups[0].add(currentTime);
		groups[0].add(ssButton);
		groups[1].add(Box.createRigidArea(new Dimension(75,5)));
		groups[1].add(resetTo);
		groups[1].add(resetButton);
		groups[2].add(IPField);
		groups[2].add(portField);
		groups[2].add(sendButton);
		
		SendThread sendThread = new SendThread();
		SSActionListener ss = new SSActionListener(sendThread);
		ResetActionListener reset = new ResetActionListener();
		UDPActionListener send = new UDPActionListener();
		ssButton.addActionListener(ss);
		resetButton.addActionListener(reset);
		sendButton.addActionListener(send);
		drawTime(time);
		sendThread.start();
	}
	public void drawTime(long time) {
		if(time >= 0) {
			String s2[] = new String[3];
			time /= 1000;
			s2[0] = ""+(time/(60*60));
			time -= (time/60);
			s2[1] = ""+(time/(60));
			time -= (time/60);
			s2[2] = ""+(time/(60));
			currentTime.setText(s2[0]+":"+s2[1]+":"+s2[2]);
			System.out.printf("%s:%s:%s %d\n", s2[0],s2[1],s2[2], this.time);
		}
	}
	
	public class SSActionListener implements ActionListener {
		SendThread sendThread;
		public SSActionListener(SendThread sendThread) {
			this.sendThread = sendThread;
		}

		@Override
		public void actionPerformed(ActionEvent e) {
			if(((JButton)e.getSource()).getText().equals(DEFAULT_START_STRING)) {
				sendThread.pause = false;
				((JButton)e.getSource()).setText(DEFAULT_STOP_STRING);
			} else if(((JButton)e.getSource()).getText().equals(DEFAULT_STOP_STRING)) {
				sendThread.pause = true;
				((JButton)e.getSource()).setText(DEFAULT_START_STRING);
			}
		}
	}//SSActionListener class
	private class ResetActionListener implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			if(((JButton)e.getSource()).getText().equals(DEFAULT_RESET_STRING)) {
				String s = resetTo.getText();
				String s2[] = s.split(":");
				long t = Long.parseLong(s2[0])*3600 + Long.parseLong(s2[1])*60 + Long.parseLong(s2[2]);
				if(t >= 0) time = t*1000;
			}
		}
	}
	
	public class UDPActionListener implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			if(((JButton)e.getSource()).getText().equals(DEFAULT_SEND_STRING)) {
				client.setUDPSettings(IPField.getText(), Integer.parseInt(portField.getText()), Integer.parseInt(portField.getText()));
				client.startUDPConnection();
				client.startThreads();
				((JButton)e.getSource()).setText(DEFAULT_NO_SEND_STRING);
			} else if(((JButton)e.getSource()).getText().equals(DEFAULT_NO_SEND_STRING)) {
				client.stopThreads();
				client.stopUDPConnection();
				((JButton)e.getSource()).setText(DEFAULT_SEND_STRING);
			}
		}
		
	}//UDPActionListener class
	
	public class SendThread extends Thread {
		boolean isRunning = true, pause = true;
		public void run() {
			while(isRunning) {
				while(pause) { try{ Thread.sleep(1); }catch(InterruptedException e){} }
				if(time > 0) {
					if(client.isClientStarted()) client.sendPacket("time."+time);
					time -= 1000;
					drawTime(time);
				}
				try{ Thread.sleep(1000); }catch(InterruptedException e){}
			}
		}//run method
	}//SendThread class
	
	
	public static void main(String[] a) {
		int width = 200, height = 200;
		JFrame frame = new JFrame("Operator GUI");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			UDPClient client = new UDPClient("localhost",4000,4000);
			TimerSendPanel panel = new TimerSendPanel(client);
		frame.add(panel);
		frame.setSize(300, 150);
		//frame.pack();
		frame.setVisible(true);
		
	}
}//TimerSendPanel class
