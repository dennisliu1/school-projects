package JoystickDriveControl;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

import javax.swing.JTextArea;

public class ReceivePackets implements Runnable {

	//private DatagramSocket base;
	//private int port;
	//private InetAddress address = null;
	public DatagramSocket sock = null;
	private DatagramPacket speed_packet = null;
	
	//private byte[] GET_SPEED = new byte[4];
	private final byte GET_SPEED_COMMAND = 0x61;
	private final byte GET_CURRENT_COMMAND = 0x64;
	private byte parent_id = 1;
	private byte motor_id_1 = 1;
	private byte motor_id_2 = 2;
	private byte motor_id_3 = 3;
	private byte motor_id_4 = 4;
	private byte motor_1 = (byte) ((parent_id << 4) | motor_id_1);
	private byte motor_2 = (byte) ((parent_id << 4) | motor_id_2);
	private byte motor_3 = (byte) ((parent_id << 4) | motor_id_3);
	private byte motor_4 = (byte) ((parent_id << 4) | motor_id_4);
	public int motor_speed1;
	public int motor_speed2;
	public int motor_speed3;
	public int motor_speed4;
	public int motor_current1;
	public int motor_current2;
	public int motor_current3;
	public int motor_current4;
	public Thread receiveThread;
	//JTextArea textMotor1;
	//JTextArea textMotor2;
	//JTextArea textMotor3;
	//JTextArea textMotor4;
	JTextArea[] textMotors;
	JTextArea[] textCurrent;
	int numMotors;
	private int interval;
	
	//private boolean run_bool = false;
	@Override
	public void run() {
		
		//run_bool = true;
		while (true)
		{
			//byte parent_id = 1;
			
			//Get speed command byte array for motor 1
			//byte device_id = 1;
			/*this.GET_SPEED[0] = (byte) ((parent_id << 4) | device_id);
			this.GET_SPEED[1] = this.GET_SPEED_COMMAND;
			this.GET_SPEED[2] = 0;
			this.GET_SPEED[3] = 0;
			//this.GET_SPEED[4] = 0;
			
			this.write(this.GET_SPEED);*/
			try {
				Thread.sleep(this.interval);
			} catch (InterruptedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
			try {
				this.sock.receive(this.speed_packet);
			} catch (IOException e) {
				// 	TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			byte[] receivedPackets = this.speed_packet.getData();
			
			if (receivedPackets[1] == this.GET_SPEED_COMMAND)
			{
				if (receivedPackets[0] == this.motor_1)
				{
					this.motor_speed1 = (int) ((receivedPackets[2] << 4) | receivedPackets[3]);
				}else if (receivedPackets[0] == this.motor_2)
				{
					this.motor_speed2 = (int) ((receivedPackets[2] << 4) | receivedPackets[3]);
				}else if (receivedPackets[0] == this.motor_3)
				{
					this.motor_speed3 = (int) ((receivedPackets[2] << 4) | receivedPackets[3]);
				}else if (receivedPackets[0] == this.motor_4)
				{
					this.motor_speed4 = (int) ((receivedPackets[2] << 4) | receivedPackets[3]);
				}
			}else if (receivedPackets[1] == this.GET_CURRENT_COMMAND)
			{
				if (receivedPackets[0] == this.motor_1)
				{
					this.motor_current1 = (int) ((receivedPackets[2] << 4) | receivedPackets[3]);
				}else if (receivedPackets[0] == this.motor_2)
				{
					this.motor_current2 = (int) ((receivedPackets[2] << 4) | receivedPackets[3]);
				}else if (receivedPackets[0] == this.motor_3)
				{
					this.motor_current3 = (int) ((receivedPackets[2] << 4) | receivedPackets[3]);
				}else if (receivedPackets[0] == this.motor_4)
				{
					this.motor_current4 = (int) ((receivedPackets[2] << 4) | receivedPackets[3]);
				}
			}
			//Setting the text of the text areas
			
			this.textMotors[0].setText(this.motor_speed1 + " ");
			this.textMotors[1].setText(this.motor_speed2 + " ");
			this.textMotors[2].setText(this.motor_speed3 + " ");
			this.textMotors[3].setText(this.motor_speed4 + " ");
			
			this.textCurrent[0].setText(this.motor_current1 + " ");
			this.textCurrent[1].setText(this.motor_current2 + " ");
			this.textCurrent[2].setText(this.motor_current3 + " ");
			this.textCurrent[3].setText(this.motor_current4 + " ");
			
			
			
			
			//Garbage from here
			/*this.motor1 = this.speed_packet.getData();
			int length = this.speed_packet.getLength();
			
			//this.textMotor1.insert(this.motor1[0], arg1)
			int min = length;
			if (this.numMotors <= length)
				min = this.numMotors;
			
			for (int i = 0; i < min; i++)
			{
				this.textMotors[i].insert(this.motor1[i] + " ", 0);
			}*/
			//ALL THIS DOWN HERE IN CASE I HAVE TO REQUEST EACH MOTOR SPEED ONE BY ONE
			//RIGHT NOW ASSUMING BYTE ARRAY CONTAINS ALL MOTOR SPEEDS
			/*
			//Get speed command byte array for motor 2
			device_id = 2;
			this.GET_SPEED[0] = (byte) ((parent_id << 4) | device_id);
			this.GET_SPEED[1] = this.GET_SPEED_COMMAND;
			this.GET_SPEED[2] = 0;
			this.GET_SPEED[3] = 0;
			this.GET_SPEED[4] = 0;
		
			//this.write(this.GET_SPEED);
			try {
				this.sock.receive(speed_packet);
			} catch (IOException e) {
				// 	TODO Auto-generated catch block
				e.printStackTrace();
			}
			this.motor2 = speed_packet.getData();
			
			//Get speed command byte array for motor 3
			//device_id = 3;
			//this.GET_SPEED[0] = (byte) ((parent_id << 4) | device_id);
			this.GET_SPEED[1] = this.GET_SPEED_COMMAND;
			this.GET_SPEED[2] = 0;
			this.GET_SPEED[3] = 0;
			this.GET_SPEED[4] = 0;
		
			//this.write(this.GET_SPEED);
			try {
				this.sock.receive(speed_packet);
			} catch (IOException e) {
				// 	TODO Auto-generated catch block
				e.printStackTrace();
			}
			this.motor3 = speed_packet.getData();
			
			//Get speed command byte array for motor 4
			//device_id = 4;
			//this.GET_SPEED[0] = (byte) ((parent_id << 4) | device_id);
			this.GET_SPEED[1] = this.GET_SPEED_COMMAND;
			this.GET_SPEED[2] = 0;
			this.GET_SPEED[3] = 0;
			this.GET_SPEED[4] = 0;
		
			//this.write(this.GET_SPEED);
			try {
				this.sock.receive(speed_packet);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			this.motor4 = speed_packet.getData();*/
		}
	}

	/**
	 * @param args
	 */
	/*public static void main(String[] args) {
		// TODO Auto-generated method stub
		try {
			sock = new DatagramSocket();
		} catch (SocketException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		this.run();
		
	}*/
	
	public void start() {
		this.receiveThread = new Thread(this);
		this.receiveThread.start();
	}
	public void stop() {
		this.receiveThread.stop();
	}
	
	public ReceivePackets(JTextArea[] motors, int num_Motors, JTextArea[] current, DatagramSocket sock)
	{
		//this.sock = Socket;
		/*try {
			this.address = InetAddress.getByName(ip);
		} catch (UnknownHostException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}*/
		//this.port = sendPort;
		this.speed_packet = new DatagramPacket(new byte[4], 4);
		this.numMotors = num_Motors;
		this.textMotors = motors;
		this.receiveThread = new Thread(this);
		this.textCurrent = current;
		this.sock = sock;
		/*this.textMotor1 = motor1;
		this.textMotor2 = motor2;
		this.textMotor3 = motor3;
		this.textMotor4 = motor4;*/
		
	}

	/*public void write(byte[] data)
	{
		try {
				this.sock.send(new DatagramPacket(data,4, this.address, this.port));
					//System.err.println(data[0] + " " + (byte)(char)data[1] + " " + data[2] + " " + data[3]);
		} catch (IOException e) {
					// TODO Auto-generated catch block
			e.printStackTrace();
		}
				
	}*/
	public void setPollInterval(int pollMillis) 
	{
	      this.interval = pollMillis;
	      //joy.setPollInterval(pollMillis);
	}
	/*public void start() 
	{
		//this.run();
		this.receiveThread.start();
	}
	
	public void stop()
	{
		//this.run_bool = false;
		this.receiveThread.stop();
		//if (this.sock != null)
			//t//his.sock.close()
	}*/
}
