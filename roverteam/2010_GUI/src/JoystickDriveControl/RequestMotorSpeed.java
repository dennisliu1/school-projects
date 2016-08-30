package JoystickDriveControl;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

public class RequestMotorSpeed implements Runnable {

	private int port;
	
	private InetAddress address = null;
	public DatagramSocket sock = null;
	//private DatagramPacket speed_packet = null;
	public Thread requestThread;
	private byte[] GET_SPEED = new byte[4];
	private final byte GET_SPEED_COMMAND = 0x61;
	//private boolean run_bool;
	private final byte GET_CURRENT_COMMAND = 0x64;
	private byte[] GET_CURRENT = new byte[4];
	private int interval = 200;
	
	/*public static void main(String[] args)
	{
		RequestMotorSpeed rc = new RequestMotorSpeed("192.168.3.2", 30001);
		
		//requestThread.start();
		//requestThread.stop();
		
		//rc.run();
		//rc.stop();
	}*/
	
	@Override
	public void run() {
		
		//this.run_bool = true;
		while (true)
		{
			//System.out.println("ehy");
			byte parent_id = 1;
			
			try {
				Thread.sleep(this.interval);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			//Get speed command byte array for motor 1
			byte device_id = 1;
			this.GET_SPEED[0] = (byte) ((parent_id << 4) | device_id);
			this.GET_SPEED[1] = this.GET_SPEED_COMMAND;
			this.GET_SPEED[2] = 0;
			this.GET_SPEED[3] = 0;
			//this.GET_SPEED[4] = 0;
			
			this.write(this.GET_SPEED);
			
			device_id = 2;
			this.GET_SPEED[0] = (byte) ((parent_id << 4) | device_id);
			this.write(this.GET_SPEED);
			
			device_id = 3;
			this.GET_SPEED[0] = (byte) ((parent_id << 4) | device_id);
			this.write(this.GET_SPEED);

			device_id = 4;
			this.GET_SPEED[0] = (byte) ((parent_id << 4) | device_id);
			this.write(this.GET_SPEED);
			
			device_id = 1;
			this.GET_CURRENT[0] = (byte) ((parent_id << 4) | device_id);
			this.GET_CURRENT[1] = this.GET_CURRENT_COMMAND;
			this.GET_CURRENT[2] = 0;
			this.GET_CURRENT[3] = 0;
			//this.GET_SPEED[4] = 0;
			
			this.write(this.GET_CURRENT);
			
			device_id = 2;
			this.GET_CURRENT[0] = (byte) ((parent_id << 4) | device_id);
			this.write(this.GET_CURRENT);
			
			device_id = 3;
			this.GET_CURRENT[0] = (byte) ((parent_id << 4) | device_id);
			this.write(this.GET_CURRENT);

			device_id = 4;
			this.GET_CURRENT[0] = (byte) ((parent_id << 4) | device_id);
			this.write(this.GET_CURRENT);
		}
	}
	
	public RequestMotorSpeed(String ip, int port, DatagramSocket sock)
	{
		try {
			this.address = InetAddress.getByName(ip);
		} catch (UnknownHostException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		this.requestThread = new Thread(this);
		this.port = port;
		this.sock = sock;
	}
	
	public void start() {
		this.requestThread = new Thread(this);
		this.requestThread.start();
	}
	public void stop() {
		this.requestThread.stop();
	}
	
	
	public void write(byte[] data)
	{
		try {
				this.sock.send(new DatagramPacket(data,4, this.address, this.port));
					//System.err.println(data[0] + " " + (byte)(char)data[1] + " " + data[2] + " " + data[3]);
		} catch (IOException e) {
					// TODO Auto-generated catch block
			e.printStackTrace();
		}
				
	}
	/*
	public void start()
	{
		this.requestThread.start();
	}
	public void stop()
	{
		this.requestThread.stop();
	}*/

}
