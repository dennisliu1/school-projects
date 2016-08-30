package JoystickDriveControl;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;

import com.centralnexus.input.Joystick;

public class JoystickInterpret implements Runnable 
{
	private static final int DRIVE_ID = 0x1;
	private static final int ID_FL = 1;
	private static final int ID_FR = 2;
	private static final int ID_BL = 3;
	private static final int ID_BR = 4;
	
	private static final int FORWARD_BUTTON = 2;
	private static final int BACKWARD_BUTTON = 3;
	private static final int EMERGENCY_BUTTON = 7;
	private static final int READY_BUTTON = 29;
	
	
	public DatagramSocket base;
	private int port;
	private InetAddress address;
	
	private Thread roverthread;
	private Joystick joy;
	
	private final int MAX_SPEED = 126;
	private final int MAX_TURN = 126;
	
	private final byte FORWARD_COMMAND = 0x32;
	private final byte FORWARD_PULSE_COMMAND = 0x31;
	private final byte TURN_COMMAND = 0x35;
	private final byte COMMIT_COMMAND = (byte)0xFF;
	private final byte BRAKE_ALL_COMMAND = (byte) 0x90;
	private final byte SET_SPEED_COMMAND = (byte) 0xC1;
	private final byte READY_COMMAND = (byte) 0x0F;
	private final byte EMERGENCY_STOP_COMMAND = (byte) 0x0;
	
	private final int NORMAL_DRIVE_MODE = 1;
	private final int FRONT_WHEEL_DRIVE_MODE = 3;
	private final int FLASH_MODE = 2;
	private final int READY_MODE = 4;
	private final int EMERGENCY_STOP_MODE = 5;
	
	private int interval;
	
	//Constructor: 
	//connect to joystick and start running itself as a thread
	public JoystickInterpret(String ip, int port, DatagramSocket sock)
	{
		try {
			this.joy = Joystick.createInstance(); //connect to joystick
			this.interval = 100;
			joy.setPollInterval(interval);
			joy.setDeadZone(0);
			roverthread = new Thread(this);
			this.address = InetAddress.getByName(ip);
			this.port = port;
			this.base = sock;
		} catch (Exception e) 
		{
			e.printStackTrace();
		}
	}
	
	//Phase 1: read in joystick input and translate into 
	public void updateFieldsEx(Joystick joystick) throws Exception
    {

			boolean button[] = this.convertButton(joystick.getButtons());
			
			float x = joy.getX();
			float y = -joy.getY();
			float z = (-joy.getZ() + 1) / 2;
			int speed = 0;
			int turn = 0;
			
			byte packet[] = new byte[4];
			
			//System.out.printf("Xaxis = %f : Yaxis = %f : Zaxis = %f : Button = %d : But1 = %b : ButF = %b : ButB = %b : ButT = %b\n",x,y, z, button, but1, butF, butB, butT);
			
			float maxSpeed = z * MAX_SPEED; //Calculating the speeds from the joystick
			speed = (int)(maxSpeed * y);
			turn = (int) (MAX_TURN * x);
			
			if (button[EMERGENCY_BUTTON])
			{
				//Emergency Stop
				packet[0] = (byte)(DRIVE_ID << 4);
				packet[1] = EMERGENCY_STOP_COMMAND;
				
				write(packet);
				
			} else if (button[READY_BUTTON]){
				
				//Ready
				packet[0] = (byte) (DRIVE_ID << 4);
				packet[1] = READY_COMMAND;
				
				write(packet);
				
			}
			
			//read in button commands and do them
			if (!button[0])
			{
				if (button[23]) //Mode 1 (?)
				{
					//Mode 1 regular drive
					//System.out.println("mode1");
					if (button[FORWARD_BUTTON])
					{
						if (speed < 0)
							speed = 0;
						
						
						//Forward regular drive
						packet[0] = (byte)(DRIVE_ID << 4);
						packet[1] = FORWARD_COMMAND;
						packet[2] = (byte)(speed >> 8);
						packet[3] = (byte)(speed & 0xFF);
						
						write(packet);
						
						packet[0] = (byte)(DRIVE_ID << 4);
						packet[1] = TURN_COMMAND;
						packet[2] = (byte)(turn >> 8);
						packet[3] = (byte)(turn & 0xFF);
						
						write(packet);
						
					}else if (button[BACKWARD_BUTTON])
					{
						if (speed > 0)
							speed = 0;
						
						//Reverse regular drive
						packet[0] = (byte)(DRIVE_ID << 4);
						packet[1] = FORWARD_COMMAND;
						packet[2] = (byte)(speed >> 8);
						packet[3] = (byte)(speed & 0xFF);
						
						write(packet);
						
						packet[0] = (byte)(DRIVE_ID << 4);
						packet[1] = TURN_COMMAND;
						packet[2] = (byte)(turn >> 8);
						packet[3] = (byte)(turn & 0xFF);
						
						write(packet);
					}
					else {
						// Brake All
						packet[0] = (byte)(DRIVE_ID << 4);
						packet[1] = BRAKE_ALL_COMMAND;
						
						write(packet);
						
					}
					
				} else if (button[24]) //Mode 2
				{
						//Forward drive front wheels
						//Backward drive rear wheels
						//System.out.println("mode2");
						if (button[FORWARD_BUTTON])
						{
							
						}else if (button[BACKWARD_BUTTON])
						{
							
						}
						else {
							// Brake All
							packet[0] = (byte)(DRIVE_ID << 4);
							packet[1] = BRAKE_ALL_COMMAND;
							
							write(packet);
							
						}
				} else if (button[25]) //Mode 3
				{
					//Forward flash
					//System.out.println("flash");
					if (button[FORWARD_BUTTON])
					{
						if (speed < 0)
							speed = 0;
						
						//Forward regular drive
						packet[0] = (byte)(DRIVE_ID << 4);
						packet[1] = FORWARD_PULSE_COMMAND;
						packet[2] = (byte)(speed >> 8);
						packet[3] = (byte)(speed & 0xFF);
						
						write(packet);
						
						packet[0] = (byte)(DRIVE_ID << 4);
						packet[1] = TURN_COMMAND;
						packet[2] = (byte)(turn >> 8);
						packet[3] = (byte)(turn & 0xFF);
						
						write(packet);
						
					}else if (button[BACKWARD_BUTTON])
					{
						if (speed > 0)
							speed = 0;
						
						//Reverse regular drive
						packet[0] = (byte)(DRIVE_ID << 4);
						packet[1] = FORWARD_PULSE_COMMAND;
						packet[2] = (byte)(speed >> 8);
						packet[3] = (byte)(speed & 0xFF);
						
						write(packet);
						
						packet[0] = (byte)(DRIVE_ID << 4);
						packet[1] = TURN_COMMAND;
						packet[2] = (byte)(turn >> 8);
						packet[3] = (byte)(turn & 0xFF);
						
						write(packet);
					}
					else {
						// Brake All
						packet[0] = (byte)(DRIVE_ID << 4);
						packet[1] = BRAKE_ALL_COMMAND;
						
						write(packet);
						
					}
				}
			}
			else {
				// Brake All
				packet[0] = (byte)(DRIVE_ID << 4);
				packet[1] = BRAKE_ALL_COMMAND;
				
				write(packet);
				
			}
			
				/*{
					if (this.toggleMode == 0)
						this.toggleMode = 1;
					else
						this.toggleMode = 0;
				}*/
			
			
			//System.out.println("forwardLeft: "  + forwardLeft + " forwardRight: " + forwardRight + " x: " + x + " y: " + y + " z: " + z);
			//System.out.println("max: " + maxSpeed);
			//System.out.println("forwardLeft: "  + forwardLeft + " forwardRight: " + forwardRight + " max: " + maxSpeed);
			//System.out.println(button);
		System.out.printf("Xaxis = %f : Yaxis = %f : Zaxis = %f : Button = %s\n", x, y, z, Integer.toBinaryString(joystick.getButtons()));
			
			//System.out.println("Speed: " + speed + " turn: " + turn);
    
    }
	
	private boolean[] convertButton(int buttons)
	{
		boolean ans[] = new boolean[32];
		  
		for (int i=0; i<32; i++)
		{
			if (((buttons >> i) & 1) == 1){
				ans[i] = true;
			}else{
				ans[i] = false;
			}
			
		}
		
		return ans;
	}
	//Sets the delay between each poll for joystick input
	public void setPollInterval(int pollMillis) 
	  {
	      this.interval = pollMillis;
	      joy.setPollInterval(pollMillis);
	  }
	//run this thread
	  public void startPolling() 
	  {
		  //this.run();
		  roverthread = new Thread(this);
	      roverthread.start();
	  }
	//stop this thread
	public void stopPolling()
	  {
		//this.stop();
		roverthread.stop();
		// this.run_bool = false;
		  //this.base.close();
	  }

	@Override
	public void run() 
	{
		//this.run_bool = true;
		
		while (true) // main loop
        {
            joy.poll(); //update joystick values

            try 
            {	//*note running Phase 1 -> Phase 2 -> Phase 3 as well
				this.updateFieldsEx(joy); //Phase 1: change joystick values to rover control values
				//System.out.println("Hey");
				try 
	            {
	                Thread.sleep(interval); //sleep thread, no need to run constantly
	            } catch(InterruptedException e) 
	            {
	            	e.printStackTrace();
	            }
			} catch (Exception e1) 
			{
				e1.printStackTrace();
			}
            
            
        }
		
	}
	
	//Phase 3: send the data to the OBC
	public void write(byte[] data)
	{	
		try {
			base.send(new DatagramPacket(data,4, this.address, this.port));
			//System.err.println(data[0] + " " + (byte)(char)data[1] + " " + data[2] + " " + data[3]);
		} catch (IOException e) { e.printStackTrace(); }	
	}//write method

	
	
	
	
	public static void main(String[] args) 
	{
		JoystickInterpret jc = null;
		try {
			jc = new JoystickInterpret("localhost", 30001, new DatagramSocket());
		} catch (SocketException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		//jc.setDeadZone(0.04);
		//jc.setPollInterval(50);
		//jc.updateDeadZone();
		
		jc.startPolling();
		//jc.stop();
		//jc.stopPolling();
		//jc.run_bool = false;
		//jc.startPolling();
		
	}
}
