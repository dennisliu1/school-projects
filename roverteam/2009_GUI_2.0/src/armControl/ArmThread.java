///***********************************************************
// * Control Station Joystick Command transmitter (Arm) 
// * for Mars Rover Project @ York
// * 
// * @author Stanley Diji
// * Date: currently being updated
// * Version: 1.1
// * requires some packages that may be missing. contact
// * dijistanley@gmail.com for more information
// *************************************************************/
//package armControl;
//
//import java.net.DatagramPacket;
//import java.net.DatagramSocket;
//import java.net.InetAddress;
//
//import com.centralnexus.input.*;
//
//import view.View;
//
//public class ArmThread implements Runnable
//{
//	private DatagramSocket base;
//	private int port;
//	private InetAddress address;
//	private View view;
//	private Joystick joy;
//	public int interval;
//	public Float deadZone1;
//	private Thread armthread;
//	private int button;
//	float X, Y, Z;
//	int SPEED_MAX = 127;
//	int LEFT_RIGHT_DIRECTION; //byte[0]
//	int UP_DOWN_DIRECTION; // byte[1]
//	int F_B_DIRECTION; // byte[2]
//	int B_DOWN_UP_DIRECTION; // byte[3]
//	int bytezerospeed = 0;
//	int byteonespeed = 0;
//	int ARM_SPEED = 0;
//	int SERVO_MAX_ANGLE = 0;
//	int servo_angle = SERVO_MAX_ANGLE;
//	
//	public ArmThread(String ip, int port, View view)
//	{
//		try {
//			this.address = InetAddress.getByName(ip);
//			this.port = port;
//			this.base = new DatagramSocket();
//			this.view = view;
//			armthread = new Thread(this);
//			this.joy = Joystick.createInstance();
//			this.interval = 50;
//			
//		} catch (Exception e) 
//		{
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		} 
//	}
//	
//	public ArmThread()
//	{
//		try {
//			this.address = InetAddress.getLocalHost();
//			this.port = 30010;
//			this.base = new DatagramSocket();
//			armthread = new Thread(this);
//			this.joy = Joystick.createInstance();
//			this.interval = 50;
//			
//		} catch (Exception e) 
//		{
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		} 
//	}
//	//=======================================================================================	
//	public void updateFieldsEx(Joystick joystick) throws Exception
//    {
//		
//		//if(convertButton(button) != button) 
//		button = convertButton(joystick.getButtons());
//		
//		this.X = joy.getX();
//		this.X = ((this.X + 1) / 2) * 1023;
//        //inverse Joy stick (Y axis)
//        this.Y = (joy.getY() != 0)? -joy.getY():joy.getY();
//        this.Y = ((this.Y + 1) / 2) * 1023;
//        //inverse Joy stick (z)
//        this.Z = ((-joy.getZ()+1)/2)*SPEED_MAX;
//        
//        byte[] packets = new byte[10];
//		
//		this.ARM_SPEED = (int)this.Z;//(int)Double.parseDouble(view.getArmSpeed());
////		this.stop();
//		packets = this.nulifyPackets(packets);
//		try{
//		/*******************************
//		 *  Byte zero Control (up / down)
//		 *******************************/
//		if(button == 3) //byte[0] up
//		{
//			byte b = (byte) (1 << 7);
//			b |= (byte) view.b0speed;
//			packets[0] = b;
//			
//			this.write(packets);
//		}else if(button == 2) // byte[0] down
//		{
//			byte b = (byte) (0 << 7);
//			b |= (byte) view.b0speed;
//			packets[0] = b;
//			
//			this.write(packets);
//		}
//		/***********************************
//		 *  Byte one control (left - right)
//		 **********************************/
//		else if(button == 4) // byte[1] left
//		{
//			byte b = (byte) (0 << 7);
//			b |= (byte) view.b1speed;
//			packets[1] = b;
//			
//			this.write(packets);
//		}else if(button == 5) // byte[1] right
//		{
//			byte b = (byte) (1 << 7);
//			b |= (byte) view.b1speed;
//			packets[1] = b;
//			
//			this.write(packets);
//		}
//		/***********************************
//		 *  Byte two control (foward / backward)
//		 ***********************************/
//		else if(button == 6) // byte[2] forward
//		{
//			byte b = (byte) (1 << 7);
//			b |= (byte) view.b2speed;
//			packets[2] = b;
//			
//			this.write(packets);
//		}else if(button == 7) // byte[2] backward
//		{
//			byte b = (byte) (0 << 7);
//			b |= (byte) view.b2speed;
//			packets[2] = b;
//			
//			this.write(packets);
//		}
//		/*****************************************
//		 *  Byte three control (tiltup / tiltdown)
//		 *****************************************/
//		else if(button == 8) // byte[3] tilt up
//		{
//			byte b = (byte) (1 << 7);
//			b |= (byte) view.b3speed;
//			packets[3] = b;
//			
//			this.write(packets);
//		}else if(button == 9) // byte[3] tilt down
//		{
//			byte b = (byte) (0 << 7);
//			b |= (byte)view.b3speed;
//			packets[3] = b;
//			
//			this.write(packets);
//		}
//		
//		if (button == 1) // Servo Motor control :: byte[4], byte[5], byte[6], byte[7]
//		{
//			if((int)this.Y > 0) //byte[4] - servo1[0]
//			{
//				byte b = (byte) (0 << 7);
//				servo_angle = (int)this.Y;
//				b |= (byte)servo_angle;
//			}else if((int)this.Y > 0) //byte[5] - servo1[1]
//			{
//				
//			}
//		}
//		}catch (Exception e)
//		{
//			throwexception(e);
//		}
//    }
//	private byte[] nulifyPackets(byte[] packets)
//	{
//		for(int i=0; i < packets.length; i++)
//		{
//			packets[i] = (byte) 0;
//		}
//		
//		return packets;
//		
//	}
//	private int convertButton(int buttonID)
//	{
//		int ans = 0;
//		int division=buttonID;
//		  
//		while(division > 0)
//		{
//			ans++;
//			division = division / 2;
//		}
//		return ans;
//	  }
//	/*********************************************************
//	 * byte 0 : DC motor up/down
//	 * byte 1 : DC motor left/right
//	 * byte 2 : DC motor forward/backward (arm extension)
//	 * byte 3 : DC motor to bend the arm down and up
//	 * byte 4 : servomotor 1
//	 * byte 5 : servomotor 2
//	 * for bytes 0, 1, 2 and 3 :
//	 * the first most significant bit control the direction (bit 7)
//	 * the 7 least significant bits control :
//	 *  - the speed for DC motors (bit 6 to 0)
//	 *  - the speed for DC motor (we will have to determine and set min and max speed experimentally)
//	 *   for byte 4 and 5 :
//	 *   the whole byte control the angle of the servo (8bit resolution angle)
//	 *   the joystick axes value will be used to increment/decrement the angle
//	 *   or we can use an increment / decrement buttons to set the angle with different steps (1�, 10�, 45�) 
//	 *   it might be easier to control the arm this way
//	 *   
//	 *   about the speed :
//	 *   if speed = 0 : the motor is stopped
//	 *   if speed = 1 : the motor is in brake mode (both connection tied to ground)
//	 *   if speed > 1 : this set the motor to the desired speed (from 2 to 127)
//	 ******************************************************************************/
//	void write(byte[] data)
//	{
//		try {
//			base.send(new DatagramPacket(data,10, this.address, this.port));
//			setText("data 0 : " + Byte.valueOf(data[0]) + ", data 1: " + Byte.toString(data[1]) + ", data 2: " + Byte.toString(data[2]) + ", data 3: " + Byte.toString(data[3])+ ", data 4: " + Byte.toString(data[4]));
//			setText("data 5 : " + Byte.valueOf(data[5]) + ", data 6: " + Byte.toString(data[6]) + ", data 7: " + Byte.toString(data[7]));
//		} catch (Exception e) {
//			setText("Error sending data packet");
//		}		
//	}
//	
//	public void stoap()
//	{
//		byte b = (byte) 1;
//		int angle = (int) (((-this.X +1) / 2) * 512);
//		byte[] packets = new byte[8];
//		for(int i=0; i < 8; i++)
//		{
//			if(i==4 || i==6)
//			{
//				packets[i] = (byte) ((byte)((byte)angle << 1) >> 1);
//				packets[i] |= (byte)(1<< 7);
//			}
//			else
//			{
//				packets[i] = b;
//			}
//		}
//		this.write(packets);
//	}
//	
//	public void updateSpeed(double speed)
//	{
//		this.ARM_SPEED = (int)speed;
//	}
//	
//	@Override
//	public void run() 
//	{
//		try
//    	{
//	        for (;;) 
//	        {
//	            joy.poll();
//	            //Send Info
//	            updateFieldsEx(joy);
//	            
//	            try 
//	            {
//	                Thread.sleep(interval);
//	            } catch(InterruptedException e) {
//	            	this.view.throwException(e);
//	                break;
//	            }
//	        }
//    	}catch (final Exception e)
//    	{
//    		//e.printStackTrace();
//    		//this.view.throwException(e);
//    		if(!this.view.getDisplay().isDisposed())//!aGUI.getDisplay().isDisposed()){
//	            view.getDisplay().asyncExec (new Runnable (){
//	            	public void run () {
//	                  view.throwException(e);//.doSomeGUIModifications();
//	               }
//	        });
//    		else
//    			System.out.println(e.toString());
//    	}
//		
//	}
//	
//	public void setPollInterval(int pollMillis) 
//	  {
//	      this.interval = pollMillis;
//	      joy.setPollInterval(pollMillis);
//	  }
//	  public void startPolling() 
//	  {
//	      armthread.start();
//	  }
//	  @SuppressWarnings("deprecation")
//	public void stopPolling()
//	  {
//		  armthread.stop();
//	  }
//	  public void setDeadZone(double deadZone) 
//	  {
//	      joy.setDeadZone(deadZone);
//	      this.updateDeadZone();
//	  }
//	  public void updateDeadZone() 
//	  {
//	      this.deadZone1 = joy.getDeadZone();
//	  }
//	  
//	  void throwexception(final Exception e)
//	  {
//		  if(!this.view.getDisplay().isDisposed())//!aGUI.getDisplay().isDisposed()){
//	            view.getDisplay().asyncExec (new Runnable (){
//	            	public void run () {
//	                  view.throwException(e);//.doSomeGUIModifications();
//	               }
//	        });
//  		else
//  			System.out.println(e.toString());
//	  }
//	  
//	  void setText(final String s)
//	  {
//		  if(!this.view.getDisplay().isDisposed())//!aGUI.getDisplay().isDisposed()){
//	            view.getDisplay().asyncExec (new Runnable (){
//	            	public void run () {
//	                  view.throwException(s);//.doSomeGUIModifications();
//	               }
//	        });
//		else
//			System.out.println(s);
//	  }
//	  
//
//}