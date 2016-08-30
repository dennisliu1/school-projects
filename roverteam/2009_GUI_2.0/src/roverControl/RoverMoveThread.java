///***********************************************************
// * Control Station Joystick (drive) Command transmitter for 
// * Mars Rover Project @ York
// * 
// * @author Stanley Diji
// * Date: currently being updated
// * Version: 1.1
// * requires some packages that may be missing. contact
// * dijistanley@gmail.com for more information
// *************************************************************/
//
//package roverControl;
//
//import java.net.DatagramPacket;
//import java.net.DatagramSocket;
//import java.net.InetAddress;
//import com.centralnexus.input.*;
//
//import view.View;
//
//public class RoverMoveThread implements Runnable
//{
//	private DatagramSocket base;
//	private int port;
//	private InetAddress address;
//	private View view;
//	private Joystick joy;
//	public int interval;
//	public Float deadZone1;
//	private Thread roverthread;
//	private int button;
//	float X, Y, Z;
//	int SPEED_MAX = 127;
//	private final Double MID_X = 0.0;
//	private final Double MID_Y = 0.0;
//	private final int BREAK = 0;
//	private final int FORWARD = 0;
//	private final int REVERSE = 1;
//	private int LEFT_SPEED = 0;
//	private int RIGHT_SPEED = 0;
//	int started = 0;
//	public RoverMoveThread(String ip, int port, View view)
//	{
//		try {
//			this.address = InetAddress.getByName(ip);
//			this.port = port;
//			this.base = new DatagramSocket();
//			this.view = view;
//			
//			roverthread = new Thread(this);
//			this.joy = Joystick.createInstance();
//			this.interval = 1000;
//			
//		} catch (Exception e) 
//		{
//			// TODO Auto-generated catch block
//			System.out.println(e.toString());
//			this.view.throwException(e);
//			//System.out.println(e.toString());
//		} 
//	}
//	
//	public RoverMoveThread(String ip, int port)
//	{
//		try {
//			this.address = InetAddress.getByName(ip);
//			this.port = port;
//			roverthread = new Thread(this);
//			this.joy = Joystick.createInstance();
//			this.interval = 50;
//		} catch (Exception e) 
//		{
//			// TODO Auto-generated catch block
//			System.out.println(e.toString());
//			this.view.throwException(e);
//			//System.out.println(e.toString());
//		} 
//	}
//	//=======================================================================================	
//	public void updateFieldsEx(Joystick joystick) throws Exception
//    {
//		//button = 0;
//		
//		//if(convertButton(joystick.getButtons()) != button) 
//		//	button = convertButton(joystick.getButtons());
//		
//		//Saitek: get status of 3 buttons STOP(1), FORWARD(8), REVERSE(4)
//		button = 0;
//		if (joy.isButtonDown(1)){
//			button = 1; //STOP
//		} else if (joy.isButtonDown(8)) {
//			button = 2; //REVERSE
//		} else if (joy.isButtonDown(4)) {
//			button = 3; //FORWARD
//		} else if (joy.isButtonDown(2)) {
//			button = 4; //TURN
//		}
//		
//		
//		//System.out.println("joy out:  X = " + joy.getX() + " Y = " + joy.getY() + " Z = " + joy.getZ());
//        //inverse Joy stick (Y axis)
//       // this.Y = (joy.getY() != 0)? -joy.getY():joy.getY();
//		this.Z = ((-joy.getZ()+1)/2)*SPEED_MAX; //speed_max = 128 half = 128 * 0.5
//        this.Y = (((-joy.getY() + 1) / 2) * (int)(this.Z * 2)) - (int) this.Z;
//      //this.X = ((-joy.getX() + 1) / 2) * 63; // if X < (31.5) move left if X > 31.5 move right else foward
//		this.X = (((-joy.getX() + 1) / 2) * (int)(this.Z * 2)) - (int) this.Z;
//        //inverse Joy stick (z)
//        
//		//System.out.println("X = " + this.X + " Y = " + this.Y + " Z = " + this.Z);
//        
//        byte[] packets = new byte[4];
//        //int speed = (int) this.Z != 1? (int) this.Z: 0; // speed = {0} UNION [2:36]
//        int speed = (int) this.Z;
//        /*if(speed != started)
//        {
//        	started = speed;
//        	this.LEFT_SPEED = speed;
//        	this.RIGHT_SPEED = speed;
//        }
//        */
//        if(button == 1)// stop
//        {
//        	this.stop();
//        }else if(button == 2) // reverse
//        {
//        	
//        	int spd = (int)this.Y;
//        	int left, right;
//        	
//        	if (this.Y >= 0) {
//        		left = right = 0;
//        	} else if (this.X == 0){
//        		left = right = spd;
//        	} else if (this.X < 0) {
//        		left = spd;
//        		right = (int)((1 + (this.X/this.Z))*spd);
//        	} else { //if (this.X < 0)
//        		left = (int)((1 - (this.X/this.Z))*spd);
//        		right = spd;
//        	}
//        	
//        	//left = spd - ((int)this.X / 2);
//        	//right = spd + ((int)this.X / 2);
//        	
//        	packets[0] = packets[2] = (byte)left;
//        	packets[1] = packets[3] = (byte)right;
//        	
//        	/*
//        	if(this.X > this.MID_X && this.Y < this.MID_Y) // move left Reverse
//        	{
//        		byte b = (byte) (REVERSE << 7);
////        		b |= (byte) speed / 4;
//        		LEFT_SPEED = ((int) this.X - (int) this.Z) < 0 ? -1 *((int) this.X - (int) this.Z):((int) this.X - (int) this.Z) ;
//        		//b |= LEFT_SPEED <= 127? (byte) LEFT_SPEED : (byte) SPEED_MAX;
//        		b |= (byte)LEFT_SPEED;
//        	//	System.out.println("Left Speed = " + LEFT_SPEED + " Right speed +" + RIGHT_SPEED);
//        		
//        		packets[1] = b;
//        		packets[3] = b;
//        		
//        		b = (byte) (REVERSE << 7);
//        		//b |= (byte) speed;
//        		b = (byte) (FORWARD << 7);
//        		RIGHT_SPEED = (int) this.Z;
//        		b |= (byte) RIGHT_SPEED;
//        		
//        		packets[0] = b;
//        		packets[2] = b;
//        	//	System.out.println("Left Speed = " + LEFT_SPEED + " Right speed +" + RIGHT_SPEED);
//        	}else if (this.X < this.MID_X && this.Y < this.MID_Y) // move right Reverse
//        	{
//        		byte b = (byte) (REVERSE << 7);
//        		//b |= (byte) speed/4;
//        		RIGHT_SPEED = (int) this.X + (int) this.Z; 
//        		//b |= LEFT_SPEED <= 127? (byte) LEFT_SPEED : (byte) SPEED_MAX;
//        		b |= (byte) RIGHT_SPEED;
//        		//b |= (byte) speed/8;
//        		
//        		packets[0] = b;
//        		packets[2] = b;
//        		
//        		b = (byte) (REVERSE << 7);
//        		//b |= (byte) speed;
//        		LEFT_SPEED = (int)this.Z;
//        		//b |= RIGHT_SPEED <= 127? (byte) RIGHT_SPEED : (byte) SPEED_MAX;
//        		b |= (byte) LEFT_SPEED;
//        		
//        		packets[1] = b;
//        		packets[3] = b; 
//        	//	System.out.println("Left Speed = " + LEFT_SPEED + " Right speed +" + RIGHT_SPEED);
//        	}else if(this.Y < this.MID_Y && this.X == this.MID_X)
//        	{
//        		byte b = (byte) (REVERSE << 7);
//        		//b |= (byte) speed/4;
//        		LEFT_SPEED = speed;
//        		b |= (byte) LEFT_SPEED;
//        		
//        		packets[0] = b;
//        		packets[2] = b;
//        		
//        		b = (byte) (REVERSE << 7);
//        		//b |= (byte) speed;
//        		RIGHT_SPEED = speed;
//        		b |= (byte) RIGHT_SPEED;
//        		        		
//        		
//        		packets[1] = b;
//        		packets[3] = b;
//        	//	System.out.println("Left Speed = " + LEFT_SPEED + " Right speed +" + RIGHT_SPEED);
//        	}
//        	*/
//        	this.write(packets);
//        	
//        }else if(button == 3) // Forward Drive
//        {
//        	
//        	int spd = (int)this.Y;
//        	int left, right;
//        	
//        	if (this.Y <= 0) {
//        		left = right = 0;
//        	} else if (this.X == 0){
//        		left = right = spd;
//        	} else if (this.X < 0) {
//        		left = spd;
//        		right = (int)((1 + (this.X/this.Z))*spd);
//        	} else { //if (this.X < 0)
//        		left = (int)((1 - (this.X/this.Z))*spd);
//        		right = spd;
//        	}
//        	
//        	//left = spd - ((int)this.X / 2);
//        	//right = spd + ((int)this.X / 2);
//        	
//        	packets[0] = packets[2] = (byte)left;
//        	packets[1] = packets[3] = (byte)right;
//        	
//        	/*
//        	if(this.X > this.MID_X && this.Y > this.MID_Y) // move left
//        	{
//        		byte b = (byte) (FORWARD << 7);
//        		LEFT_SPEED = ((int) this.X - (int) this.Z) < 0 ? -1 * ((int) this.X - (int) this.Z): ((int) this.X - (int) this.Z)  ;
//        		LEFT_SPEED = LEFT_SPEED ==1 ? 0 : LEFT_SPEED;
//        		//b |= LEFT_SPEED <= 127? (byte) LEFT_SPEED : (byte) SPEED_MAX;
//        		b |= (byte)LEFT_SPEED;
//        		
//        		packets[1] = b;
//        		packets[3] = b;
//        		
//        		b = (byte) (FORWARD << 7);
//        		RIGHT_SPEED = speed;
//        		b |= (byte) RIGHT_SPEED;
//        		
//        		packets[0] = b;
//        		packets[2] = b; 
//        	//	System.out.println("Left Speed = " + LEFT_SPEED + " Right speed +" + RIGHT_SPEED);
//        	}else if (this.X < this.MID_X && this.Y > this.MID_Y) // move right
//        	{
//        		byte b = (byte) (FORWARD << 7);
//        		RIGHT_SPEED = (int) this.X + (int) this.Z; 
//        		//b |= LEFT_SPEED <= 127? (byte) LEFT_SPEED : (byte) SPEED_MAX;
//        		b |= (byte) RIGHT_SPEED;
//        		//b |= (byte) speed/8;
//        		
//        		packets[0] = b;
//        		packets[2] = b;
//        		
//        		b = (byte) (FORWARD << 7);
//        		LEFT_SPEED = speed;
//        		//b |= RIGHT_SPEED <= 127? (byte) RIGHT_SPEED : (byte) SPEED_MAX;
//        		b |= (byte) LEFT_SPEED;
//        		packets[1] = b;
//        		packets[3] = b;
//        	//	System.out.println("Left Speed = " + LEFT_SPEED + " Right speed +" + RIGHT_SPEED);
//        	}else if((Y > this.MID_Y) && ((int)this.X == 0))//this.MID_X)
//        	{
//        		byte b = (byte) (FORWARD << 7);
//        		LEFT_SPEED = (int) this.Y;//speed;
//        		b |= (byte) LEFT_SPEED;
//        		
//        		packets[0] = b;
//        		packets[2] = b;
//        		
//        		b = (byte) (FORWARD << 7);
//        		RIGHT_SPEED = (int) this.Y;//speed;
//        		b |= (byte) RIGHT_SPEED;
//        		        		
//        		packets[1] = b;
//        		packets[3] = b;
//        //		System.out.println("Left Speed = " + LEFT_SPEED + " Right speed +" + RIGHT_SPEED);
//        //		System.out.println("Y = " + (int) this.Y);
//        	}
//        	
//        	if(this.X > this.MID_X && Y < 0)
//        	{
//        		byte b = (byte) (FORWARD << 7); 
//        		LEFT_SPEED = SPEED_MAX;
//        		b |= (byte) LEFT_SPEED;
//        		
//        		packets[0] = b; packets[2] = b;
//        		
//        		b = (byte) (REVERSE << 7); 
//        		RIGHT_SPEED = SPEED_MAX;
//        		b |= (byte) RIGHT_SPEED; // ON THE SPOT TURN
//        		packets[1] = b; packets[3] = b;
//        	//	System.out.println("Left Speed = " + LEFT_SPEED + " Right speed +" + RIGHT_SPEED);
//        	}else if(this.X < this.MID_X && Y < 0)
//        	{
//        		byte b = (byte) (FORWARD << 7); 
//        		RIGHT_SPEED = SPEED_MAX;
//        		b |= (byte) RIGHT_SPEED;
//        		
//        		packets[1] = b; packets[3] = b;
//        		
//        		b = (byte) (REVERSE << 7); 
//        		LEFT_SPEED = SPEED_MAX;
//        		b |= (byte) LEFT_SPEED; // ON THE SPOT TURN
//        		packets[0] = b; packets[2] = b;
//        	//	System.out.println("Left Speed = " + LEFT_SPEED + " Right speed +" + RIGHT_SPEED);
//        	}   
//        	*/
//        	  	this.write(packets);
//        } else if (button == 4) {
//        	
//        	int spd = (int)this.X;
//        	int left = -spd;
//        	int right = spd;
//        	
//        	packets[0] = packets[2] = (byte)left;
//        	packets[1] = packets[3] = (byte)right;
//        	
//        	this.write(packets);
//        	
//        } else
//        {
//        	
//        }
//    }
//	
//	public void stop()
//	{
//		byte b = (byte) BREAK;
//		byte[] packets = new byte[4];
//		for(int i=0; i < 4; i++)
//		{
//			packets[i] = b;
//		}
//		this.write(packets);
//	}
//	
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
//	/**
//	 * 
//	 * @param data Data to be sent (size of byte = 4)
//	 */
//	public void write(byte[] data)
//	{
//		try {
//			if(view != null)	
//			{
//		
//				base.send(new DatagramPacket(data,4, this.address, this.port));
//				setText("data 0 : " + Byte.valueOf(data[0]) + ", data 1: " + Byte.toString(data[1]) + ", data 2: " + Byte.toString(data[2]) + ", data 3: " + Byte.toString(data[3]));
//			//	setText(view.getText() + "\n data 4: " + data[4] + " data 5: " + data[5] + " data 6: " + data[6] + " data 7: " + data[7] + " ");
//			}else
//			{
//				System.out.println("x : " + (int)this.X + " Y : " + (int)this.Y);
//				//System.out.println("data 0: " + data[0] + " data 1: " + data[1] + " data 2: " + data[2] + " data 3: " + data[3] + " ");
//				//System.out.println("data 4: " + data[4] + " data 5: " + data[5] + " data 6: " + data[6] + " data 7: " + data[7] + " ");
//			}
//		} catch (Exception e) {
//			if(view != null)
//			{
//				//System.out.println("Exception Thrown!!!");
//				//this.view.setText(e.toString() + "\n Error sending data packet");
//				throwexception(e);
//			}
//			else
//				System.out.println("Error sending data packet");
//		}		
//	}
//	
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
//	            this.updateFieldsEx(joy);
//	            
//	            try 
//	            {
//	                Thread.sleep(interval);
//	            } catch(InterruptedException e) {
//	            	this.view.throwException(e);
//	                break;
//	            }
//	            
//	            Thread.sleep(100);
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
//	      roverthread.start();
//	  }
//	  @SuppressWarnings("deprecation")
//	public void stopPolling()
//	  {
//		  roverthread.stop();
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
//
