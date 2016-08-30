//package controllers;
//
//import java.io.IOException;
//
//import com.centralnexus.input.Joystick;
//
//public class JoystickClient implements Runnable 
//{
//	private Thread roverthread;
//	private Joystick joy;
//	public int interval;
//	public Float deadZone1;
//	private int button;
//	float X, Y, Z;
//	private final int BREAK = 0;
//	private final int FORWARD = 0;
//	private final int REVERSE = 1;
//	private int LEFT_SPEED = 0;
//	private int RIGHT_SPEED = 0;
//	int started = 0;
//	public static final int SPEED_MAX = 63;
//	
//	public JoystickClient() 
//	{
//	
//		try {
//			this.joy = Joystick.createInstance();
//			this.interval = 100;
//			roverthread = new Thread(this);
//		} catch (Exception e) 
//		{
//			e.printStackTrace();
//		}
//		
//	}
//	
//	public void updateFieldsEx(Joystick joystick) throws Exception
//    {
//			boolean but1 = joystick.isButtonDown(1);
//			boolean butF = joystick.isButtonDown(4);
//			boolean butB = joystick.isButtonDown(8);
//			boolean butT = joystick.isButtonDown(2);
//			
//			button = convertButton(joystick.getButtons());
//			if(joy.getX() > 0)
//				this.X = joy.getX() * 10;
//			else if(joy.getX() < 0)
//				this.X = -joy.getX()*10;
//			else
//				this.X = 0;
//			
//			if(joy.getY() > 0)
//				this.Y = joy.getY() * 10;
//			else if(joy.getY() < 0)
//				this.Y = -joy.getY()*10;
//			else
//				this.Y = 0;
//		System.out.printf("Xaxis = %f : Yaxis = %f : Zaxis = %f : Button = %d : But1 = %b : ButF = %b : ButB = %b : ButT = %b\n", this.X, this.Y, joy.getZ(), button, but1, butF, butB, butT);
//    }
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
//	}
//	public void setPollInterval(int pollMillis) 
//	  {
//	      this.interval = pollMillis;
//	      joy.setPollInterval(pollMillis);
//	  }
//	  public void startPolling() 
//	  {
//	      roverthread.start();
//	  }
//
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
//	  
//	  public void stop()
//	  {
//		  byte b = (byte) BREAK;
//		  byte[] packets = new byte[4];
//		  for(int i=0; i < 4; i++)
//		  {
//			  packets[i] = b;
//		  }
//		  //this.write(packets);
//	  }	
//	
//	
//	public static void main(String[] args) 
//	{
//		JoystickClient jc = new JoystickClient();
//		jc.setDeadZone(0.04);
//		jc.setPollInterval(50);
//		jc.updateDeadZone();
//		jc.startPolling();
//		
//	}
//
//	@Override
//	public void run() 
//	{
//		for (;;) 
//        {
//            joy.poll();
//
//            try 
//            {
//				this.updateFieldsEx(joy);
//				try 
//	            {
//	                Thread.sleep(interval);
//	            } catch(InterruptedException e) 
//	            {
//	            	e.printStackTrace();
//	            }
//			} catch (Exception e1) 
//			{
//				e1.printStackTrace();
//			}
//            
//            
//        }
//		
//	}
//
//}
