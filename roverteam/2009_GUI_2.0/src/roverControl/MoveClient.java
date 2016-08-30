//package roverControl;
//import com.centralnexus.input.Joystick;
//
//
//public class MoveClient 
//{
//	public static void main(String[] args)
//	{
//		try
//		{
//			RoverMoveThread j = new RoverMoveThread("artemis.cse.yorku.ca", 30001);
//			@SuppressWarnings("unused")	int joystickNum = -1;
//	        double deadZone = 0.005;
//	        int interval = 100;
//	          
//	        for (int idx = 0; idx < args.length; idx++) 
//	        {
//	              if (args[idx].startsWith("-d:")) 
//	              {
//	                  deadZone = 
//	                      Double.valueOf(args[idx].substring(3, args[idx].length()))
//	                      .doubleValue();
//	               
//	              }
//	              else if (args[idx].startsWith("-i:")) 
//	              {
//	                  interval = 
//	                      Integer.valueOf(args[idx].substring(3, args[idx].length()))
//	                      .intValue();
//	              }
//	              else if (args[idx].startsWith("-v")) {
//	                  for (int id = -1; id <= Joystick.getNumDevices(); id++) {
//	                      System.out.println("Joystick " + id + ": " + Joystick.isPluggedIn(id));
//	                  }
//	              }
//	              else if (args[idx].startsWith("-h")) 
//	              {
//	                  help();
//	              }
//	              else {
//	                  System.out.println("Unknown option: " + args[idx]);
//	                  help();
//	              }
//	          }
//			
//	        if (deadZone >= 0.0) 
//	        {
//	              j.setDeadZone(deadZone);
//	        }
//	        
//			j.setPollInterval(interval);
//			j.updateDeadZone();
//			j.startPolling();
//			
//			//System.exit(0);
//		}catch (Exception e)
//		{
//			e.printStackTrace();
//		}
//	}
//	
//	 private static void help() 
//	  {
//	      System.out.println("Help:");
//	      System.out.println(" -h This help screen info");
//	      System.out.println(" -v Verbose Joystick debug information");
//	      /*System.out.println(" -j:n Set the Joystick ID to test (n is an integer)");
//	      System.out.println(" -j2:n Set the second joystick ID to test (n is an integer)");*/
//	      System.out.println(" -d:n Set the dead zone size of the Joystick (n is a real number)");
//	     // System.out.println(" -d2:n Set the dead zone size of the second Joystick (n is a real number)");
//	  }
//
//}
