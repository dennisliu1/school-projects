package compassTilt;

import java.io.*;
import java.net.*;
import java.util.StringTokenizer;

public class CompassClient {

	private String address;
	private int portno;
	
	private Socket clientSocket;
	private DataOutputStream outToServer;
	private BufferedReader inFromServer;
	
	public int heading=0, tiltx=0, tilty=0;
	
	public CompassClient(String addr, int port) {
		// TODO Auto-generated constructor stub
		
		address = addr;
		portno = port;
		
		connect();
		
		
	}
	
	public boolean update(){
		
		StringTokenizer response = null;
		
		// Check if connection is still established
		if (clientSocket == null || !clientSocket.isConnected() || clientSocket.isClosed()){
			connect();
			return false;
		}
		
		try {
			outToServer.writeBytes("G");
			outToServer.flush();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			try {
				clientSocket.close();
			} catch (IOException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
			return false;
		}
		
		try {
			String msg = inFromServer.readLine();
			
			if (msg == null){
				return false;
			}
			
			response = new StringTokenizer(msg);
			
			heading = Integer.parseInt(response.nextToken());
			tiltx = Integer.parseInt(response.nextToken());
			tilty = Integer.parseInt(response.nextToken());
			
			return true;
			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			try {
				clientSocket.close();
			} catch (IOException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
			return false;
		}
		
		//return false;
		
	}
	
	private void connect(){
		
		if (clientSocket != null){
			// close socket before reoprening
			try {
				clientSocket.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
		
		try {
			clientSocket = new Socket(address, portno);
			clientSocket.setKeepAlive(true);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return;
		}
		
		// If no connection established, return
		if (clientSocket == null || !clientSocket.isConnected()){
			return;
		}
		
		try {
			outToServer = new DataOutputStream(clientSocket.getOutputStream());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		try {
			inFromServer = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}

}
/*
public static void main(String argv[]) throws Exception
{
 String sentence;
 String modifiedSentence;
 BufferedReader inFromUser = new BufferedReader( new InputStreamReader(System.in));
 Socket clientSocket = new Socket("localhost", 6789);
 DataOutputStream outToServer = new DataOutputStream(clientSocket.getOutputStream());
 BufferedReader inFromServer = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
 sentence = inFromUser.readLine();
 outToServer.writeBytes(sentence + '\n');
 modifiedSentence = inFromServer.readLine();
 System.out.println("FROM SERVER: " + modifiedSentence);
 clientSocket.close();
}
*/