package tcp;
import interfaces.Server;
import java.io.*;
import java.net.*;

/**
 * The rover side of an end point of a communication channel between a device in the base
 * and the rover. This class essentially creates a server that awaits for a single incoming
 * connection from the base based on TCP protocol..
 * 
 * @author Damon Sotoudeh
 * @version 2.0
 */
public class TCPServer implements Server
{
	private Socket rover;
	private InputStream in;
	private OutputStream out;
	
	/**
	 * Encapsulates a socket for communication with the base.
	 * 
	 * @param s a socket.
	 * @throws IOException if an I/O error occurs.
	 */
	public TCPServer(Socket s) throws IOException
	{
		this.rover = s;
		this.rover.setTcpNoDelay(false);
		this.in = this.rover.getInputStream();
		this.out = this.rover.getOutputStream();
	}
	
	public void write(byte b) throws IOException
	{
		this.out.write(new byte[]{b});
		this.out.flush();
	}
	
	public int[] read() throws IOException
	{
		int[] result = new int[2];
		byte[] b = new byte[1];
		this.in.read(b);
		
		byte b1 = b[0];
		byte b2 = (new Byte(b1)).byteValue();
		
		result[0] = (b1 >> 5) & 7;
		result[1] = b2 & 31;
		return result;
	}
	
	public void close() throws IOException
	{
		this.in.close();
		this.out.close();
		this.rover.close();
	}
}
