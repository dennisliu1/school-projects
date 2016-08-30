package tcp;
import interfaces.Client;
import java.io.*;
import java.net.*;

/**
 * The base side of an end point of a communication channel between a device in the base
 * and the rover based on TCP protocol.
 * 
 * @author Damon Sotoudeh
 * @version 2.0
 */
public class TCPClient implements Client
{
	private Socket base;
	protected InputStream in;
	protected OutputStream out;
	
	/**
	 * Opens a communication channel to rover on the given port.
	 * 
	 * @param port port number.
	 * @param roverIP IP address of the rover.
	 * @throws IOException if an I/O error occurs when creating the socket. 
	 */
	public TCPClient(String roverIP, int port) throws IOException
	{
		try
		{
			this.base = new Socket(roverIP, port);
			this.base.setTcpNoDelay(false);
			this.in = this.base.getInputStream();
			this.out = this.base.getOutputStream();
		}
		catch (UnknownHostException e)
		{
			System.out.println("Rover IP is not valid.");
			System.exit(-1);
		}
	}
	
	public void write(int id, int parameter) throws IOException
	{
		if(id < 0 || id > 7)
		{
			throw new IllegalArgumentException("id must be between 0 and 7 inclusive!");
		}
		if(parameter < 0 || parameter > 31)
		{
			throw new IllegalArgumentException("id must be between 0 and 31 inclusive!");
		}
		
		byte b = (byte) (id << 5);
		b |= ((byte)parameter);
		
		this.out.write(new byte[]{b});
		this.out.flush();
	}
	
	public byte read() throws IOException
	{
		byte[] b = new byte[1];
		this.in.read(b);
		return b[0];
	}
	
	public void close() throws IOException
	{
		this.in.close();
		this.out.close();
		this.base.close();
	}
}
