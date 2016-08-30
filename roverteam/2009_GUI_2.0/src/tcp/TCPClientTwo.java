package tcp;

import java.io.IOException;

public class TCPClientTwo extends TCPClient
{

	public TCPClientTwo(String roverIP, int port) throws IOException 
	{
		super(roverIP, port);	
	}
	
	public void write(int parameter) throws IOException
	{
		if(parameter < 0 || parameter > 31)
		{
			throw new IllegalArgumentException("parameter must be between 0 and 31 inclusive!");
		}
		
		byte b = (byte) parameter ;
		//b |= ((byte)parameter);
		
		out.write(new byte[]{b});
		out.flush();
	}
	
	public void write(float parameter) throws IOException
	{
		if(parameter < 0 || parameter > 31)
		{
			throw new IllegalArgumentException("parameter must be between 0 and 31 inclusive!");
		}
		
		byte b = (byte) parameter ;
		//b |= ((byte)parameter);
		
		out.write(new byte[]{b});
		out.flush();
	}

	public void write(byte[] bs) throws IOException 
	{
		
		out.write(bs);
		out.flush();
	}

}
