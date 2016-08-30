package interfaces;
import java.io.IOException;

/**
 * This interface provides the general specification of any network program that runs on the
 * rover. Each such program communicates with a program residing in the base station and receives
 * (and possibly sends) information in chunks of exactly one byte.
 * 
 * @author Damon Sotoudeh
 * @version 1.0
 */
public interface Server
{
	/**
	 * Sends a single byte to the base station program.
	 * 
	 * @param b a byte value.
	 * @throws IOException if an I/O error occurs.
	 */
	public void write(byte b) throws IOException;
	
	/**
	 * Reads and returns the two arguments that is received from base. The result is
	 * returned in the form of an integer array of size 2. <b>The first argument is
	 * the device id, and the second is the parameter associated with that device.</b>
	 * 
	 * @return an integer array of size 2, containing the two arguments received from the base.
	 * @throws IOException If the first byte cannot be read for any reason
	 * other than the end of the file, if the input stream has been closed,
	 * or if some other I/O error occurs.
	 * @see Client#write(int, int)
	 */
	public int[] read() throws IOException;
	
	/**
	 * Closes this socket including the input and output streams.
	 * Once a socket is closed, it is essentially terminated and
	 * cannot be used for further communication.
	 * 
	 * @throws Exception if an I/O error occurs when closing this socket.
	 */
	public void close() throws Exception;
}
