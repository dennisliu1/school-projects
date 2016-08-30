package interfaces;
import java.io.IOException;

/**
 * This interface provides the general specification of any network program that runs from the
 * base. Each such program connects to a corresponding program on the rover and sends (and possibly
 * receives) information in chunks of exactly one byte.
 * 
 * @author Damon Sotoudeh
 * @version 1.0
 */
public interface Client
{	
	/**
	 * Converts the two passed integers to a single byte and then transmits the byte to rover.
	 * Conversion is as follows:
	 * <ul>
	 * <li>
	 *   The 3 leftmost bits will be used for device id. This means the device id can take
	 *   8 values ranging from 0 to 7 only.
	 *  </li>
	 *  <li>
	 *   The 5 right most bits are used for the parameter associated with the device. This
	 *   means the parameter can take 32 values ranging from 0 to 31 only.
	 *  </li>
	 * </ul>
	 * The two parts are combined together to form a single byte containing information about
	 * a particular device and its associated parameter. Once the conversion is done, the byte
	 * is transmitted to rover.
	 * 
	 * @param id the device id, ranging from 0 to 7 inclusive.
	 * @param parameter the device parameter, ranging from 0 to 31 inclusive.
	 * @throws IOException if any I/O error occurs.
	 * @throws IllegalArgumentException if any of the arguments is out of range.
	 */
	public void write(int id, int parameter) throws IOException;

	/**
	 * Reads and returns at most a single byte that is received from rover.
	 * 
	 * @return a single byte received from rover.
	 * @throws IOException If the first byte cannot be read for any reason
	 * other than the end of the file, if the input stream has been closed,
	 * or if some other I/O error occurs. 
	 */
	public byte read() throws IOException;

	/**
	 * Closes this socket including the input and output streams.
	 * Once a socket is closed, it is essentially terminated and
	 * cannot be used for further communication.
	 * 
	 * @throws Exception if an I/O error occurs when closing this socket.
	 */
	public void close() throws Exception;
}
