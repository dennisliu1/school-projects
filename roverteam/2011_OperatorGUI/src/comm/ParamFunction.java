package comm;

/**
 * Function that runs while inside a thread. The function runs when significant changes are done inside the thread,
 * or at the end of an iteration.
 * 
 * @version 1.0
 * @author Dennis Liu, YURT 2012
 */
public interface ParamFunction {
	public void execute();
}
