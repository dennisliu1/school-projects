package gui.testing;



/**
 * Function that will run in the GUIComponent. To use, implement "checkValues" for
 * when to do the update (should be when monitored values are changed. And
 * "doUpdate" is what your component needs to do in order to update the display.
 * 
 * Please do not override update(), or it will cause alot of problems... thanks.
 * For an example, look at TimerPanel2.
 * 
 * @version 1.0
 * @author Dennis Liu, YURT 2012
 */
public abstract class UpdateFunction {
	private int sleepTime = 1;
	public boolean setSleepTime(int millisec) {
		if(millisec >= 0) {
			sleepTime = millisec;
			return true;
		}
		return false;
	}
	/**
	 * thread runs this method. Does work if condition was met. When no work is done,
	 * function sleeps.
	 * *sleeping the thread when condition fails is ESSENTIAL. The thread will
	 * constantly run the functions added to it, so if you do not add Thread.sleep
	 * to the update, you are doing "busy waiting", which is terrible.
	 * @return if check values worked, and work is done.
	 */
	public boolean update() {
		if(!checkValues())	{//see if condition matches
			try{Thread.sleep(sleepTime); } catch(Exception e) {}//unsuccessful, sleep
			return false;
		}
		doUpdate();
		return true;
	}
	/**
	 * the check for when function should run. Since this class is to be implemented,
	 * it does not take arguments. Instead, you should pass the variables needed through
	 * construction.
	 * @return if condition was met
	 */
	public abstract boolean checkValues();
	/**
	 * work to do in the function. For outside variables, construct and pass the
	 * variables required.
	 */
	public abstract void doUpdate();
}
