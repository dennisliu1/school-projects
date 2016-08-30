package zRovermodel;

import java.util.Calendar;
import java.util.Date;
import java.util.LinkedList;

public class SerializedComponent extends Component
{

	/*
	 * This flag is to indicate that this component should be writen on its
	 * next call to send a message to the rover
	 */
	private boolean writeFlag;
	private Calendar time;
	private int resetTime = 1000;
	private int readTime = 1000;
	
	public SerializedComponent()
	{
		super();
	}
	
	public SerializedComponent(String name, Integer identifier,
			Robot._dataType dataType)
	{
		super(name, identifier, dataType);
		this.unsetWriteFlag();
		this.time = Calendar.getInstance();
		time.setTime(new Date(System.currentTimeMillis()));
	}
	
	public SerializedComponent(String name, Integer identifier, Robot._dataType dataType, 
			String data, String accessType, boolean writeFlag,  LinkedList<Component> components)
	{
		super(name, identifier, dataType, data, accessType, components);
		this.setWriteFlag(writeFlag);
	}
	
	public SerializedComponent( SerializedComponent sComp)
	{
		this(sComp.getName(), sComp.getIdentifier(), sComp.getDataType(),
				sComp.getData(), sComp.getAccessType(), sComp.getWriteFlag(),
				sComp.getComponents());
	}
	
	public void setWriteFlag()
	{
		this.writeFlag  = true;
	}
	
	public void setWriteFlag(boolean set)
	{
		this.writeFlag = set;
	}
	
	public void unsetWriteFlag()
	{
		this.writeFlag = false;
	}
	
	public boolean getWriteFlag()
	{
		return this.writeFlag;
	}
	public int getResetTimer() {
		return this.resetTime;
	}
	public int getReadTimer() {
		return this.readTime;
	}
	public void setResetTimer(int t) {
		this.resetTime = t;
	}
	public void setReadTimer(int t) {
		this.readTime = t;
	}
	public void setTime(long time) {
		this.time.setTime(new Date(time));
	}
	public boolean isPastResetTime(Calendar d) {
		if(resetTime < 0) return false;
		return d.getTimeInMillis() >= time.getTimeInMillis()+resetTime;
	}
	public boolean isPastReadTime(Calendar d) {
		if(readTime < 0) return false;
		return d.getTimeInMillis() >= time.getTimeInMillis()+readTime;
	}
}
