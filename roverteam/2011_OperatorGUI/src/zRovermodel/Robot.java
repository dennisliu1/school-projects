package zRovermodel;

import java.util.Iterator;
import java.util.LinkedList;

import aiInterface.data.DataContainer;

/**
 * 
 * @author glen
 *
 */
public abstract class Robot extends DataContainer
{

	private String name;
	private Integer identifier;
	private _dataType dataType;
	private String data;
	private String accessType;
	private double lowerBound, upperBound;
	/*
	 * This needs more consideration, how will this be used?
	 * It needs to map to how many bytes are used for this data.
	 * TODO
	 */
	public enum _dataType {
		_byte, _int, _long, _float, _double, _short
	}
	public static final String READ_ONLY = "R";
	public static final String READ_WRITE = "R+W";
	public static final String WRITE_ONLY = "W";
	private static final String DEFAULT_NAME = "unidentified";
	private static final Integer DEFAULT_ID = -1;
	private static final _dataType DEFAULT_DATATYPE = _dataType._byte;
	private static final String DEFAULT_DATA = "-1";
	private static final String DEFAULT_ACCESS = READ_ONLY;
	
	public static final String ROBOT_NAME = "name";
	public static final String ROBOT_ID = "ID";
	public static final String ROBOT_DATATYPE = "dataType";
	public static final String ROBOT_COMPONENT = "Component";
	public static final String ROBOT_COMPONENTS = "Components";
	
	protected Robot()
	{
		this.setName(DEFAULT_NAME);
		this.setIdentifier(DEFAULT_ID);
		this.setDataType(DEFAULT_DATATYPE);
		this.setData(DEFAULT_DATA);
		this.setAccessType(DEFAULT_ACCESS);
	}

	public String getName()
	{
		return name;
	}

	public void setName(String name)
	{
		this.name = new String(name);
	}

	public Integer getIdentifier()
	{
		return this.identifier;
	}

	public void setIdentifier(Integer identifier)
	{
		this.identifier = new Integer(identifier);
	}

	public _dataType getDataType()
	{
		return this.dataType;
	}

	public void setDataType(_dataType dataType)
	{
		this.dataType = dataType;
	}
	
	public String getData()
	{
		return this.data;
	}
	
	public void setData(String data)
	{
		this.data = new String(data);
	}
	
	public String getAccessType()
	{
		return new String(this.accessType);
	}
	
	public void setAccessType(String accesstype)
	{
		this.accessType = new String(accesstype);
	}
	
	public void setDataType(String type)
	{
		// If statements !!! woohooo
		if ( type.equals("_byte"))
		{
			this.setDataType(Robot._dataType._byte);
		}
		else if (type.equals("_int"))
		{
			this.setDataType(Robot._dataType._int);
		}
		else if (type.equals("_float"))
		{
			this.setDataType(Robot._dataType._float);
		}
		else if (type.equals("_long"))
		{
			this.setDataType(Robot._dataType._long);
		}
		else if (type.equals("_double"))
		{
			this.setDataType(Robot._dataType._double);
		}
	}
	
	public double getLowerBound() {
		return lowerBound;
	}
	public void setLowerBound(double lowerBound) {
		this.lowerBound = lowerBound;
	}
	public double getUpperBound() {
		return upperBound;
	}
	public void setUpperBound(double upperBound) {
		this.upperBound = upperBound;
	}
	
	public String toString()
	{
		String out = "";
		out = out + "name = " + this.getName().toString() + "\n";
		out = out + "Identifier = " + this.getIdentifier().toString() + "\n";
		out = out + "dataType = " + this.getDataType().toString() + "\n";
		out = out + "data = " + this.getData().toString() + "\n";
		out = out + "access type = " + this.getAccessType() + "\n";
		return out;
	}
	
	public String getSource()
	{
		return this.toString();
	}
	
}
