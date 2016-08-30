package view;

import java.util.*;

public class Error 
{
	private ArrayList<String> errors;
	private boolean status;
	public Error()
	{
		this.errors = new ArrayList<String>();
		this.status = true;
	}
	
	public void setStatus(boolean status)
	{
		this.status = status;
	}
	public boolean getStatus()
	{
		return this.status;
	}
	
	public void addFault(String fault)
	{
		this.errors.add(fault);
	}
	
	public String toString()
	{
		String ans = "";
		for(int i=0; i < this.errors.size(); i++)
		{
			ans = this.errors.get(i) + "\n";
		}
		return ans;
	}
	

}
