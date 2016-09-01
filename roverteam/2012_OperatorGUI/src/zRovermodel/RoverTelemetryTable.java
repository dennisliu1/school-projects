package zRovermodel;

import java.util.Calendar;
import java.util.Date;

import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.swing.JTable;

import Comm.Test.UDPClientGUI;
import Comm.UDP.UDPClient;

import zRovermodel.Robot._dataType;

public class RoverTelemetryTable {
	String[] columnNames;
	YURTRover rover;
	JTable table;
	JScrollPane scrollPane;
	ValueThread valueThread;
	
	public RoverTelemetryTable() {
		columnNames = new String[]{"Device Name", "DID","Field Name", 
				"FID", "LowerBound", "value", "UpperBound", "DataType", 
				"Permission", "Read Time", "Reset Time"};
		
		rover = YURTRover.buildRoverModel();
		int numFields = 0;
		for(int i = 0; i < rover.getComponents().size(); i++)
			numFields += rover.getComponents().get(i).getComponents().size();
		
		Object[][] data = new Object[numFields][columnNames.length];
		int iter = 0;
		String deviceName;
		int deviceID;
		String fieldName;
		int fieldID;
		double lowerBound,upperBound;
		String value;
		_dataType dataType;
		String permission;
		int readTime, resetTime;
		for(int i = 0; i < rover.getComponents().size(); i++) {
			deviceName = rover.getComponents().get(i).getName();
			deviceID = rover.getComponents().get(i).getIdentifier();
			for(int j = 0; j < rover.getComponents().get(i).getComponents().size(); j++) {
				fieldName = rover.getComponents().get(i).getComponents().get(j).getName();
				fieldID = rover.getComponents().get(i).getComponents().get(j).getIdentifier();
				lowerBound = rover.getComponents().get(i).getComponents().get(j).getLowerBound();
				value = rover.getComponents().get(i).getComponents().get(j).getData();
				upperBound = rover.getComponents().get(i).getComponents().get(j).getUpperBound();
				dataType = rover.getComponents().get(i).getComponents().get(j).getDataType();
				permission = rover.getComponents().get(i).getComponents().get(j).getAccessType();
				readTime = ((SerializedComponent)rover.getComponents().get(i).getComponents().get(j)).getReadTimer();
				resetTime = ((SerializedComponent)rover.getComponents().get(i).getComponents().get(j)).getResetTimer();
				data[iter][0] = deviceName;
				data[iter][1] = deviceID;
				data[iter][2] = fieldName;
				data[iter][3] = fieldID;
				data[iter][4] = lowerBound;
				data[iter][5] = value;
				data[iter][6] = upperBound;
				data[iter][7] = dataType;
				data[iter][8] = permission;
				data[iter][9] = readTime;
				data[iter][10] = resetTime;
				iter++;
			}
		}
		table = new JTable(data, columnNames);
		scrollPane = new JScrollPane(table);
		table.setFillsViewportHeight(true);
		valueThread = new ValueThread();
		valueThread.start();
	}//constructor
	public class ValueThread extends Thread {
		boolean isRunning;
		int counter, defaultCounter;
		
		public ValueThread() {
			isRunning = true;
			defaultCounter = 1000;
			counter = defaultCounter;
		}
		
		public void run() {
			int iter = 0;
			while(isRunning) {
				if(counter%1000 == 0) {
					iter = 0;
					for(int i = 0; i < rover.getComponents().size(); i++) {
						for(int j = 0; j < rover.getComponents().get(i).getComponents().size(); j++) {
							table.setValueAt(rover.getComponents().get(i).getComponents().get(j).getData(), iter, 6);
							iter++;
						}
					}
				}
				try { Thread.sleep(1000); } catch (InterruptedException ex) { }//sleep 1 ms
				if(counter <= 1) counter = defaultCounter;
				else counter -= 1000;
			}
		}
	}//ResetThread class
	
	public static void main(String[] a) {
		JFrame frameServer2 = new JFrame("Rover Table");
		RoverTelemetryTable table = new RoverTelemetryTable();
			frameServer2.add(table.scrollPane);
		frameServer2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frameServer2.pack();
		frameServer2.setVisible(true);
	}
}//RoverTelemetryTable class
