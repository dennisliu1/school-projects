package cameraTabs;

import java.awt.Canvas;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.GridLayout;
import java.awt.event.ActionListener;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextArea;


public class USBGUI {
	public static String hostname = "localhost";
	private static int startPort = 30200;
	private static int numberOfCameras = 1;
	static USBPlayer cam;
	static CameraInitPanel initPanel;
	static Canvas canvas;
	
	public static void main(String[]a) {
		JFrame frame = new JFrame("USB Video");
		frame.setLayout(new GridLayout(1,2));
			canvas = new Canvas();
			canvas.setPreferredSize(new Dimension(800,600));
			canvas.setSize(new Dimension(800,600));
			initPanel = new CameraInitPanel(hostname, ""+startPort);
			cam = new USBPlayer(canvas,initPanel);
			initPanel.startCamB.addActionListener(cam.new USBCamActionListener());
			initPanel.stopCamB.addActionListener(cam.new USBCamActionListener());
			
		frame.add(canvas);
		//frame.add(cam);
		frame.add(initPanel);
		//frame.pack();
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setVisible(true);
	}
	
}
