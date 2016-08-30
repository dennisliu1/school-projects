package compassTilt;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.*;

public class CompassPanel extends JPanel{
	JTextArea degreeTArea, headingTArea;
	int width, height, scale;
	int cWidth = 10, cHeight = 65;
	int tol = 23, degree = 0;
	String sHeading = "", sDegree = "";
	CompassPaint compassDisplay;
	CompassPanel self = this;
	
	public CompassPanel(int compassOffset, int compassHeight, boolean bTest){
		super();
		this.cWidth = compassOffset;
		this.cHeight = compassHeight;
		this.setPreferredSize(new Dimension(100, 150));
		this.setLayout(new BoxLayout(this,BoxLayout.Y_AXIS));
		//this.setSize(160, 160);
		
		compassDisplay = new CompassPaint(compassOffset, compassHeight);
		this.add(compassDisplay);
		
		JPanel bottomPanel = new JPanel();
		bottomPanel.setLayout(new FlowLayout());
			headingTArea = new JTextArea(sHeading);
			headingTArea.setPreferredSize(new Dimension(40,20));
			headingTArea.setEditable(false);
		bottomPanel.add(headingTArea);
			degreeTArea = new JTextArea(sDegree);
			degreeTArea.setPreferredSize(new Dimension(40,20));
		bottomPanel.add(degreeTArea);
		
		if( bTest ) {//for testing purposes
			JButton changeAngle = new JButton("Test!");
			changeAngle.addActionListener( new MyActionListener());
			bottomPanel.add(changeAngle);
		} else {//when real compass is connected
			degreeTArea.setEditable(false);
		}
		
		this.add(bottomPanel);
	}
	
	public void changeHeading(int angle) {
		if( ((0 <= angle) && (angle < tol)) || ((360-tol < angle) && (angle <= 360)) ) sHeading = "N";
		else if( (( 45-tol <= angle) && (angle <  45+tol)) ) sHeading = "NE";
		else if( (( 90-tol <= angle) && (angle <  90+tol)) ) sHeading = "E";
		else if( ((135-tol <= angle) && (angle < 135+tol)) ) sHeading = "SE";
		else if( ((180-tol <= angle) && (angle < 180+tol)) ) sHeading = "S";
		else if( ((225-tol <= angle) && (angle < 225+tol)) ) sHeading = "SW";
		else if( ((270-tol <= angle) && (angle < 270+tol)) ) sHeading = "W";
		else if( ((315-tol <= angle) && (angle < 315+tol)) ) sHeading = "NW";
		else {
			sDegree = ":(";
			sHeading = "X.X!";
		}
		sDegree = Integer.toString(angle);
		
		headingTArea.setText(sHeading);
		degreeTArea.setText(sDegree);
	}
	//-----------------------------------------------------
		public class MyActionListener implements ActionListener {
			@Override
			public void actionPerformed(ActionEvent e) {
				compassDisplay.angle = degree = Integer.parseInt(degreeTArea.getText());
				compassDisplay.repaint();
				changeHeading(degree);
			}
		}//class MyActionListener
	
		/*
	   public static void main(String args[]) {
		    Compass panel = new Compass(10, 50, true);
		    JFrame frame = new JFrame("Compass");
		    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		    frame.add(panel);
	
		    //frame.setSize(200,230);
		    frame.pack();
		    System.out.println(frame.getSize());
		    frame.setVisible(true);
	  }
	  */
}
