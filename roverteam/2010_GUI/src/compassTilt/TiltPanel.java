/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package compassTilt;

import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.JTextArea;

/**
 * Rover Tilt panel. Displays the horizontal and vertical tilt of the rover.
 * This is displayed in slider and numerical format for easy understanding.
 * @author Dennis Liu
 * YURT 2010-2011
 * --------------------------------
 * The Panel is divided into top/bottom sections. Top is horizontal, bottom is vertical.
 * This panel is placed on the top right of the GUI, next to the DriveArmPanel.
 * Each section shows:
 * - label+field for horizontal/vertical <h/v(Label,TextArea)>
 * * note that the label+field is nested in h/vNameNumPanel.
 * - slider <h/v Slider>
 * --------------------------------
 * Hierarchy:
 * RoverTiltPanel
 *	hPanel
 *		hNameNumPanel (holds label+field)
 *			hLabel, hTextArea
 *		hSlider
 *	vPanel
 *		vNameNumPanel (holds label+field)
 *			vLabel, vTextArea
 *		vSlider
 * --------------------------------
 * Constants:
 * TILT_MIN = 0							- minimum value for tilt
 * TILT_MAX = 100						- maximum value for tilt
 * TILT_INIT = (TILT_MAX+TILT_MIN)/2	- starting tilt value (middle)
 * TILT_MAJOR_TICK_SPACING = 10			- major tick spacing. For display only
 * TILT_MINOR_TICK_SPACING = 5			- minor tick spacing. For display only
 * --------------------------------
 */
//sliding bars for rover tilt sensor (horizontal, vertical, #s)
public class TiltPanel extends JPanel {
	public static final int TILT_MIN = -90, TILT_MAX = 90, TILT_INIT = (TILT_MAX+TILT_MIN)/2,
			TILT_MAJOR_TICK_SPACING = TILT_MAX/5,TILT_MINOR_TICK_SPACING = TILT_MAX/10, DEF_WIDTH = 210, DEF_HEIGHT = 139;
	int width, height;
	JSlider hSlider, vSlider;
	JLabel hLabel, vLabel;
	JTextArea hTextArea, vTextArea;
	JPanel hNameNumPanel, vNameNumPanel, hPanel, vPanel;
	
	int hTilt = TILT_INIT, vTilt = TILT_INIT;
	
	JButton testHB, testVB;
	TiltPanel self = this;

	public TiltPanel(int width, int height, boolean bTest) {
		super();//constructor for JPanel
		this.width = width;
		this.height = height;
		
		
		this.setPreferredSize(new Dimension(width, height));
		this.setLayout( new BoxLayout(this, BoxLayout.Y_AXIS) );//make layout vertical
		//make horizontal Panel
		hPanel = new JPanel();
		//label+field on top, slider below it. So use vertical layout
		hPanel.setLayout( new BoxLayout(hPanel, BoxLayout.Y_AXIS));
			//panel to store label+field (horizontal)
			hNameNumPanel = new JPanel();
			hNameNumPanel.setLayout( new FlowLayout() );
				hLabel = new JLabel("H Tilt:");//label
				hTextArea = new JTextArea(hTilt+"");//field
				hTextArea.setPreferredSize( new Dimension(50, 14));
			hNameNumPanel.add( hLabel );//add label+field together
			hNameNumPanel.add( hTextArea );
			//create and set slider
			hSlider = new JSlider(JSlider.HORIZONTAL, TILT_MIN, TILT_MAX, hTilt);
			hSlider.setMajorTickSpacing(TILT_MAJOR_TICK_SPACING);//set tick display
			hSlider.setMinorTickSpacing(TILT_MINOR_TICK_SPACING);
			hSlider.setPaintTicks(true);//show ticks and labels every major tick
			hSlider.setPaintLabels(true);
		hPanel.add(hNameNumPanel);//add label+field and slider together
		hPanel.add(hSlider);
		this.add(hPanel);//add horizontal panel to main panel
		//make vertical Panel. Refer to above.
		vPanel = new JPanel();
		vPanel.setLayout( new BoxLayout(vPanel, BoxLayout.Y_AXIS));
			vNameNumPanel = new JPanel();
			vNameNumPanel.setLayout( new FlowLayout() );
				vLabel = new JLabel("V Tilt:");
				vTextArea = new JTextArea(vTilt+"");
				vTextArea.setPreferredSize( new Dimension(50, 14));
				
			vNameNumPanel.add( vLabel );
			vNameNumPanel.add( vTextArea );
			vSlider = new JSlider(JSlider.HORIZONTAL, TILT_MIN, TILT_MAX, vTilt);
			vSlider.setMajorTickSpacing(TILT_MAJOR_TICK_SPACING);
			vSlider.setMinorTickSpacing(TILT_MINOR_TICK_SPACING);
			vSlider.setPaintTicks(true);
			//vSlider.setPaintLabels(true);
		vPanel.add(vSlider);
		vPanel.add(vNameNumPanel);
		this.add(vPanel);
		
		if( bTest ) {
			testHB = new JButton("Hori!");
			testHB.addActionListener(new TestActionListener());
			testVB = new JButton("Verti!");
			testVB.addActionListener(new TestActionListener());
			hNameNumPanel.add(testHB);
			vNameNumPanel.add(testVB);
		} else {
			hTextArea.setEditable(false);
			vTextArea.setEditable(false);
		}
	}//RoverTiltPanel constructor
	
	public void setHTilt(int value) {
		hTilt = value;
		hSlider.setValue(hTilt);
		hTextArea.setText(hTilt+"");
	}
	public void setVTilt(int value) {
		vTilt = value;
		vSlider.setValue(vTilt);
		vTextArea.setText(vTilt+"");
	}
	
	//------------------------------------
	private class TestActionListener implements ActionListener {
		
		public void actionPerformed(ActionEvent e) {
			if(e.getSource() == testHB) {
				System.out.println();
				self.setHTilt(Integer.parseInt(hTextArea.getText()));
			} else if(e.getSource() == testVB) {
				self.setVTilt(Integer.parseInt(vTextArea.getText()));
			}
		}//method
	}//class
	
	
	public static void main(String[] args) {
		JFrame test = new JFrame();
		TiltPanel p = new TiltPanel(211, 149, true);
		test.add(p);
		test.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		test.pack();
		System.out.println(test.getSize());
		test.setVisible(true);
	}
}//RoverTiltPanel class
