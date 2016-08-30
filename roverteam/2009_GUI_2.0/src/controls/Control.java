package controls;
/**
 * Control Button Mappings for Mars Rover Project @ york University
 * @author Luthor S. C. Diji
 * Version 1.0
 * dijistanley@gmail.com
 */
import java.awt.EventQueue;
import java.awt.SystemColor;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JTextArea;

public class Control extends JFrame {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
	private JTextArea textArea;
	/**
	 * Launch the application
	 * @param args
	 */
	public static void main(String args[]) {
		EventQueue.invokeLater(new Runnable() {
			public void run() {
				try {
					Control frame = new Control();
					frame.setVisible(true);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
	}

	/**
	 * Create the frame
	 */
	public Control() {
		super();
		getContentPane().setBackground(SystemColor.activeCaption);
		getContentPane().setLayout(null);
		setBounds(100, 100, 413, 472);
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		textArea = new JTextArea();
		textArea.setBounds(10, 10, 382, 388);
		getContentPane().add(textArea);
		textArea.setText(" Rover Movement Control \r\n\n" +
				  " Increase Speed	: Move the Throttle Button up \r\n" +
				  " Reduce Speed		: Move the Throttle Button Down \r\n" +
				  " Foward		: Joystick - Up\r\n" +
				  " Reverse		: Joystick - Down\r\n" +
				  " Left		: Joystick - Left\r\n" +
				  " Right		: Joystick - Right\r\n" +
				  "\r\n"+
				  "   Arm Control \r\n" +
				  " Button 1	: Start Screw \r\n" +
				  " Button 2	: Move Arm Down \r\n" +
				  " Button 3	: Move Arm UP \r\n" +
				  " Button 4	: Move Screw In \r\n" +
				  " Button 5	: Move Screw Out \r\n\n" +
				  "   Camera Control -> Arm & Movement (select Camera First) \r\n" +
				  " Button 6	: Tilt Up \r\n" +
				  " Button 7	: Tilt Down \r\n" +
				  " Button 8	: Pan Left \r\n" +
				  " Button 9	: Pan Right \r\n" +
				  " Button 11	: Take snap shots\r");
		
		
		
		final JButton exitButton = new JButton();
		exitButton.setText("EXIT");
		exitButton.setBounds(298, 404, 85, 25);
		exitButton.addActionListener(new ActionListener()
		{
			public void actionPerformed(ActionEvent e) {
				// TODO Auto-generated method stub
				//System.exit(0);
				setVisible(false);
			}
		});
		
		getContentPane().add(exitButton);

		
		
		//
	}

}
