package driveArmControl;

import java.awt.Color;
import java.awt.FlowLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;

public class DeployPanel extends JPanel {
	public int deployNum = 5;
	public static final String offString = "Deploy", onString = "Turning";
	public JButton deployButton[] = new JButton[deployNum];
	public LightPanel deployLight[] = new LightPanel[deployNum];
	
	private JPanel deploySet[] = new JPanel[deployNum];
	
	public DeployPanel(int deployNum) {
		if(deployNum >= 0) this.deployNum = deployNum;
		this.setLayout(new GridLayout(2,3));
		
		for(int i = 0; i < deployNum; i++) {
			deployButton[i] = new JButton(offString+ " " +(i+1));
			deployLight[i] = new LightPanel(20,20,Color.green, Color.yellow, Color.red);//2,1,0
			deploySet[i] = new JPanel();
			deploySet[i].setLayout(new FlowLayout());
			deploySet[i].add(deployLight[i]);
			deploySet[i].add(deployButton[i]);
			this.add(deploySet[i]);
		}
	}//constructor
	public DeployPanel() {
		this(5);
	}
	
	private class DeployActionListener implements ActionListener {
		
		@Override
		public void actionPerformed(ActionEvent ae) {
			String text = ((JButton)ae.getSource()).getText();
			int deployNum = Integer.parseInt(text.split(" ")[1]);//get number
			if( text.contains(offString) ) {
				
			} else if( text.contains(onString) ) {
				
			}	
		}//actionPerformed
		
	}//DeployActionListener class
	
	
	
	
	public static void main(String[] args) {
		JFrame test = new JFrame("test");
		DeployPanel dp = new DeployPanel();
		test.add(dp);
		test.pack();
		test.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		test.setVisible(true);
	}
}//DeployPanel class

