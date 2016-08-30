/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package cameraTabs;

import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JPanel;

/**
 *
 * @author yurt
 */
public class USBSplitterGUI extends JPanel {
	public CameraInitPanel sourceInitPanel, splitInitPanels[];
	public int numPanels = 4;
	USBSplitter splitter;
	
	public USBSplitterGUI() {
		this.setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
		sourceInitPanel = new CameraInitPanel("localhost", "4000", "400", "USB");
		this.add(sourceInitPanel);
		splitInitPanels = new CameraInitPanel[numPanels];
		for(int i = 0; i < numPanels; i++) {
			splitInitPanels[i] = new CameraInitPanel("localhost", ""+(5000+i), "400", "USB");
			this.add(splitInitPanels[i]);
		}
		
		splitter = new USBSplitter(sourceInitPanel, splitInitPanels);
		sourceInitPanel.startCamB.addActionListener(splitter.sourceActionListener);
		sourceInitPanel.stopCamB.addActionListener(splitter.sourceActionListener);
		for(int i = 0; i < splitInitPanels.length; i++) {
			splitInitPanels[i].startCamB.addActionListener(splitter.destActionListener[i]);
			splitInitPanels[i].stopCamB.addActionListener(splitter.destActionListener[i]);
		}
	}
	
	public static void main(String[] args) {
		JFrame frame = new JFrame();
			USBSplitterGUI initPanels = new USBSplitterGUI();
			frame.add(initPanels);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//		frame.setSize(500, 500);
		frame.pack();
		frame.setVisible(true);
	}
}
