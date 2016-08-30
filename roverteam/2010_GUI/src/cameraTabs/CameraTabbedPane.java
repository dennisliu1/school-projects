/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package cameraTabs;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.Component;
import java.awt.Dimension;

import java.util.ArrayList;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTabbedPane;

/**
 * The main tabbed panel that allows one to add cameras and display them on screen.
 * Under each tab is one split pane. The top half is the video, and the bottom half
 * is adding the cameras.
 * To add more tabs, click the right most tab.
 * To add a camera, enter its ip address and port number, and select what type it is.
 * then press start.
 * @author Dennis Liu
 * YURT 2010-2011
 * --------------------------------
 * I didn't comment AddCameraTabComponent, but basically it adds a tab header that
 * adds new tabs. It isn't commented because Saurabh wants to use FocusListener
 * instead ...? IDK w/e so basically it isn't finalized.
 * --------------------------------
 * Hierarchy:
 *	CameraTabPanel (Tabpane)
 *		tabLabels[guiNumTabs] (tab labels)
 *		mainPanel[guiNumTabs] (tab panels)
 *			CameraDisplayPanel
 * --------------------------------
 * Variables:
 * guiNumTabs	- number of tabs to use
 * --------------------------------
 */
public class CameraTabbedPane extends JTabbedPane {
	int guiNumTabs;
	ArrayList<CameraDisplayPanel> mainPanel = new ArrayList<CameraDisplayPanel>();
	JPanel addNewTabButtons;
	ArrayList<JLabel> tabLabels = new ArrayList<JLabel>();
	
	JTabbedPane self = this;

	public CameraTabbedPane(int guiNumTabs) {
		super();//constructor for JTabbedPane
		this.guiNumTabs = guiNumTabs;//save guiNumTabs
		
		tabLabels.add( new JLabel("Add 1"));//first two tabs saved are the addition ones.
		tabLabels.add( new JLabel("Add 4"));//but both are added to the pane LAST
		
		//create labels for tabs to use. Note that one can simplify this by ignoring tablabels
		//and just using addTabs to add the panels.
		// default 4 camera panels
		for( int i = 0; i < guiNumTabs; i++ ) {
			mainPanel.add( new CameraDisplayPanel(1,"","") );//create CameraDisplayPanel
			tabLabels.add( new JLabel("Tab "+(i+1)));
			this.add(mainPanel.get(i));//add to tab
			this.setTabComponentAt(i, tabLabels.get(i+2));//add label to match tab
		}
		//addNewTabButtons = new AddCameraTabComponent(this, guiNumTabs);
		//add new tab 'tab's
		this.add( new JLabel("n/a") );//don't matter
		this.setTabComponentAt(guiNumTabs, tabLabels.get(0));
		this.add( new JLabel("n/a2") );
		this.setTabComponentAt(guiNumTabs+1, tabLabels.get(1));
		//tabs scroll horizontal list when overflow screen
		this.setTabLayoutPolicy( JTabbedPane.SCROLL_TAB_LAYOUT);
		//-------------------------------------------------------------
		CurrentTabT thread = new CurrentTabT();//run thread to work with tab changes
		thread.start();
	}// CameraTabPanel constructor
	//----------------------		SUB CLASSES		--------------------------
	private class CurrentTabT extends Thread {
		int curComponentNum = 0;
		CameraDisplayPanel curComponent = null;
		
		public CurrentTabT() {
			curComponentNum = self.getSelectedIndex();
			curComponent = (CameraDisplayPanel) self.getSelectedComponent();
		}
		
		public void run() {
			while(true) {
				
				if(self.getSelectedIndex() == guiNumTabs) {//add "1 camera" tab
					self.setSelectedComponent(addCameraTab(1));
				} else if(self.getSelectedIndex() == (guiNumTabs + 1) ) {//add "4 camera" tab
					self.setSelectedComponent(addCameraTab(4));
				} else if(curComponentNum != self.getSelectedIndex()) {//check if tab is changed
					System.out.printf("%s->%s=%s\n", curComponentNum, self.getSelectedIndex(), self.getSelectedComponent());
					try {
						curComponent.stopAllVideo();
					} catch(Exception e) {}
					curComponent = (CameraDisplayPanel) self.getSelectedComponent();
					curComponentNum = self.getSelectedIndex();
					
				}
				try { Thread.sleep(100); } 
				catch (InterruptedException e) { e.printStackTrace(); }
			}
		}//run
		
		//adds a new tab at the end of the tabbedPane, with "num" number of cameras in the tab
		public CameraDisplayPanel addCameraTab(int num) {
			self.removeTabAt(guiNumTabs+1);
			self.removeTabAt(guiNumTabs);
				mainPanel.add( new CameraDisplayPanel(num,"","") );//create CameraDisplayPanel
				if(num == 1) tabLabels.add( new JLabel("Tab "+(guiNumTabs+1)));
				else if( num > 1) tabLabels.add( new JLabel("T"+num+"b "+(guiNumTabs+1)));
				self.add(mainPanel.get(guiNumTabs));//add to tab
				self.setTabComponentAt(guiNumTabs, tabLabels.get(guiNumTabs+2));//add label to match tab
			self.add( new JLabel("n/a") );
			self.setTabComponentAt(guiNumTabs+1, tabLabels.get(0));
			self.add( new JLabel("n/a2") );
			self.setTabComponentAt(guiNumTabs+2, tabLabels.get(1));
			return mainPanel.get(guiNumTabs++);
		}// addCameraTab method
	}//CurrentTabT class
}//CameraTabPanel class
