/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package GUI;

import java.awt.Dimension;
import java.awt.FlowLayout;

import java.util.ArrayList;
import javax.swing.BoxLayout;
import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;
import javax.swing.JTextArea;
import javax.swing.ListSelectionModel;

/**
 * GPSPanel is bottom panel that displays added GPS coordinates.
 * @author Dennis Liu
 * YURT 2010-2011
 * --------------------------------
 * The panel is divided into 2 main sections:
 * 1. utilPane (left side)
 *	utilPane is further divided into top and bottom:
 *	top: GPS converter from (NAV27) to GPS <GPSConverter>
 *	bottom: adds the coordinates to the list. <GPSenter:degree/decimal(Panel,Label,TextArea),addGPSButton>
 * 2. GPSList (right side)
 *	<GPSScroll, GPSList, GPSModel>
 *	Contains a JList that displays the GPS coordinates. A JScrollPane nests it,
 *	the JList uses DefaultListModel to manage the coordinates. See tutorial on
 *	how to use JList on oracle.com for more info.
 * waypoints is what stores the coordinates.
 * --------------------------------
 * Hierarchy:
 *	GPSPanel
 *		utilPane (splitpane top/bottom) (left panel)
 *			GPSConverter (top subpanel)
 *			GPSEnter (bottom subpanel)
 *				degreePanel
 *					degree(Label/TextArea)
 *				decimalPanel
 *					decimal(Label/TextArea)
 *				addGPSButton
 *		GPSScroll (nesting for JList) (right panel)
 *			GPSList, GPSModel
 * --------------------------------
 */
public class GPSPanel extends JSplitPane {
	JScrollPane GPSScroll;
	JList GPSList;
	DefaultListModel GPSModel;

	JSplitPane utilPane;
	JPanel GPSConverter, GPSEnter, degreePanel, decimalPanel;
	JLabel degreeLabel, decimalLabel;
	JTextArea degreeTextArea, decimalTextArea;
	JButton addGPSButton;
	ArrayList<GPSCoord> waypoints = new ArrayList<GPSCoord>();

	public GPSPanel() {
		super();
		this.setOrientation(JSplitPane.HORIZONTAL_SPLIT);//makes the main split left/right
		this.setOneTouchExpandable(true);//make it so you can see just 1 side only
			//GPS converter panel
			GPSConverter = new JPanel();//placeholder
			/********************/
			//GPS enter panel
			GPSEnter = new JPanel();
			//the GPSEnter displays two label/textarea panels (for two fields)
			//so GPSEnter doesn't takes in degree/decimalPanel, in top/bottom order.
			GPSEnter.setLayout( new BoxLayout(GPSEnter, BoxLayout.PAGE_AXIS));
				//degreePanel
				degreePanel = new JPanel();//make degreePanel
				degreePanel.setLayout( new FlowLayout());
					degreeLabel = new JLabel("DEG:");//label for field
					degreeTextArea = new JTextArea();
					degreeTextArea.setPreferredSize( new Dimension(100, 14));//set size
				degreePanel.add(degreeLabel);//add label/text together
				degreePanel.add(degreeTextArea);
				//decimalPanel: same as above.
				decimalPanel = new JPanel();
				decimalPanel.setLayout( new FlowLayout());
					decimalLabel = new JLabel("DECI");
					decimalTextArea = new JTextArea();
					decimalTextArea.setPreferredSize( new Dimension(100, 14));
				decimalPanel.add(decimalLabel);
				decimalPanel.add(decimalTextArea);
				addGPSButton = new JButton("Add");//add GPS button
			GPSEnter.add(degreePanel);//put the bottom panel together
			GPSEnter.add(decimalPanel);
			GPSEnter.add(addGPSButton);
		//finalize and create the left main panel
		utilPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT, true, GPSConverter, GPSEnter);
		utilPane.setOneTouchExpandable(true);
		this.setLeftComponent(utilPane);
			//create list on right side
			GPSModel = new DefaultListModel();//model to organize list
			GPSList = new JList(GPSModel);//create list
			GPSList.setSelectionMode(ListSelectionModel.MULTIPLE_INTERVAL_SELECTION);
			GPSList.setLayoutOrientation(JList.VERTICAL_WRAP);
			GPSList.setVisibleRowCount(-1);//let computer set the visible row count
			GPSScroll = new JScrollPane(GPSList);//let computer set scroll stuff
		this.setRightComponent(GPSScroll);//add list to right side
	}//GPSPanel constructor

	/*
	 * format for GPS coordinates.
	 * <DEGREE><DECIMAL_DEGREE>
	 */
	public class GPSCoord {
		double degree, decimal;
		public GPSCoord(double degree, double decimal) {
			this.degree = degree;
			this.decimal = decimal;
		}
	}//GPSCoord class
}//RoverTiltPanel class
