package gui.testing;

import gui.component.GUIComponent;

import java.awt.Color;
import java.awt.Graphics;

import javax.swing.JFrame;

public class ArmPicPanel extends GUIComponent {
	public static final int DEFAULT_WIDTH = 200;
	public static final int DEFAULT_HEIGHT = 200;
	public static final Color DEFAULT_BOX_COLOR = Color.black;
	
	private int pX, pY;
	private int baseWidth, baseHeight;
	private int[] armAngles;
	private int[] armLengths;
	private int canfieldWidth, canfieldHeight;
	private int[] canfields;
	public UpdateArmFunction updateArmFunction;
	
	public ArmPicPanel(int x, int y, float transparency,int[] armAngles, int[] canfields) {
		this(x,y,DEFAULT_WIDTH,DEFAULT_HEIGHT,transparency, armAngles, canfields);
	}
	public ArmPicPanel(int x, int y, int width, int height, float transparency, int[] armAngles,  int[] canfields) {
		super(x,y,width,height,transparency);
		this.armAngles = armAngles;
		this.canfields = canfields;
		armLengths = new int[]{25,50,50};
		
		pX = 130; pY = 130;
		baseWidth = 50; baseHeight = 50;
		canfieldWidth = 50; canfieldHeight = 50;

		updateArmFunction = new UpdateArmFunction(armAngles, canfields);
	}
	@Override
	public void paintBuffer( Graphics g ) {
		super.paintComponent(g);
		g.setColor(DEFAULT_BOX_COLOR);
		g.drawRect(0, 0, width-1, height-1);
		
		//build base, draw base angle
		g.drawRect(pX, pY, baseWidth, baseHeight);
		g.drawOval(pX, pY, baseWidth, baseHeight);
		g.drawLine(pX+baseWidth/2, pY+baseHeight/2, pX+baseWidth/2+(int)(armLengths[0]*Math.cos((armAngles[0]/180.0)*Math.PI)), pY+baseHeight/2-(int)(armLengths[0]*Math.sin((armAngles[0]/180.0)*Math.PI)));
		//draw arm part two
		g.drawLine(pX+baseWidth/2, pY, pX+baseWidth/2+(int)(armLengths[1]*Math.cos(((180-armAngles[1])/180.0)*Math.PI)), pY-(int)(armLengths[1]*Math.sin(((180-armAngles[1])/180.0)*Math.PI)));
		//draw arm part three
		g.drawLine(pX+baseWidth/2+(int)(armLengths[1]*Math.cos(((180-armAngles[1])/180.0)*Math.PI)), pY-(int)(armLengths[1]*Math.sin(((180-armAngles[1])/180.0)*Math.PI)), 
				pX+baseWidth/2+(int)(armLengths[1]*Math.cos(((180-armAngles[1])/180.0)*Math.PI))+(int)(armLengths[2]*Math.cos(((270-armAngles[2]-armAngles[1])/180.0)*Math.PI)), 
				pY-(int)(armLengths[1]*Math.sin(((180-armAngles[1])/180.0)*Math.PI))-(int)(armLengths[2]*Math.sin(((270-armAngles[2]-armAngles[1])/180.0)*Math.PI)));
		g.drawRect(pX+baseWidth/2+(int)(armLengths[1]*Math.cos(((180-armAngles[1])/180.0)*Math.PI))+(int)(armLengths[2]*Math.cos(((270-armAngles[2]-armAngles[1])/180.0)*Math.PI))-canfieldWidth, 
				pY-(int)(armLengths[1]*Math.sin(((180-armAngles[1])/180.0)*Math.PI))-(int)(armLengths[2]*Math.sin(((270-armAngles[2]-armAngles[1])/180.0)*Math.PI))-canfieldHeight/2,
				canfieldWidth, canfieldHeight);
		g.drawString("a: "+canfields[0], 
				pX+baseWidth/2+(int)(armLengths[1]*Math.cos(((180-armAngles[1])/180.0)*Math.PI))+(int)(armLengths[2]*Math.cos(((270-armAngles[2]-armAngles[1])/180.0)*Math.PI))-canfieldWidth+2, 
				pY-(int)(armLengths[1]*Math.sin(((180-armAngles[1])/180.0)*Math.PI))-(int)(armLengths[2]*Math.sin(((270-armAngles[2]-armAngles[1])/180.0)*Math.PI))-12);
		g.drawString("b: "+canfields[1], 
				pX+baseWidth/2+(int)(armLengths[1]*Math.cos(((180-armAngles[1])/180.0)*Math.PI))+(int)(armLengths[2]*Math.cos(((270-armAngles[2]-armAngles[1])/180.0)*Math.PI))-canfieldWidth+2, 
				pY-(int)(armLengths[1]*Math.sin(((180-armAngles[1])/180.0)*Math.PI))-(int)(armLengths[2]*Math.sin(((270-armAngles[2]-armAngles[1])/180.0)*Math.PI))+2);
		g.drawString("c: "+canfields[2], 
				pX+baseWidth/2+(int)(armLengths[1]*Math.cos(((180-armAngles[1])/180.0)*Math.PI))+(int)(armLengths[2]*Math.cos(((270-armAngles[2]-armAngles[1])/180.0)*Math.PI))-canfieldWidth+2, 
				pY-(int)(armLengths[1]*Math.sin(((180-armAngles[1])/180.0)*Math.PI))-(int)(armLengths[2]*Math.sin(((270-armAngles[2]-armAngles[1])/180.0)*Math.PI))+18);
	}
	public void updateDisplay() {
		this.repaint();
	}
	public class UpdateArmFunction extends UpdateFunction {
		int[] a, b, at,bt;
		//the values are different and the screen gets updated  
		
		public UpdateArmFunction(int[] a, int[] b) 
		{
			this.a = a;
			at = new int[a.length];
			System.arraycopy(a, 0, at, 0, a.length);
			this.b = b;
			bt = new int[b.length];
			System.arraycopy(b, 0, bt, 0, b.length);
		}
		
		public boolean checkValues() 
		{
			for(int i = 0; i < a.length; i++)
				if(at[i] != a[i]) return true;
			for(int i = 0; i < b.length; i++)
				if(bt[i] != b[i]) return true;
			return false;
		}
	
		public void doUpdate() {
			System.arraycopy(a, 0, at, 0, a.length);
			System.arraycopy(b, 0, bt, 0, b.length);
			updateDisplay();
		}
	} // end of update speed class
	public static void main(String[] args) {
		boolean v1 = true; 
		int A= 75;
		int B=100;
		int[] armAngles = new int[]{0,0,0};
		int[] canfields = new int[]{0,0,0};
		//-------------FRAME---------------
		
		JFrame frame = new JFrame(); 
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setTitle("Speed - test");
		//frame.setSize(800, 800);
		
		//-----------Main panel------------
		ArmPicPanel constructorCall = new ArmPicPanel(A,B,1f, armAngles, canfields);
		frame.add(constructorCall);
		frame.pack();
		frame.setVisible(true);
		do
		{
			if(armAngles[0] >= 360) {
				armAngles[0] = 0;
				armAngles[0] += 10;
			} else armAngles[0] += 10;
			if(armAngles[1] >= 180) {
				armAngles[1] = 0;
			} else armAngles[1] += 10;
			if(armAngles[2] >= 180) {
				armAngles[2] = 0;
			} else armAngles[2] += 10;
			if(canfields[0] >= 180) {
				canfields[0] = 0;
			} else canfields[0] += 10;
			canfields[1] = canfields[0];
			canfields[2] = canfields[0];
			constructorCall.updateDisplay();
			System.out.printf("%d %d\n", armAngles[0], armAngles[1]);
			try { Thread.sleep(1000);
			} catch (InterruptedException e) { }
		} while (v1);
		//v1 is a variable which is always true, so the loop never ends. 
	}//testing main
}
