
import java.applet.*;
import java.awt.*;
import java.awt.event.*;
import java.io.*;
import java.util.*;
/**
 * Date   : May 29,2009
 * Author : Dennis Liu
 * Class  : ICS4M1-01
 * highly recommended to be run on netbeans.
 * A polygon program for grade 8 students, to help them study polygons.
 * User clicks points on the grid to create points, and finishes the shape by clicking the starting location.
 * The name, area, angles, lengths and positions are displayed. The user can also
 * save and load polygon shapes (.txt format). Annotation is "x=/y=".
 * This focuses the applet window so the user can use keyboard shortcuts.
 * Angles are in radians. When saving and loading, one simply types the name.
 * Do not enter the format or location when typing the name for save/load.
 * location of the saved files are specified by the variable dirLoc. FIles are all .txt format.
 * coordinates are still relative to the applet size, so it is compatible with off-limits shapes (on purpose).
 * For user friendliness, do not change the settings to below the limits.
 * Also, reflex angles are not given as this is a polygon generator, not a general shape generator (should not have >180 degrees)
 *
 * Shortcuts: comma=absolute mode period=polar mode minus=ortho mode equal=reset
 * open bracket=save polygon close bracket=load polygon slash=undo point
 */
public class LIUDENmajorproject extends Applet
	implements KeyListener, MouseListener, MouseMotionListener  {
	/**directory location*/			String dirLoc="C:\\School\\Project\\src\\project\\";
	/**cursor size:10*/				int curSize = 10;
	/**left X offset for grid:170*/	int minX=170;
	/**right X offset for grid:0*/	int MaxX=0;
	/**Top Y offset for grid:30*/	int minY=30;
	/**Bottom Y offset for grid:30*/int maxY=30;
	/**X size of applet:700*/		int appX=1000;
	/**Y size of applet:700*/		int appY=650;
	/**pts required to touch end*/  int offSet=3;

	/**reads the key pressed by user*/String keyReader = "";
	/**width of the applet*/int width;/**height of the applet*/int height;
	/**x mouse position*/int mouseX;/**y mouse position*/ int mouseY;
	/**number of points used. NOT Zero Based*/int numPoints = 0;
	/**area of the polygon*/double area = 0; 
	/**angle used for calculating orthographic line*/double numOne = 0;
	/**holds the second number typed for absolute or polar point*/double numTwo = 0;
	/**perimeter of polygon*/double totalLength = 0;/**total angles*/double totalAngle = 0;
	/**x coordinate of points*/double[]xArray = new double[0];
	/**y coordinate of points*/double[]yArray = new double[0];
	/**length of the ilnes. Lines are stored in order of the points lArray[0]=(P1->P0)*/double[]lArray = new double[0];
	/**angles of the points. In radians*/double[]angleArray = new double[0];
	/**backbuffer used to stop flashing*/Image backbuffer;
	/**backgroundd image. Used with backbuffer*/Graphics backg;
	/**polygon shape completed*/boolean polyDone = false;
	/**button is pressed or not*/boolean isButtonPressed = false;
	/**in main screen or not*/boolean mainScreen = true;
	/**absolute mode*/boolean absCoord = false;/**polar mode*/boolean polarCoord = false;
	/**orthographic mode*/boolean orthoCoord = false;/**done typing first number*/boolean numDone = false;
	/**name of the polygon*/String polyName = "";
	/**text name,used for save/load text files*/String txtName;
	/**load text file*/boolean loadPoly;/**save text file*/boolean savePoly;
	/*---Constants---
	 * dirLoc   = directory location
	 * curSize  = cursor size
	 * min/maxX = minimum/maximum X size
	 * min/maxY = minimum/maximum Y size
	 * appX/Y   = size of the applet
	 * offSet   = pts required to touch end
	 *---int---
	 * width     = width of the applet
	 * height    = height of the applet
	 * mouseX    = x mouse position
	 * mouseY    = y mouse position
	 * numPoints = number of points used. NOT Zero Based
	 * area      = area of the polygon
	 *---double---
	 * numOne        = holds the first number typed for absolute or polar point
	 * numTwo        = holds the second number typed for absolute or polar point
	 * totalLength = perimeter of polygon
	 * totalAngle  = total angle amount
	 *---double[]---
	 * xArray[]     = x coordinate of points
	 * yArray[]     = y coordinate of points
	 * lArray[]     = length of the ilnes. Lines are stored in order of the points lArray[0]=(P1->P0)
	 * angleArray[] = angles of the points. In radians
	 *---Image---
	 * backbuffer = backbuffer used to stop flashing
	 *---Graphic---
	 * backg      = backgroundd image. Used with backbuffer
	 *---boolean---
	 * polyDone   = polygon shape completed
	 * absCoord   = absolute mode
	 * polarCoord = polar mode
	 * orthoCoord = orthographic mode
	 * numDone    = done typing first number
	 * polyName   = name of the polygon
	 * loadPoly   = load polygon mode
	 * savePoly   = save polygon mode
	 *---String---
	 * keyReader = reads the key pressed by user
	 * polyname  = name of polygon
	 * dirLoc    = directory location
	 * txtname   = text file name. Used when saving/loading files*/
	/**graphing program*/
	@Override
	public void init() {
		resize(appX,appY);//resizes window to correct size
		width = getSize().width;//gets size of applet
		height = getSize().height;
		setBackground(Color.black);
		mouseX = width/2;//default position of mouse
		mouseY = height/2;
		backbuffer = createImage(width,height);//creates backbuffer
		backg = backbuffer.getGraphics();//defined into a graphic
		addKeyListener( this );//used for key and mouse reading
		addMouseListener( this );
		addMouseMotionListener( this );
		setFocusable(true);//focuses window
		redraw();//redraw window
	}

	public void keyPressed( KeyEvent e ) { }//required by default
	public void keyReleased( KeyEvent e ) { }
	public void keyTyped( KeyEvent e ) {
		/**gets the character version of key pressed by user*/char c = e.getKeyChar();
		if (c == KeyEvent.VK_SPACE) {//spacebar
			keyReader ="";//reset what is typed
			backg.clearRect(400,0, width,15);//clear what is typed
		}
		if (absCoord||polarCoord) {//when in either absolute or polar mode
			if(c>=KeyEvent.VK_0 && c<=KeyEvent.VK_9) {//reads 0-9 number keys
				keyReader += c;//adds to string to be read
			}
			if (c==KeyEvent.VK_ENTER) {//enter
				if (!numDone) {//first number
					numOne = Integer.parseInt(keyReader);//changes string to int
					numDone = true;//first number done
					keyReader ="";//string resetted
					backg.clearRect(400,0, width,30);//clear what is typed
				} else {//second number
					numTwo = Integer.parseInt(keyReader);//changes string to int
					numDone = false;//default for next use
					if(absCoord)absPt(numOne,numTwo);//calculates point
					else if(polarCoord&&numPoints>0)polarPt(xArray,yArray,numOne,numTwo);//polar works only if not the first point
					keyReader = "";//string resetted
					backg.clearRect(0,0,width,height);//clear what is typed
				}
			}
			if(c==KeyEvent.VK_BACK_SPACE) {//backspace
				if(keyReader.length()>0)//does not work when nothing typed
					keyReader = keyReader.substring(0, keyReader.length()-1);//shortens string by one
				backg.clearRect(400,0, width,15);//clears box
			}
			redraw();
		}//if(absCoord||polarCoord)
		if (c == KeyEvent.VK_COMMA) {//comma ,
			if (absCoord) absCoord = false;//turn on/off absolute mode
			else {
				keyReader ="";//default what is typed
                numDone=false;//default numbers
                numOne=0;
                numTwo=0;
				noMode();
				if(!polyDone)absCoord = true;//does not work when done polygon
                backg.clearRect(0,0,width,height);
                redraw();
			}
		}//comma
		if (c == KeyEvent.VK_PERIOD) {//period .
			if (polarCoord) polarCoord = false;//turn on/off polar mode
			else {
				keyReader="";//default what is typed
                numDone=false;
                numOne=0;
                numTwo=0;
				noMode();
				if(!polyDone)polarCoord = true;
                backg.clearRect(0,0, width, height);
                redraw();
			}
		}//period
		if (c == KeyEvent.VK_MINUS) {//minus -
			if (orthoCoord) orthoCoord = false;//turn on/off ortho mode
			else if(!polyDone)orthoCoord = true;
		}//minus
		if (c == KeyEvent.VK_EQUALS) {//equal =
			resetAll();//redefault everything
		}//equals
		if(c == KeyEvent.VK_SLASH&&numPoints>0) {//slash(/)and when there is a point
			if(polyDone) {//undefines variables used when polygon is done
				area = 0;
				angleArray = new double[0];
				polyName = "";
				polyDone = false;//polygon is now not done
				savePoly=false;
				keyReader="";
			}
			numPoints -= 1;//lower by one point
			xArray = addArray(xArray,numPoints);//lower one point
			yArray = addArray(yArray,numPoints);
			lArray = addArray(lArray,numPoints);
			backg.clearRect(0,0,width, height);//redraw
			redraw();
		}//slash
		if(savePoly&&polyDone) {//saves only if polygon is done
			if(c==KeyEvent.VK_BACK_SPACE) {//backspace
				if(keyReader.length()>0)//does not work when nothing typed
					keyReader = keyReader.substring(0, keyReader.length()-1);//shortens string by one
				backg.clearRect(400,0, width,15);//clears box
				redraw();
			}
			if (c==KeyEvent.VK_ENTER) {//done typing name
				try {
					numPoints=0;//reset numPoints
					txtName=keyReader;
					String[] outArray = new String[xArray.length+yArray.length];//temporary string array to save to file
					for(int a=0;a<outArray.length;a++) {//create string array
						//checks for odd/even number-odd is y num,x is even num
						if(a%2==0)outArray[a]="x="+Double.toString(xArray[a-numPoints]);//even num
						else {
							numPoints++;//next set of numbers
							outArray[a]="y="+Double.toString(yArray[a-numPoints]);
						}//odd num
					}
					createFile(dirLoc+txtName+".txt");//creates file
					txtOutput(dirLoc+txtName+".txt",outArray);//outputs file
					//creates the file and adds the info into it;always overwrites file
					keyReader="";//reset strings for next use
					txtName="";
					savePoly=false;//redefault
				} catch(Exception f) {//outputs the error
					resetAll();
					keyReader = ""+f;//file failed, reset everything
				}
			}
			else if(c!=KeyEvent.VK_OPEN_BRACKET && c!=KeyEvent.CHAR_UNDEFINED) {
				//used to type the named; special characters should not be used!
				//open bracket is checked in cse user turns off savePoly
				keyReader += c;
			}
		}//if(savePoly&&polyDone)
		if(c == KeyEvent.VK_OPEN_BRACKET) {//open bracket [
			if (savePoly) {
				keyReader="";
				savePoly = false;
			}//turn on/off polar mode
			else {
				keyReader="";//default what is typed
				noMode();
				savePoly = true;//only one mode on at once
                backg.clearRect(0,0, width, height);//refresh
                redraw();
			}
		}
		if(loadPoly) {
			if(c==KeyEvent.VK_BACK_SPACE) {//backspace
				if(keyReader.length()>0)//does not work when nothing typed
					keyReader = keyReader.substring(0, keyReader.length()-1);//shortens string by one
				backg.clearRect(400,0, width,15);//clears box
				redraw();
			}
			else if (c==KeyEvent.VK_ENTER) {//enter
				numPoints=0;//reset index
				txtName=keyReader;
				try {
					String[] txtArray = txtDump(dirLoc+txtName+".txt");//dumps text file into memory
					xArray=new double[txtArray.length/2];//defines arrays for new polygon
					yArray=new double[txtArray.length/2];
					lArray=new double[txtArray.length/2];
					for(int a=0;a<txtArray.length;a++) {
						if(a%2==0)xArray[a-numPoints]=Double.parseDouble(txtArray[a].substring(2));//even num
						else {
							numPoints++;//next object
							yArray[a-numPoints]=Double.parseDouble(txtArray[a].substring(2));
							if(!(numPoints<2))//adds length only if there are at least two points
								lArray[numPoints-2] = (int)hLength(xArray[numPoints-1]-xArray[numPoints-2],yArray[numPoints-1]-yArray[numPoints-2]);
						}//odd num
					}
					donePoly();
					keyReader="";//defaults for next use
					txtName="";
					loadPoly=false;
					backg.clearRect(0,0,width,height);//refresh
					redraw();
				} catch(Exception f) {//outputs the error
					resetAll();
					keyReader = ""+f;//file failed, reset everything and output error
				}
			}
			else if(c!=KeyEvent.VK_CLOSE_BRACKET && c!=KeyEvent.CHAR_UNDEFINED) {
				//used to type the named; special characters should not be used!
				//open bracket is checked in cse user turns off savePoly
				keyReader += c;
			}
		}
		if(c == KeyEvent.VK_CLOSE_BRACKET) {//close bracket ]
			if (loadPoly) {
				keyReader="";
				loadPoly = false;
			}//turn on/off polar mode
			else {
				keyReader="";//default what is typed
				noMode();
				loadPoly = true;//only one mode on at once
                backg.clearRect(0,0, width, height);
                redraw();
			}
		}
		backg.clearRect(0,670,width, height);//clear bottom of the applet
		redraw();
		repaint();
		e.consume();
	}//keypressed()

	public void mouseEntered( MouseEvent e ) { }
	public void mouseExited( MouseEvent e ) { }
	public void mousePressed( MouseEvent e ) { }
	public void mouseReleased( MouseEvent e ) { }
	public void mouseClicked( MouseEvent e ) {
		if ((numPoints >= 3)&&((e.getX() >= xArray[0]-offSet)&&(e.getX() <= xArray[0]+offSet))&&
				((e.getY() >= yArray[0]-offSet)&&(e.getY() <= yArray[0]+offSet)))  {
			//click within 5 units of start point and an actual polygon (smallest is triangle)
			donePoly();
			backg.clearRect(0,0,width,height);//clears screen
		}
		if(!polyDone){//not done polygon
			mouseX = e.getX();//gets position of mouse
			mouseY = e.getY();
			if (orthoCoord&&numPoints>0) {//checks for ortho
				if (checkOrtho(xArray,yArray,e.getX(),e.getY())) mouseY = (int)yArray[yArray.length-1];
				else mouseX = (int)xArray[xArray.length-1];//redefines x/y for ortho
			}
			absPt(mouseX,mouseY);
			backg.clearRect(0,0,width,height);//redraws shape
		}
		redraw();
		repaint();
		e.consume();
	}//mouseClicked()
	public void mouseDragged( MouseEvent e ) {  // called during motion with buttons down
		mouseX = e.getX();//get mouse coordinates
		mouseY = e.getY();
		showInfo();//shows info at the bottom of applet
		redraw();
		repaint();
		e.consume();
	}
	public void mouseMoved( MouseEvent e ) {// called during motion when no buttons are down
		mouseX = e.getX();//get mouse coordinates
		mouseY = e.getY();
		if (orthoCoord&&numPoints>0) {//checks for ortho
			if (checkOrtho(xArray,yArray,e.getX(),e.getY())) mouseY = (int)yArray[yArray.length-1];
			else mouseX = (int)xArray[xArray.length-1];//redefines x/y for ortho
		}
		showInfo();//shows info at the bottom of applet
		backg.clearRect(0,0,width, height);//redraw screen
		if (polyDone) {
			mouseX = e.getX();//clears ortho
			mouseY = e.getY();
		}
		backg.drawLine(mouseX,mouseY,mouseX, mouseY-curSize);//draws cursor
		backg.drawLine(mouseX,mouseY,mouseX-curSize,mouseY);
		backg.drawLine(mouseX,mouseY,mouseX, mouseY+curSize);
		backg.drawLine(mouseX,mouseY,mouseX+curSize,mouseY);
		redraw();
		repaint();
		e.consume();
	}
	@Override
	public void paint( Graphics g ) {
		update(g);//used for repaint; sent to update method
	}
	@Override
	public void update (Graphics g) {
		getToolkit().sync();//syncs graphics state (used for windows)
		g.drawImage( backbuffer, 0, 0, this );//redraws backbuffer
	}
	/**
	 * redraws applet. Used to extend update
	 */
	public void redraw() {
		backg.setColor( Color.green);//change color to green
		backg.drawString(Integer.toString(numPoints)+" Points :",minX,height-maxY+15);//shows number of points
		backg.drawString("Length    X/Y Coord    Angle(dgr)",5,15);//draws legend
		backg.drawString(keyReader,450,height-maxY+15);//display user typed string
		backg.setColor( Color.white);
		backg.drawLine(minX,minY, width,minY);//draws limits for polygon area;top
		backg.drawLine(minX,minY, minX,height-maxY);//left
		backg.drawLine(minX,height-maxY, width,height-maxY);//bottom
		backg.drawLine(width-1, maxY, width-1, height-maxY);//right;offset of -1 so it shows on screen
		backg.setColor( Color.green);
		for (int a = 0; a<=xArray.length-2;a++) {//draws the shape
			backg.drawLine((int)xArray[a],(int)yArray[a],(int)xArray[a+1],(int)yArray[a+1]);
		}

		String showStr;//used for showing
		for (int a = 0; a<=xArray.length-1;a++) {//loops for all points
			showStr = Double.toString(xArray[a])+"-"+Double.toString(yArray[a]);
			backg.drawString(showStr,55,35+a*16);//displays coordinates
			showStr = Double.toString(lArray[a]);
			backg.drawString(showStr,10,35+a*16);//displays lengths
		}

		if(absCoord||polarCoord||orthoCoord) showModes();//display modes if active
		if(absCoord) {//shows what number to type when in absolute
			if (numDone) backg.drawString("         y coord: ",380,height-maxY+15);
			else backg.drawString("         x coord: ",380,height-maxY+15);
		}//text to display depending on what is typed
		if(polarCoord) {//shows what number to type when in polar
			if (numDone) backg.drawString("      angle(dgr): ",380,height-maxY+15);
			else backg.drawString("          length: ",380,height-maxY+15);
		}
		if(savePoly) backg.drawString("(done)save shp: ",360,height-maxY+15);//for save/load modes
		if(loadPoly) backg.drawString("  load shape: ",380,height-maxY+15);

		if (polyDone) {//show if polgyon is done
			absCoord=false;
			orthoCoord=false;
			polarCoord=false;
			totalLength = 0;totalAngle = 0;//calculates total angle and length

			backg.drawString(getName(numPoints),230,height-15);//display name
			backg.drawLine((int)xArray[0],(int)yArray[0],(int)xArray[xArray.length-1],(int)yArray[yArray.length-1]);
			//draws the last line connect last point to first point
			
			for (int a = 0; a<=angleArray.length-1;a++) {//loops for all angles and lengths
				showStr = Double.toString(roundNum(Math.toDegrees(angleArray[a]),2));
				backg.drawString(showStr,130,35+a*16);//display angle
				totalAngle += angleArray[a];//adds angles and lengths to total
				totalLength += lArray[a];
			}
			showStr = Double.toString(area);
			backg.drawString("Area: "+showStr,minX+380,20);//show area
			showStr = Double.toString(roundNum(Math.toDegrees(totalAngle),2));
			backg.drawString("Total Angle: "+showStr,minX+180,20);//show total angle
			showStr = Double.toString(totalLength);
			backg.drawString("Total Length: "+showStr,minX+20,20);//show total length
		}
		if(numPoints>0){//display color for points
			backg.setColor(Color.red);//first point is red
			backg.drawArc((int)xArray[0]-2,(int)yArray[0]-2,4,4,0,360);//draw for first point
			backg.setColor(Color.orange);
			for (int a = 1; a<=xArray.length-1;a++) {//all other points are orange
				backg.drawArc((int)xArray[a]-2,(int)yArray[a]-2,4,4,0,360);
			}
			backg.setColor( Color.green);//change back to green
		}
		checkLimits();//check if line is out of limits
		if (numPoints!=0&&!polyDone)backg.drawLine((int)xArray[xArray.length-1],(int)yArray[yArray.length-1], mouseX,mouseY);
		//display line to cursor
	}//redraw()
	/**
	 * shows information and shortcuts in the bottom section of applet
	 */
	public void showInfo() {
		showStatus("("+mouseX+","+mouseY+")"+" " +"(Absolute Coord: ' )"+
				"(Polar Coord: . )"+"(Ortho: - )"+"(reset = )"
				+"(clear: space )"+"(save shape: [ )"+"(load shape: ] )"+"(undo: / )");
	}
	/**
	 * displays information when in different modes
	 */
	public void showModes() {
		if (absCoord) backg.drawString("Absolute",5,height-15);//absolute
		if(numDone&&absCoord) backg.drawString(Double.toString(numOne),5,height-30);//shows first number if present
		if (polarCoord) backg.drawString("polar",60,height-15);//polar
		if(numDone&&polarCoord) backg.drawString(Double.toString(numOne),60,height-30);//shows second number if present
		if (orthoCoord) backg.drawString("Ortho",95,height-15);//ortho
	}
	/**
	 * finishes off polygon and calculates angles and area
	 */
	public void donePoly() {
		polyDone = true;//polygon is done
		lArray = addArray(lArray,numPoints);//connects final line
		lArray[lArray.length-1] = (int)hLength(xArray[numPoints-1]-xArray[0],yArray[numPoints-1]-yArray[0]);//gets length
		area = polygonArea(xArray,yArray);//gets area
		angleArray = getAngles(xArray,yArray,lArray);//gets angles
	}
	public void noMode() {
		absCoord=false;
		polarCoord=false;
		savePoly=false;
		loadPoly=false;
	}
	/**
	 * reset all variables to default
	 */
	public void resetAll() {
		polyDone = false;//defaults everything
		numPoints = 0;
		area = 0;
		xArray = new double[0];
		yArray = new double[0];
		lArray = new double[0];
		angleArray = new double[0];
		polyName="";
		keyReader="";
		noMode();//clears modes
		backg.clearRect(0,0,width, height);//redraws shape
	}
	/**
	 * adds point to array
	 * @param x x component of point
	 * @param y y component of point
	 */
	public void absPt (double x,double y) {
		if(x<170) x = 170;
			else if (x>width) x=width;
		if(y<30) y = 30;
			else if (y>height-30) y=height-30;
		numPoints++;//next point
		xArray = addArray(xArray,numPoints);//adds one to array
		yArray = addArray(yArray,numPoints);
		lArray = addArray(lArray,numPoints);
		xArray[xArray.length-1] = x;//adds point
		yArray[yArray.length-1] = y;
		if(!(xArray.length<2))//adds length only if there are at least two points
			lArray[numPoints-2] = (int)hLength(xArray[numPoints-1]-xArray[numPoints-2],yArray[numPoints-1]-yArray[numPoints-2]);
	}
	/**
	 * calculates polar coordinate
	 * @param distance distance specified from last point
	 * @param angle angle used-zero degrees is to the right (east on a compass)
	 */
	public void polarPt(double[] xArray,double[] yArray,double distance,double angle) {
		double x=1, y=-1;//calculate the x/y coordinates based on the distance
		x *= Math.floor(distance*Math.cos(Math.toRadians(angle)));//numbers are floored because points are integers
		y *= Math.floor(distance*Math.sin(Math.toRadians(angle)));
		absPt(xArray[xArray.length-1]+x,yArray[yArray.length-1]+y);//creates the point
	}
	/**
	 * check limits of cursor
	 */
	public void checkLimits() {
		if(mouseX<170) mouseX = 170;//resets lines to fit x,y limits
			else if (mouseX>width) mouseX=width;
		if(mouseY<30) mouseY = 30;
			else if (mouseY>height-30) mouseY=height-30;
	}
	/**
	 * gets the name of the polygon
	 * @param numPoints = number of angles polygon has
	 * @return
	 */
	public static String getName(int numPoints) {
		String temp="";//temp string to store name
		if(numPoints<20)//less than 20 points
		switch(numPoints) {//gets one name
			case(3): temp = "Triangle"; break;
			case(4): temp = "Quadrilateral"; break;
			case(5): temp = "Pentagon"; break;
			case(6): temp = "Hexagon"; break;
			case(7): temp = "Heptagon"; break;
			case(8): temp = "Octagon"; break;
			case(9): temp = "Nonagon"; break;
			case(10): temp = "Decagon"; break;
			case(11): temp = "Hendecagon"; break;
			case(12): temp = "Dodecagon"; break;
			case(13): temp = "Triskaidecagon"; break;
			case(14): temp = "Tetrakaidecagon"; break;
			case(15): temp = "Pendedecagon"; break;
			case(16): temp = "Hexdecagon"; break;
			case(17): temp = "Heptdecagon"; break;
			case(18): temp = "Octdecagon"; break;
			case(19): temp = "Enneadecagon"; break;
			case(20): temp = "Icosagon"; break;
		} else {//higher than 20 points,name is taken from prefix+suffix
			int digitTen = (int)Math.floor(numPoints/10),digitOne = numPoints%10;
			//gets tens and ones digits
			switch(digitTen) {//prefix (tens digit)
				case(2): temp += "Icosikai"; break;
				case(3): temp += "Triacontakai"; break;
				case(4): temp += "Tetracontakai"; break;
				case(5): temp += "Pentacontakai"; break;
				case(6): temp += "Hexacontakai"; break;
				case(7): temp += "Heptacontakai"; break;
				case(8): temp += "Octacontakai"; break;
				case(9): temp += "Enneacontakai"; break;
			}
			switch(digitOne) {//suffix (ones digit)
				case(1): temp += "henagon"; break;
				case(2): temp += "digon"; break;
				case(3): temp += "triaon"; break;
				case(4): temp += "tetragon"; break;
				case(5): temp += "pentagon"; break;
				case(6): temp += "hexagon"; break;
				case(7): temp += "heptagon"; break;
				case(8): temp += "octagon"; break;
				case(9): temp += "enneagon"; break;
			}
		}
		return temp;//returns name
	}//getName()
	/**
	 * increases array size by an amount
	 * @param array array to be changed
	 * @param numPoints array size wanted returned
	 * @return array with new length
	 */
	public static double[] addArray(double[] array,int numPoints) {
		double[] tArray = array;//copy array to temporary var
		array = new double[numPoints];//redefines array with new size
		for(int a = 0; a< array.length-1; a++) {//loops for new array length (so it can handle smaller array size too)
			array[a] = tArray[a];//copys info back to array
		}
		if(array.length>0&&array.length<tArray.length) array[array.length-1] = tArray[array.length-1];
		//checks for last point
		return array;
	}
	/**
	 * checks for which line to show ortho (horizontal or vertical)
	 * @param curx
	 * @param cury
	 * @return boolean
	 */
	public static boolean checkOrtho(double[] xArray,double[] yArray,double curx,double cury) {
		double x = curx - xArray[xArray.length-1];//gets length of x
		double y = cury - yArray[yArray.length-1];//gets length of y
		double orthoAngle = y/x;//finds slope
		if ((orthoAngle < 1&&orthoAngle > 0)||(orthoAngle > -1&&orthoAngle<0)) return true;
		else return false;//checks which quardrant it is in and shows correct line
	}
	/**
	 * gets the polygon area. Uses cross product to find area.
	 * x1 x2 x3 x4...xN xN+1
	 *   X  X  X       X
	 * y1 y2 y3 y4...yN yN+1
	 * Area = (1/2)*(x1y2-x2y1+x2y3-x3y2...(xN)yN+1-(xN+1)yN)
	 * @param xArray x coordinates of points
	 * @param yArray y coordinates of points
	 * @return area
	 */
	public static double polygonArea(double[] xArray, double[] yArray) {
		double Area = 0;//default area as zero
		for (int a= 0; a<xArray.length-1;a++)//does the formula
			Area += (xArray[a])*(yArray[a+1]) - (xArray[a+1])*(yArray[a]);
		Area += (xArray[xArray.length-1])*(yArray[0]) - (xArray[0])*(yArray[yArray.length-1]);
		Area /= 2;//divides by two
		Area = Math.abs(Area);//area cannot be negative
		return Area;
	}
	/**
	 * calculates angles of all the points
	 * @param xArray x coordinates of points
	 * @param yArray y coordinates of points
	 * @param lArray lengths of the polyon
	 * @return
	 */
	public static double[] getAngles(double[] xArray,double[] yArray,double[]lArray) {
		double[] Array = new double[xArray.length];//array containing the angles,in radians
		double xLength = 0,yLength = 0,hypo = 0;
		int count = 0;
		int oneCon,twoCon;
		/*
		 * x/yLength = components for hypotenuse
		 * hypo = hypotenuse
		 * count = cuonter for while loop
		 * num1,num2 = index used for calculation
		 */
		while (count<=xArray.length-1) {//loops for all points
			oneCon = count-1;//by default it should use previous and next points
			twoCon = count+1;//respectively
			if(count == 0) {//used for first point
				oneCon = xArray.length-1;
				twoCon = 1;
			}
			if(count == xArray.length-1) {//used for last point
				twoCon = 0;
			}
			xLength = Math.abs(xArray[oneCon] - xArray[twoCon]);//get the x and y lengths of the hypotenuse
			yLength = Math.abs(yArray[oneCon] - yArray[twoCon]);
			hypo = Math.floor(Hypo(xLength,yLength));//gets exact hypotenuse
			Array[count] = cosLaw(lArray[oneCon],lArray[count],hypo);//plugs in cosine law to find angle
			count++;
		}
		return Array;
	}
	/**
	 * gives the hypotenuse of the two lengths
	 * @param xLength first length
	 * @param yLength second length
	 * @return hypotenuse
	 */
	public static double Hypo(double xLength,double yLength) {
		return Math.sqrt(Math.pow(xLength,2)+Math.pow(yLength,2));
		//plugs in H.theorm
	}
	/**
	 * does the cosine law
	 * @param length1 first length
	 * @param length2 second length
	 * @param hypo third length
	 * @return the angle of the third length (hypo)
	 */
	public static double cosLaw(double length1,double length2,double hypo) {
		double angle = Math.acos((Math.pow(length1,2)+Math.pow(length2,2)-Math.pow(hypo,2))/(2*length1*length2));
		//plugs in formula
		return angle;
	}
	/**
	 * does the P.theorm of A^2+B^2=C^2 to find C
	 * @param x x length of the triangle
	 * @param y y length of the triangle
	 * @return hypotenuse
	 */
	public static double hLength(double x, double y) {
		return Math.sqrt(Math.pow(x, 2)+Math.pow(y, 2));
	}
	/**
	 * rounds number to a certain decimal number
	 * @param num number to be set
	 * @param dec number of decimals to round
	 * @return rounded number
	 */
	public static double roundNum(double num,double dec) {
		return Math.floor(num*Math.pow(10,dec))/Math.pow(10,dec);
	}
	/**
     * Gets the length of the text file (number of lines used)
     * @param filename name of the file to be outputted
     * @return length length of the text file
     * @exception Exception throws out input error
     */
    public static int txtLength(String filename) {
		int length = 0;
		//length = length of the file
		try {
			Scanner fileScanner = new Scanner(new File(filename));//initialize file
			while(fileScanner.hasNext()) {//loops until no more lines
				if (fileScanner.hasNext()) length++;//add one to counter
				else fileScanner.close();//close the file at the end of the file
				fileScanner.nextLine();//goes to next line
			}
		}
		catch(Exception e) {//outputs error
			System.out.println("reading error: "+e);
		}
		return length;//returns the length
    }
	/**
     * Reads a text file and dumps the data into a string array
     * @param filename name of the file to be outputted
     * @return txtArray a string array of the information
     * @exception Exception throws out input error
     */
    public static String[] txtDump(String filename) {
		String[] txtArray;
		txtArray = new String[0];
		/**
		 * txtArray = array informatin will be placed into
		 */
		try {
			Scanner fileScanner = new Scanner(new File(filename));
			int length = txtLength(filename),counter = 0;
			//fileScanner = opens the txt file
			//length = the number of lines the file has
			txtArray = new String[length];//delcares the same length as txt file
			while (fileScanner.hasNext() && counter < length) {//loops until no more lines
					txtArray[counter] = fileScanner.nextLine();//places line into array
					counter++;//next line
			}
			fileScanner.close();
		}
		catch(Exception e) {//outputs error
			System.out.println("reading error: "+e);
		}
		return txtArray;//returns the array
    }
	/**
	 * outputs a string array into a txt file
	 * @param filename name of the file to be outputted
	 * @param OutArray string array to be outputted
	 */
    public static void txtOutput(String filename,String[]OutArray) {
		try {
			PrintStream out = new PrintStream(new FileOutputStream(filename));
			//declares the txt file
			for(int x=0;x<OutArray.length;x++) {//loops for all the lines in the array
				out.print(OutArray[x]);//outputs one line
				if(x != OutArray.length-1)out.println();//moves to next line
				//if condition to stop it at the last line
			}
			out.close();//close the file
		}
		catch(Exception e){//outputs the error
			System.out.println("output error: "+e);
		}
    }
	/**
	 * creates a file
	 * @param name name of the file. Should include directory
	 */
	public static void createFile(String name) {
		Writer writer = null;//used for writing into stream
        try {
            File file = new File(name);//creates file to create
            writer = new BufferedWriter(new FileWriter(file));//creates file
			writer.close();//closes file
        } catch (FileNotFoundException e) {//output error
            e.printStackTrace();
        } catch (IOException e) {//output error
            e.printStackTrace();
        }
	}
}//applet