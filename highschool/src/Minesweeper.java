//delete if not using this package
import java.applet.*;
import java.awt.*;
import java.awt.event.*;
import java.io.*;
import java.util.*;
import java.text.*;
/**
 *
 */
public class Minesweeper extends Applet implements KeyListener, MouseListener, MouseMotionListener,Runnable {
	/**directory location*/			String dirLoc="C:\\School\\Project\\src\\project\\";
	/**cursor size:10*/				int cursorSize = 3,
	/**X size of applet:700*/			windowX,
	/**Y size of applet:700*/			windowY,
										gridX = 10,
										gridY = 10,
										gridSize = 20,
										gameMines = 10,
										gameFlags = gameMines,
										gameTime = 0,
										gameTimeMin =0,
										gameTimeCurrent = 0;
									Color gridColor = Color.blue,
										gridBackground = Color.darkGray,
										cursorColor = Color.red,
										gridMineColor = Color.lightGray;
									Color[] gridArrayColor = new Color[10];
									

	/**reads the key pressed by user*/String keyReader = "";
	/**width of the applet*/int width;/**height of the applet*/int height;
	/**x mouse position*/int mouseX;/**y mouse position*/ int mouseY;
	/**backbuffer used to stop flashing*/Image backbuffer;
	/**backgroundd image. Used with backbuffer*/Graphics backg;
	/**polygon shape completed*/boolean polyDone = false,
	/**button is pressed or not*/isButtonPressed = false;
	/*  0=default 1=lose 2=win */ byte gameOver = 0;
	Thread t = null;
	Calendar cal = Calendar.getInstance();
	int gameTimeStart = cal.get( Calendar.SECOND ),counter = gridX*gridY;
	String gameOvermsg = "";
	
	int[][] gridMine;
	boolean[][] gridShow;
	Random generator = new Random();
	@Override
	public void init() {
		if(gameTimeStart == 60) gameTimeStart = -1;
		gridArrayColor[0] = Color.white;
		gridArrayColor[4] = Color.pink;
		gridArrayColor[1] = Color.orange;
		gridArrayColor[2] = Color.yellow;
		gridArrayColor[3] = Color.green;
		gridArrayColor[4] = Color.cyan;
		gridArrayColor[6] = Color.blue;
		gridArrayColor[7] = Color.magenta;
		gridArrayColor[8] = Color.red;
		gridArrayColor[9] = Color.black;
		windowX=gridX*(gridSize+1)+1;//the plus one is needed to show the margins
		windowY=gridY*(gridSize+1)+1;
		resize(windowX,windowY+40);//resizes window to correct size
		width = getSize().width;//gets size of applet
		height = getSize().height;
		setBackground(gridBackground);
		mouseX = width/2;//default position of mouse
		mouseY = height/2;
		backbuffer = createImage(width,height);//creates backbuffer
		backg = backbuffer.getGraphics();//defined into a graphic

		addKeyListener( this );//used for key and mouse reading
		addMouseListener( this );
		addMouseMotionListener( this );
		setFocusable(true);//focuses window
		backg.setColor(gridColor);
	}

	@Override
	public void start() {
		mine_grid_default();
		mine_grid_plantMines(gameMines);
		if ( t == null ) {//thread not delcared
			t = new Thread( this );//delcare thread
			t.setPriority( Thread.MIN_PRIORITY );//minimum priority
			t.start();//start thread
		}
		mine_grid_drawScreen();
	}
	public void keyPressed( KeyEvent e ) { }//required by default
	public void keyReleased( KeyEvent e ) { }
	public void keyTyped( KeyEvent e ) {
		backg.clearRect(0,670,width, height);//clear bottom of the applet
		repaint();
		e.consume();
	}//keypressed()

	public void mouseEntered( MouseEvent e ) { }
	public void mouseExited( MouseEvent e ) { }
	public void mousePressed( MouseEvent e ) { }
	public void mouseReleased( MouseEvent e ) { }
	public void mouseClicked( MouseEvent e ) {
	int c = e.getButton();
	if(gameOver == 0) {//cannot click when game over'd
		if(c == e.BUTTON1) {
			mouseX = e.getX();
			mouseY = e.getY();
			int squareX=mouseX/(gridSize+1),
				squareY=mouseY/(gridSize+1);
			if(squareX<0)squareX++;
			if(squareY<0)squareY++;
			mine_grid_checkMines(squareY,squareX);
			if(counter == gameMines) gameOver = 2;//win condition
			if(gridMine[squareY][squareX] == 9) gameOver=1;//lose condition
			if (gameOver == 1) gameOvermsg = " You lose!";
				else if (gameOver == 2) gameOvermsg = " You Win!";
			mine_grid_drawScreen();
			repaint();
			e.consume();
		}
	}}//mouseClicked()
	public void mouseDragged( MouseEvent e ) {  // called during motion with buttons down
		mouseX = e.getX();
		mouseY = e.getY();
		repaint();
		e.consume();
	}
	public void mouseMoved( MouseEvent e ) {// called during motion when no buttons are down
		mouseX = e.getX();
		mouseY = e.getY();
		mine_grid_drawScreen();
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
	public void run() {
		try {
		while (true) {
			// Here's where the thread does some work
			if(gameOver == 0) {
				cal = Calendar.getInstance();
				gameTimeCurrent = cal.get( Calendar.SECOND );
				if(gameTimeCurrent == 0) {
					gameTimeStart  = cal.get( Calendar.SECOND );
					gameTimeMin += gameTime;
					gameTime=0;
				}
				else gameTime = gameTimeCurrent-gameTimeStart;
			}
			showStatus("B: "+gameFlags+"W: "+counter+" T: "+(gameTime+gameTimeMin)+gameOvermsg);
		}
		}
		catch (Exception e) { }
	}
	public void mine_grid_drawScreen() {
		backg.clearRect(0,0,width,height);
		mine_grid_drawLines();
		mine_grid_drawMines();
		backg.setColor(cursorColor);
		backg.drawLine(mouseX,mouseY,mouseX, mouseY-cursorSize);//draws cursor
		backg.drawLine(mouseX,mouseY,mouseX-cursorSize,mouseY);
		backg.drawLine(mouseX,mouseY,mouseX, mouseY+cursorSize);
		backg.drawLine(mouseX,mouseY,mouseX+cursorSize,mouseY);
		backg.setColor(gridColor);
	}

	public void mine_grid_drawMines() {
		backg.setColor(gridMineColor);
		int a=1,b=1;
		for(int y=0;y<gridY;y++) {
			for(int x=0;x<gridX;x++) {
				if(gridShow[y][x] == false) 
					backg.fillRect(gridSize*x+a,gridSize*y+b, gridSize,gridSize);
				else {
					if(gridMine[y][x] == 9) {
						backg.setColor(Color.red);
						backg.fillRect(gridSize*x+a,gridSize*y+b, gridSize,gridSize);
					}
					backg.setColor(gridArrayColor[gridMine[y][x]]);
					backg.drawString(Integer.toString(gridMine[y][x]),gridSize*x+a+5,gridSize*y+b+15);
				}
				backg.setColor(gridMineColor);
				a++;
			}
			a=1;
			b++;
		}
		backg.setColor(Color.green);
	}
	public void mine_grid_drawLines() {
		for(int a=0; a<=gridX;a++) {
			backg.drawLine(0,a*(gridSize+1), gridX*(gridSize+1),a*(gridSize+1));
		}
		for(int a=0; a<=gridY;a++) {
			backg.drawLine(a*(gridSize+1),0, a*(gridSize+1),gridY*(gridSize+1));
		}
	}
	public void mine_grid_checkMines(int y,int x) {//checks for all blank squares
		if(gridShow[y][x] == true) return;
		if (gridMine[y][x] != 0) {
			gridShow[y][x] = true;
			counter--;
			return;
		} else {
			gridShow[y][x] = true;
			counter--;
			try { mine_grid_checkMines(y-1,x); } catch(Exception e) { }
			try { mine_grid_checkMines(y+1,x); } catch(Exception e) { }
			try { mine_grid_checkMines(y,x-1); } catch(Exception e) { }
			try { mine_grid_checkMines(y,x+1); } catch(Exception e) { }
		}
	}
	public void mine_grid_plantMines(int counter) {
		while(counter>0) {
			int minePlantX= generator.nextInt(gridX),minePlantY= generator.nextInt(gridY);
			if(gridMine[minePlantX][minePlantY] != 9) {
				gridMine[minePlantX][minePlantY]=8;
				for(int x=0;x<3;x++) {
					for(int y=0;y<3;y++) {
						try
						{ if(gridMine[minePlantX-1+y][minePlantY-1+x] < 9) gridMine[minePlantX-1+y][minePlantY-1+x]++; }
						catch(Exception e) {}
				}}
				counter--;
			}
		}
	}
	public void mine_grid_default() {
		gridShow = new boolean[gridX][gridY];
		gridMine = new int[gridX][gridY];
		for(int y=0;y<gridY;y++) {//default
			for(int x=0;x<gridY;x++) {
				gridShow[x][y] = false;
				gridMine[x][y] = 0;
			}
		}
	}
}//applet