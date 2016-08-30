/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package GUI;

/**
 *
 * @author den
 */
public class Main {
	public static final int MAIN_GUI_WIDTH = 1280, MAIN_GUI_HEIGHT = 600;
	public static void main(String[] args) {
		GUI inter = new GUI(MAIN_GUI_WIDTH, MAIN_GUI_HEIGHT, false, true);
		inter.setAlwaysOnTop(true);
	}
}
