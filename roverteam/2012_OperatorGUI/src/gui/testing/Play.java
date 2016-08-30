package gui.testing;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.net.URISyntaxException;

import javax.swing.JFrame;
import javax.swing.WindowConstants;

import org.gstreamer.State;
import org.gstreamer.swing.VideoComponent;

import cameraTabs.RTSPPlayer;


public class Play {

	/**
	 * @param args
	 * @throws URISyntaxException 
	 */
	public static void main(String[] args) throws URISyntaxException {
		// TODO Auto-generated method stub
//		Video v = new Video(args, "rtsp://127.0.0.1:8554/test");
		
		int width1 = 500;
		int height1 = 500;
		int x1 = 0;
		int y1 = 0;
		RTSPPlayer v = new RTSPPlayer(args, "rtsp://127.0.0.1:8554/test");
		v.videoComponent = new VideoComponent();
		v.setVideoSink();
		v.videoComponent.setPreferredSize(new Dimension(width1, height1));
		v.videoComponent.setBounds(x1,y1, width1, height1);
		v.videoComponent.setSize(width1, height1);
		v.playbin.setState(State.PLAYING);
		JFrame frame = new JFrame("VideoPlayer");
		frame.add(v.videoComponent);
//		frame.getContentPane().add(v.videoComponent, BorderLayout.CENTER);
		frame.setPreferredSize(new Dimension(640, 480));
		frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
		frame.pack();
		frame.setVisible(true);
		
	}

}
