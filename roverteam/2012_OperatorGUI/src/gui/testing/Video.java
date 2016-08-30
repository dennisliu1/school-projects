package gui.testing;
import java.awt.BorderLayout;
import java.awt.Dimension;
import java.io.File;

import org.gstreamer.*;
import org.gstreamer.elements.PlayBin2;
import org.gstreamer.swing.VideoComponent;
import java.net.URI;
import java.net.URISyntaxException;

import javax.swing.*;

public class Video {
	final PlayBin2 playbin;
	private VideoComponent videoComponent;
	
	public Video(String [] args, String uri) throws URISyntaxException {
		args = Gst.init("VideoPlayer", args);
		playbin = new PlayBin2("VideoPlayer", new URI(uri));
		SwingUtilities.invokeLater(new Runnable() {
	
			public void run() {
				videoComponent = new VideoComponent();
				playbin.setVideoSink(videoComponent.getElement());
				
				JFrame frame = new JFrame("VideoPlayer");
				frame.getContentPane().add(videoComponent, BorderLayout.CENTER);
				frame.setPreferredSize(new Dimension(640, 480));
				frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
				frame.pack();
				frame.setVisible(true);
				playbin.setState(State.PLAYING);
			}
		});
		Gst.main();
		playbin.setState(State.NULL);
	}
	public void setPlay(){
		playbin.setState(State.PLAYING);
	}
	public void setPause(){
		playbin.setState(State.PAUSED);
	}
	public void setStop(){
		playbin.setState(State.NULL);
	}
	public VideoComponent getVideoComponent(){
		return videoComponent;
	}
}