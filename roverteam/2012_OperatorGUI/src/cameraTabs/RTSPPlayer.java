package cameraTabs;

import java.awt.BorderLayout;
import java.awt.Canvas;
import java.awt.Component;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;

import org.gstreamer.Gst;
import org.gstreamer.State;
import org.gstreamer.elements.PlayBin2;
import org.gstreamer.swing.VideoComponent;


import uk.co.caprica.vlcj.player.MediaPlayerFactory;
import uk.co.caprica.vlcj.player.embedded.EmbeddedMediaPlayer;


public class RTSPPlayer implements ActionListener{
	public final PlayBin2 playbin;
	public VideoComponent videoComponent;
	public StopActionListener stopVideoActionListener;
	private RTSPPlayer self = this;
	
	public RTSPPlayer(String [] args, String uri) throws URISyntaxException {
		//this.videoComponent = videoComponent;
		args = Gst.init("VideoPlayer Test", args);
		playbin = new PlayBin2("VideoPlayer", new URI(uri));
//		SwingUtilities.invokeLater(new Runnable() {
//			
//			public void run() {
//				//videoComponent = new VideoComponent();
//				//frame.getContentPane().add(videoComponent, BorderLayout.CENTER);
//				//playbin.setState(State.PLAYING);
//			}
//		});
		playbin.setState(State.NULL);
		stopVideoActionListener = new StopActionListener();
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
	public void setVideoSink() {
		playbin.setVideoSink(self.videoComponent.getElement());
		//playbin.setVideoSink(videoComp.getElement());
	}
	public void startMain() {
		Gst.main();
	}
	@Override
	public void actionPerformed(ActionEvent e) {
		// switch; turns start to pause, and back.
		if(playbin != null)
		if(playbin.getState().equals(State.PLAYING)) {
			this.setPause();
		} else if(playbin.getState().equals(State.PAUSED)) {
			this.setPlay();
		}
	}
	public class StopActionListener implements ActionListener {

		@Override
		public void actionPerformed(ActionEvent e) {
			if(playbin != null)
			if(playbin.getState() != null)
				setStop();
		}
		
	}
	public StopActionListener getStopActionListener() {
		return stopVideoActionListener;
	}
}//VideoPlayer class
