package cameraTabs;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.net.URI;
import java.net.URISyntaxException;

import org.gstreamer.Gst;
import org.gstreamer.Pipeline;
import org.gstreamer.State;
import org.gstreamer.elements.PlayBin2;
import org.gstreamer.swing.VideoComponent;

public class RTSPPlayer implements ActionListener{
	public final PlayBin2 playbin;
	public VideoComponent videoComponent;
	public StopActionListener stopVideoActionListener;
	private RTSPPlayer self = this;
	
	public RTSPPlayer(String [] args, String uri) throws URISyntaxException {
		args = Gst.init("VideoPlayer Test", args);
		playbin = new PlayBin2("VideoPlayer", new URI(uri));
		playbin.set("latency", 0);
//		SwingUtilities.invokeLater(new Runnable() {
//			
//			public void run() {
//				//videoComponent = new VideoComponent();
//				//frame.getContentPane().add(videoComponent, BorderLayout.CENTER);
//				//playbin.setState(State.PLAYING);
//			}
//		});
//		videoComponent = new VideoComponent();
//		Gst.main();
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
	public State getState() {
		return playbin.getState();
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
