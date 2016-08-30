package cameraTabs;

import java.awt.Canvas;
import java.awt.Component;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JTextField;


import uk.co.caprica.vlcj.player.MediaPlayerFactory;
import uk.co.caprica.vlcj.player.embedded.EmbeddedMediaPlayer;


public class VideoPlayer implements ActionListener{
	private static final String GRANDSTREAM_VIDEO_PREFIX = "rtsp://admin:admin@";
	
	
	// Private instance variables
	private EmbeddedMediaPlayer player;
	private String mediaUri;
	private String mediaOptions;
	
	// Control panel componenets
	private JButton play, stop;
	private JTextField addr, port, cache;
	
	// Optional PTZ components (hardcoded for GrandStream)
	private JButton left, right, up, down, center;
	
	
	public VideoPlayer (Canvas cv, CameraInitPanel control, boolean ptz) {
		
		MediaPlayerFactory factory = new MediaPlayerFactory(new String[] {});
		
		player = factory.newMediaPlayer(null);
		
		player.setVideoSurface(cv);
		
		if (control != null){
			//assign Panel elements to local variables
			addr = control.fieldInfos[0];
			port = control.fieldInfos[1];
			cache = control.fieldInfos[2];
			play = control.startCamB;
			stop = control.stopCamB;
//			play.addActionListener(this);
//			stop.addActionListener(this);
		}
	}
	
	public VideoPlayer (Canvas cv, CameraInitPanel control) {
		
		this(cv, control, false);
		
	}
	
	public VideoPlayer (Canvas cv){
		
		this(cv ,null);
		
	}
	
	public void changeMedia(String addr, String port, int cache){
		
		mediaUri = addr;
		
		if (mediaUri.startsWith("rtsp")){
			if (port != null && port.length() > 0)
				mediaUri = mediaUri + "/" + port;
			mediaOptions = ":rtsp-caching=" + cache;
		} else if (mediaUri.startsWith("http")){
			if (port != null && port.length() > 0)
				mediaUri = mediaUri + "/" + port;
			mediaOptions = ":http-caching=" + cache;
		} else {
			if (port != null && port.length() > 0)
				mediaUri = mediaUri + "/" + port;
			mediaOptions = "";
		}
		
	}
	
	public void changeMedia(String addr){
		// Use default cache size of 300 ms
		changeMedia(addr, null, 300);
	}
	
	public void play(){
		
		player.playMedia(mediaUri, mediaOptions);
		
	}
	
	public void stop(){
		
		player.stop();
		
	}

	@Override
	public void actionPerformed(ActionEvent ae) {
		
		Object source = ae.getSource();
		
		if (source.equals(play)){
			
			int cacheVal;
				
			try {
				cacheVal = Integer.parseInt(cache.getText());
			}catch (Exception e){
				cacheVal = 0;
			}
			
			this.changeMedia(GRANDSTREAM_VIDEO_PREFIX+addr.getText(), port.getText(), cacheVal);
			this.play();
			
		} else if (source.equals(stop)) {
			this.stop();
		}
		
	}
	

}
