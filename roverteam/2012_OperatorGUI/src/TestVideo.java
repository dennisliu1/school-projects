import java.awt.BorderLayout;
import java.awt.Dimension;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import org.gstreamer.Bin;
import org.gstreamer.Bus;
import org.gstreamer.Caps;
import org.gstreamer.Element;
import org.gstreamer.Element.PAD_ADDED;
import org.gstreamer.ElementFactory;
import org.gstreamer.GhostPad;
import org.gstreamer.Gst;
import org.gstreamer.GstObject;
import org.gstreamer.Pad;
import org.gstreamer.Pipeline;
import org.gstreamer.State;
import org.gstreamer.Structure;
import org.gstreamer.elements.DecodeBin2;
import org.gstreamer.swing.VideoComponent;


public class TestVideo {
    public TestVideo() {
    }
    private static Pipeline pipe;
    public static void main(String[] args) {
        args = Gst.init("VideoTest", args);
        pipe = new Pipeline("VideoStream");
        final Element videosrc = ElementFactory.make("rtspsrc", "source");//source=name; does nothing
        final Element queue = ElementFactory.make("queue", "queue");
        final Element decoder = ElementFactory.make("decodebin2","decoder");
        final Element sink = ElementFactory.make("xvimagesink", "video");
        
        videosrc.set("location", "rtsp://192.168.80.100:9400/test");
        videosrc.set("latency", 0);
        sink.set("sync", false);
        
        
        
        //final Element videofilter = ElementFactory.make("capsfilter", "filter");
        //videofilter.setCaps(Caps.fromString("video/x-raw-yuv, width=720, height=576"
        //        + ", bpp=32, depth=32, framerate=25/1"));
        SwingUtilities.invokeLater(new Runnable() {

            public void run() {
                VideoComponent videoComponent = new VideoComponent();
                Element videosink = videoComponent.getElement();
                pipe.addMany(videosrc, queue, decoder, sink, videosink);
                Element.linkMany(videosrc, queue, decoder, sink, videosink);
                
                // Now create a JFrame to display the video output
                JFrame frame = new JFrame("Swing Video Test");
                frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
                frame.add(videoComponent, BorderLayout.CENTER);
                videoComponent.setPreferredSize(new Dimension(720, 576));
                frame.pack();
                frame.setVisible(true);
                
                // Start the pipeline processing
                pipe.setState(State.PLAYING);
            }
        });
    }
}