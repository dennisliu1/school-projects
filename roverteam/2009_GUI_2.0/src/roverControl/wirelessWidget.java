package roverControl;

import java.io.IOException;
import java.io.InputStream;
import java.net.ConnectException;
import java.net.Socket;
import java.net.UnknownHostException;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Timer;
import java.util.TimerTask;

import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Label;

public class wirelessWidget extends Composite {

	@Override
	public void dispose() {
		super.dispose();
		timer.cancel();
	}

	private Display display;
	private Socket telescope_socket;
	private InputStream telescope_in;
	private String host;
	private byte[] b = new byte[2000];
	private Label essid;
	private Label ap;
	private Label bitRate;
	private Label txPower;
	private Label linkQuality;
	private Label signalLevel;
	private Label lastUpdate;
	private Timer timer;

  public static String now(String dateFormat) {
	    Calendar cal = Calendar.getInstance();
	    SimpleDateFormat sdf = new SimpleDateFormat(dateFormat);
	    return sdf.format(cal.getTime());

	  }


	  
	public wirelessWidget(Composite parent, int style, String host_) {
		super(parent, style);
		display = getDisplay();
		host = host_;
		setLayout(new GridLayout(1, false));

		Label l = new Label(this, SWT.NONE);
		l.setText("Host: " + host);
		l.setLayoutData(new GridData());

		essid = new Label(this, SWT.NONE);
		essid.setText("No Connection");
		GridData gd = new GridData(SWT.FILL, SWT.FILL, true, true);
		gd.widthHint = 200;
		essid.setLayoutData(gd);

		ap = new Label(this, SWT.NONE);
		ap.setLayoutData(gd);

		bitRate = new Label(this, SWT.NONE);
		bitRate.setLayoutData(gd);

		txPower = new Label(this, SWT.NONE);
		txPower.setLayoutData(gd);

		linkQuality = new Label(this, SWT.NONE);
		linkQuality.setLayoutData(gd);

		signalLevel = new Label(this, SWT.NONE);
		signalLevel.setLayoutData(gd);

		lastUpdate = new Label(this, SWT.NONE);
		lastUpdate.setLayoutData(gd);

		timer = new Timer();
		timer.scheduleAtFixedRate(new TimerTask(){
			private String essid_;
			private String ap_;
			private String bitRate_;
			private String txPower_;
			private String linkQuality_;
			private String signalLevel_;

			public void run() {
				String s = getReading();

				if(!s.equals("")){
					essid_ = "ESSID: " + s.substring(s.indexOf("ESSID:")).split("\"")[1];
					ap_ =  "Access Point: " + s.substring(s.indexOf("Access Point:")).split(" ")[2];
					bitRate_ = s.substring(s.indexOf("Bit Rate:")).split(" ")[0] + " " + s.substring(s.indexOf("Bit Rate:")).split(" ")[1] + s.substring(s.indexOf("Bit Rate:")).split(" ")[2];
					txPower_ = s.substring(s.indexOf("Tx-Power:")).split(" ")[0] + " " + s.substring(s.indexOf("Tx-Power:")).split(" ")[1];
					linkQuality_ = s.substring(s.indexOf("Link ")).split(" ")[0] + s.substring(s.indexOf("Link ")).split(" ")[1];
					signalLevel_ = s.substring(s.indexOf("Signal level")).split(" ")[0] + " " + s.substring(s.indexOf("Signal level")).split(" ")[1] + s.substring(s.indexOf("Signal level")).split(" ")[2];;
					
					display.asyncExec(new Runnable(){@Override
						public void run() {
							essid.setText(essid_);
							ap.setText(ap_);
							bitRate.setText(bitRate_);
							txPower.setText(txPower_);
							linkQuality.setText(linkQuality_);
							signalLevel.setText(signalLevel_);
							lastUpdate.setText("Last update at " + now("H:mm:ss"));
						}
					});
				}else{
					
					display.asyncExec(new Runnable(){@Override
						public void run() {
							essid.setText("No Connection");
							ap.setText("");
							bitRate.setText("");
							txPower.setText("");
							linkQuality.setText("");
							signalLevel.setText("");
							lastUpdate.setText("");
						}
					});
				}
					
			}
		}, 0, 1000*15);
	}

	private String getReading() {

		if (connect()) {

			try {
				int res = telescope_in.read(b, 0, 2000);
				return new String(b, 0, res);

			} catch (IOException e) {
				e.printStackTrace();
			}
			disconnect();
		}
		return "";
	}

	private void disconnect() {
		if (telescope_socket != null) {
			try {
				// telescope_out.close();
				telescope_in.close();
				telescope_socket.close();

			} catch (IOException e) {
				e.printStackTrace();

			}
		}
	}

	private boolean connect() {
		try {
			telescope_socket = new Socket(host, 30300);
			telescope_in = telescope_socket.getInputStream();

		} catch (ConnectException e) {
			System.out.println("wlanServer not listening");
			return false;

		} catch (UnknownHostException e) {
			System.out.println("unknown host");
			return false;
		} catch (IOException e) {
			System.out.println("general exception");
			return false;
		}
		return true;
	}

}
