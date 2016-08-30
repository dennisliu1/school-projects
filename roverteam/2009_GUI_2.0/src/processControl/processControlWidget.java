package processControl;
/**
 * processControl Client widget for MarsRover Project @ York
 * Author: Alexandre Walzberg
 * Date: Mar 15, 2009.
 */

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ConnectException;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Timer;
import java.util.TimerTask;
import java.lang.Thread;

import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.MessageBox;
import org.eclipse.swt.widgets.Text;
import org.eclipse.swt.widgets.Shell;

public class processControlWidget extends Composite {
	
	private processControlWidget me;
	
	private final int RUNNING=1, TERMINATED=2, KILLED=3;

	private String hostname;
	private int port;
	private Socket tcpConnection;
	private InputStream tcpIn;
	private OutputStream tcpOut;
	
	private boolean connection_ok=false;
	private String connectionState="Not connected yet"; 
	private Label connectionStatusL;
		
	private Thread thread;
	private process[] process_tab;
	private boolean get_list=false;
	private int lastUpdate;
	private int tab;
	private processControlTabs tabs;
	
	private Timer timer;
	
	public String statusText[] = {"off", "Running", "Terminated", "Killed", "??"};
	
	private class process{
		// process param
		public String name;
		public String path;
		public String def_param;
		public byte id;
		public int port;
		public boolean def_launch;

		// process graphics
		public Button pButtonRun;
		public Button pButtonKill;
		public Label pLabelName;
		public Label pLabelState;
		
		public Text pTextParam;
		// process statue
		public int pid;
		public int status;
		public int returnValue;
		
		public process(final String name, String path, final String param, int port, boolean def_launch, int pid, final int status, final byte id){
			// init fields
			this.name = name;
			this.path = path;
			this.id = id;
			this.def_param = param;
			this.port = port;
			this.def_launch = def_launch;
			this.pid = pid;
			this.status = status;
			this.returnValue = 0;
			
			me.getDisplay().syncExec(new Runnable() {
				public void run() {
					pLabelName = new Label(me, SWT.NONE);
					pLabelName.setText(name);
					GridData g = new GridData(SWT.LEFT, SWT.LEFT, false, false);
					pLabelName.setLayoutData(g);
					
					pTextParam = new Text(me, SWT.BORDER);
					pTextParam.setText(param);
					g = new GridData(SWT.LEFT, SWT.LEFT, false, false);
					g.widthHint = 100;
					pTextParam.setLayoutData(g);
					
					pLabelState = new Label(me, SWT.BORDER);
					pLabelState.setText(statusText[status]);
					g = new GridData(SWT.FILL, SWT.LEFT, true, false);
					g.widthHint = 150;
					pLabelState.setLayoutData(g);
					
					pButtonRun = new Button(me, SWT.NONE);
					pButtonRun.setText("Run");
					g = new GridData(SWT.LEFT, SWT.LEFT, false, false);
					pButtonRun.setLayoutData(g);
					pButtonRun.addListener(SWT.Selection, new Listener(){@Override
						public void handleEvent(Event arg0) {
							byte b[] = new byte[2];
							byte cmd = (byte)10;
							
							try{
								if(process_tab[id].def_param.compareTo(process_tab[id].pTextParam.getText()) != 0){
									b[0]=(byte)11; b[1]=(byte)(id+1);
									byte param[] = (process_tab[id].pTextParam.getText()).getBytes();
									int size = process_tab[id].pTextParam.getText().length();
									byte sizeb[] = intToByteArray(size);
									tcpOut.write(b, 0, 2);
									tcpOut.write(sizeb, 0, 4);
									tcpOut.write(param, 0, size);
								}else{
									b[0]=cmd; b[1]=(byte)(id+1);
									tcpOut.write(b, 0, 2);
								}
							} catch (IOException e) {
								resetConnection();
							}							
						}
					});
					
					
					pButtonKill = new Button(me, SWT.NONE);
					pButtonKill.setText("Kill");
					g = new GridData(SWT.LEFT, SWT.LEFT, false, false);
					pButtonKill.setLayoutData(g);
					pButtonKill.addListener(SWT.Selection, new Listener(){@Override
						public void handleEvent(Event arg0) {
							try{
								byte b[] = new byte[2];
								byte cmd=(byte)21;
								b[0]=cmd; b[1]=(byte)(id+1);
								MessageBox messageBox = new MessageBox(new Shell(), SWT.ICON_QUESTION | SWT.YES | SWT.NO);
							    messageBox.setMessage("Do you really want to kill the apllication?");
							    messageBox.setText("Kill application?");
							    int response = messageBox.open();
							    if (response == SWT.YES) tcpOut.write(b, 0, 2);   
								
							} catch (IOException e) {
								resetConnection();
							}
						}
					});
					
					refresh();
					me.pack();
				}
			});
		}
		
		public void refresh(){
			switch(this.status){
				case 1:
					pLabelState.setText(statusText[this.status]+" pid:"+Integer.toString(this.pid));
					break;
				case 2:
					pLabelState.setText(statusText[this.status] + " (" + this.returnValue + ")");
					break;
				case 0: case 3: case 4:
					pLabelState.setText(statusText[this.status]);
					break;
			}
			if(this.status == 1){
				pLabelState.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_GREEN));
				pButtonRun.setEnabled(false);
				pButtonKill.setEnabled(true);
			}else{
				pLabelState.setBackground(Display.getCurrent().getSystemColor(SWT.COLOR_RED));
				pButtonRun.setEnabled(true);
				pButtonKill.setEnabled(false);
			}
		}
	}

	public processControlWidget(Composite parent, int style, String hostname, int port, processControlTabs tabs1, int tab1) {

		//setting basis
		super(parent, style);
		setLayout(new GridLayout(5, false));
		this.me=this;
		tab=tab1;
		tabs=tabs1;
		
		//setting network
		this.hostname = hostname;
		System.out.println(hostname);
		this.port = port;
		
		//------------------------------------------------------------------
		connectionStatusL = new Label(this, SWT.NONE); 
		connectionStatusL.setText(connectionState);
		connectionStatusL.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false, 3, 0));
		
		Button reconnect = new Button(this, SWT.NONE);
		reconnect.setText("Reconnect");
		reconnect.setLayoutData(new GridData(SWT.RIGHT, SWT.LEFT, false, false, 2, 0));
		reconnect.addListener(SWT.Selection, new Listener(){@Override
			public void handleEvent(Event arg0) {
				tabs.reconnect(tab);	
			
			}
		});
		
		//------------------------------------------------------------------		
		Label caption0 = new Label(this, SWT.NONE);
		caption0.setText("Program");
		caption0.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false));
		
		Label caption1 = new Label(this, SWT.NONE);
		caption1.setText("Param");
		caption1.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false));
		
		Label caption2 = new Label(this, SWT.NONE);
		caption2.setText("Status");
		caption2.setLayoutData(new GridData(SWT.LEFT, SWT.LEFT, false, false));
		
		new Label(this, SWT.NONE).setLayoutData(new GridData());
		new Label(this, SWT.NONE).setLayoutData(new GridData());
		
		timer = new Timer();
		timer.scheduleAtFixedRate(new tryConnect(), 0, 1000);
	}
	
	private class receiveMessage extends Thread {
		
		public receiveMessage(String host){
			super("receiver"+host);
		}
		
		@Override
		public void run(){
			while(true){
				
				if(!get_list && connection_ok){
					try
					{
						byte b[] = new byte[2];
						b[0]=0x0; b[1]=0x0;
						tcpOut.write(b,0,2);
					} catch (IOException e) {
						me.resetConnection();
					}
				}
				
				if(connection_ok){
					try{
						lastUpdate=-1;
						byte b[] = new byte[10];
						byte bpid[] = new byte[5];
						int r;
						if (readSocket(2, b)>0){
							int cmd=b[0];
							int id=b[1];
							switch(cmd){
								case 0:
									if(readSocket(4, b)>0){
										int length = (b[0]&255)+((b[1]&255)<<8)+((b[2]&255)<<16)+((b[3]&255)<<24);
										byte list[] = new byte[60000];
										if(readSocket(length+1, list)>0){;
											String lines = new String(list);
											process_tab=new process[id];
											for(byte i=0; i<id; i++){
												String line = lines.split("\n")[i];
												process_tab[i] = new process(
													line.split("#")[0],
													line.split("#")[1],
													line.split("#")[2],
													Integer.parseInt(line.substring(2).split("#")[3]),
													Boolean.parseBoolean(line.split("#")[4]),
													Integer.parseInt(line.split("#")[5]),
													Integer.parseInt(line.split("#")[6]),
													i
												);
											}

											connectionState = "Connected!";
											get_list=true;
											
										}else{
											me.resetConnection();
										}
									}else{
										me.resetConnection();
									}
									break;
									
								case 10: case 11:
									if (readSocket(5, bpid)>0){
										r = Integer.parseInt(new String(bpid));
										process_tab[id-1].pid=r;
										if(r>0) process_tab[id-1].status=RUNNING;
										lastUpdate = id-1;
									}else{
										me.resetConnection();
									}
									break;
								case 20:
									if (readSocket(4, b)>0){
										r = (b[0]+(b[1]<<8)+(b[2]<<16)+((b[3]<<24)));
										process_tab[id-1].returnValue=r;
										process_tab[id-1].status=TERMINATED;
										lastUpdate = id-1;
									}else{
										me.resetConnection();
									}
									break;
								case 21:
									if (readSocket(4, b)>0){
										r = (b[0]+(b[1]<<8)+(b[2]<<16)+((b[3]<<24)));
										if(r>0){
											process_tab[id-1].pid=0;
											process_tab[id-1].status=KILLED;
										}
										lastUpdate = id-1;
									}else{
										me.resetConnection();
									}
									break;
							}
						}else{
							me.resetConnection();
						}
						if(lastUpdate != -1){
							me.getDisplay().syncExec(new Runnable() {
								public void run() {
									process_tab[lastUpdate].refresh();
								}
							});
						}
						
					} catch (IOException e) {
						me.resetConnection();
					}
				}
				
				me.getDisplay().syncExec(new Runnable() {
					public void run() {
						connectionStatusL.setText(connectionState);
						connectionStatusL.getShell().pack();
					}
				});
			}
		}
	}
	
	public class tryConnect extends TimerTask {
		public void run(){
			boolean error=false;
			if(!connection_ok){
				try{
					tcpConnection = new Socket(hostname, port);
					tcpOut = tcpConnection.getOutputStream();
					tcpIn = tcpConnection.getInputStream();
					
				}catch(ConnectException e) {
					connectionState = "Connection refused! Make sure the process_manager server is running";
					error=true;
				} catch (UnknownHostException e) {
					connectionState = "_Can't reach host! Can you ping the host?";
					error=true;
				} catch (IOException e) {
					connectionState = "Connection IO problem! Interupted ?";
					error=true;
				}
				
				if(error==false){
					connection_ok = true;
					connectionState = "Connected! Retrieve application";
					thread = new receiveMessage(me.hostname);
					thread.start();
					timer.cancel();
				}
				
				if(!me.isDisposed()){
					me.getDisplay().syncExec(new Runnable() {
						public void run() {
							connectionStatusL.setText(connectionState);	
							connectionStatusL.pack();
							connectionStatusL.getParent().layout();
						}
					});
				}
			}
		}
	}
	
	private int readSocket(int length, byte[] buf) throws IOException {
		int res=0;
		while (length != res) {
			int r = tcpIn.read(buf, res, length - res);
			if (r > 0){
				res += r;
			}else{
				return r;
			}
		}
		return 1;
	}
	
	private static byte[] intToByteArray(int value) {
        byte[] b = new byte[4];
        for (int i = 0; i < 4; i++) {
            int offset = (b.length - 1 - i) * 8;
            b[i] = (byte) ((value >>> offset) & 0xFF);
        }
        return b;
    }
	
	private void resetConnection() {
		connectionState="Disconnected" ;
		connection_ok=false;
		get_list=false;
				
		disconnect();
		
		me.getDisplay().syncExec(new Runnable() {
			public void run() {
				connectionStatusL.setText(connectionState);	
				connectionStatusL.pack();
				connectionStatusL.getParent().layout();
			
			}
		});
	}

	private void disconnect() {
		if(thread != null)
			thread.stop();
		try{
			if(tcpOut != null) tcpOut.close();
			if(tcpIn != null) tcpIn.close();
			if(tcpConnection != null)tcpConnection.close();
		} catch (IOException e) {
			connectionState = "Connection IO problem! Interupted ?";
		}
	}
	
	@Override
	public void dispose() {
		connectionState="Disconnected. Trying to reconnect...";
		if(connection_ok==false && timer != null) timer.cancel();
		disconnect();
		
		super.dispose();
	}
}
