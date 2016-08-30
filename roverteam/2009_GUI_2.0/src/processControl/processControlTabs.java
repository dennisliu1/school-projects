package processControl;
/**
 * processControl Client widget for MarsRover Project @ York
 * Author: Alexandre Walzberg
 * Date: Mar 15, 2009.
 */

import org.eclipse.swt.SWT;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.TabFolder;
import org.eclipse.swt.widgets.TabItem;


public class processControlTabs extends Composite {
	
	private TabFolder folder;
	
	private String host[] = {"192.168.1.1", "192.168.0.1", "stamp1", "stamp2"};
	private String caption[] = {"rover", "base", "stamp1", "stamp2"};
	private int port[] = {30000, 30000, 30000, 30000, 30000};
	private int nbTab=host.length;
	
	private processControlWidget tab_content[] = new processControlWidget[nbTab];
	private TabItem tab[] = new TabItem[nbTab];
	
	public processControlTabs(Composite parent, int style){
		super(parent, style);
		setLayout(new GridLayout(1, false));
		
		folder = new TabFolder(this, SWT.NONE);
		folder.addSelectionListener(new SelectionListener(){@Override
			public void widgetSelected(SelectionEvent arg0) {
				if(folder.getShell()!=null){
					folder.getShell().pack();
					folder.getShell().layout();
				}
			}
	
			public void widgetDefaultSelected(SelectionEvent arg0) {}
		});
		GridData gd = new GridData(SWT.FILL, SWT.FILL, true, true);
		gd.minimumHeight = 200;
		gd.minimumWidth = 300;
		folder.setLayoutData(gd);
		
		for(int i=0; i<nbTab; i++){
			tab[i] = new TabItem(folder, SWT.NONE);
			tab[i].setText(caption[i]);
			tab_content[i] = new processControlWidget(folder, SWT.NONE, host[i], port[i], this, i);
			tab_content[i].setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
			tab[i].setControl(tab_content[i]);
		}
		
		folder.pack();
		
	}
	
	public void reconnect(int pcw){
		tab_content[pcw].dispose();
		tab_content[pcw] = new processControlWidget(folder, SWT.NONE, host[pcw], port[pcw], this, pcw);
		tab_content[pcw].setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
		tab[pcw].setControl(tab_content[pcw]);

	}
	
	@Override
	public void dispose() {
		//make sure we close cleanly
		for(int i=0; i<nbTab; i++){
			tab_content[i].dispose();
		}
		super.dispose();
	}
}
