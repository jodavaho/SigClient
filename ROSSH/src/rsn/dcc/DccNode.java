package rsn.dcc;

import static rsn.dcc.DccParams.BAUD_RATE;
import static rsn.dcc.DccParams.CONTINOUS_TIME;
import static rsn.dcc.DccParams.ESC;
import static rsn.dcc.DccParams.FREQ;
import static rsn.dcc.DccParams.FREQUENCY;
import static rsn.dcc.DccParams.FREQ_SRV;
import static rsn.dcc.DccParams.HUGE_TIME;
import static rsn.dcc.DccParams.LOG;
import static rsn.dcc.DccParams.LONG_TIME;
import static rsn.dcc.DccParams.LOWEST_RECV_FREQ;
import static rsn.dcc.DccParams.MAX_PULSE_NO;
import static rsn.dcc.DccParams.MAX_PULSE_RATE;
import static rsn.dcc.DccParams.MIN_PULSE_NO;
import static rsn.dcc.DccParams.MIN_PULSE_RATE;
import static rsn.dcc.DccParams.NORMAL_TIME;
import static rsn.dcc.DccParams.PATTERN_MATCHING;
import static rsn.dcc.DccParams.PGRM;
import static rsn.dcc.DccParams.RESET_SRV;
import static rsn.dcc.DccParams.SCAN_TIME;
import static rsn.dcc.DccParams.SETUP;
import static rsn.dcc.DccParams.SHORT_TIME;
import static rsn.dcc.DccParams.SIG_TOPIC;
import static rsn.dcc.DccParams.VERY_LONG_TIME;
import static rsn.dcc.DccParams.initProgram;
import static rsn.dcc.DccParams.isDCCFull;
import static rsn.dcc.DccParams.loadFromRos;
import static rsn.dcc.DccParams.port;
import static rsn.dcc.DccParams.setVars;
import static rsn.dcc.DccParams.simulate;
import static rsn.dcc.DccParams.stopReading;
import static rsn.dcc.DccParams.waitForData;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Iterator;
import java.util.TooManyListenersException;
import java.util.Vector;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import ros.NodeHandle;
import ros.Publisher;
import ros.Ros;
import ros.RosException;
import ros.ServiceServer;
import ros.pkg.dcc.msg.RosDccData;
import ros.pkg.dcc.srv.FreqChange;
import ros.pkg.dcc.srv.FreqChange.Request;
import ros.pkg.dcc.srv.FreqChange.Response;
import ros.pkg.std_msgs.msg.Int32;
import ros.pkg.std_srvs.srv.Empty;


public class DccNode implements SerialPortEventListener, ServiceServer<FreqChange.Request,FreqChange.Response,FreqChange>{

	Ros ros;
	long seq = 0;
	NodeHandle nh;
	String gBuffer;
	int tableInUse=2;
	int singleTable=-1;
	SerialPort dccPort;
	Vector<Integer> listTable;
	boolean startupComplete=false;
	char []tempFreq = new char[8];
	Publisher<RosDccData> dccDataOut;
	DccData tempDccData = new DccData();
	long starttime=System.currentTimeMillis()/1000;
	ServiceServer.Callback<Empty.Request,Empty.Response> resetHandler;
	ServiceServer.Callback<FreqChange.Request,FreqChange.Response> freqHandler;
	final static int LIST_TABLE = 1, SINGLE_TABLE = 2;

	/**
	 * Creates a new DCC collector object
	 * 
	 * @param listeners
	 * @param dccPort
	 */
	private DccNode(){
		ros = Ros.getInstance();
		if (!ros.isInitialized()){
			ros.init("dcc");
		}
		loadFromRos();
		nh = ros.createNodeHandle();
		rdd = new RosDccData();
		listTable = new Vector<Integer>();
	}

	private boolean startup(){
		//subscribe to data 
		//subscribe to .... something
		gBuffer ="";
		dccPort = null;
		try {
			// Change frequence
			freqHandler = new ServiceServer.Callback<FreqChange.Request,FreqChange.Response>() {
				@Override
				public Response call(Request request) {
					int ff= request.desired;
					int sz=request.desired_list.length;
					FreqChange c =new FreqChange();
					FreqChange.Response r = c.createResponse();
					Int32 i = new Int32();
					if (sz>0){
						ros.logInfo("Heard request for: "+request.desired_list.length+" frequencies ... Honoring.");
						setFrequencies(request.desired_list);
						i.data = (short) (request.desired_list.length);
					} else {
						ros.logInfo("Heard request for: frequency: "+request.desired+"... Honoring.");
						ff = setFrequency(Integer.toString(request.desired));
						i.data = (short) ff;
					}
					r.setTo(i);
					ros.logInfo("Freq change request done");
					return r;
				}
			};

			// Reset all configuration params in dcc
			resetHandler = new ServiceServer.Callback<Empty.Request,Empty.Response>() {

				@Override
				public Empty.Response call(Empty.Request request) {
					Empty.Response r = new Empty.Response();
					ros.logInfo("Dcc reset requested");
					resetDcc(true); //force erase

					// frequencies tables are erased, so write frequencies again
					if(tableInUse==SINGLE_TABLE) { //single frequency
						int prevFreq = singleTable; //copy current frequency
						setFrequency(Integer.toString(48000)); //force to write
						setFrequency(Integer.toString(prevFreq)); //set it back
					} else if (tableInUse==LIST_TABLE){ //list
						Vector<Integer> prevList = listTable;
						Integer[] dummyList = new Integer[1];
						dummyList[0] = 48000;
						setFrequencies(dummyList);
						setFrequencies((Integer[])prevList.toArray());
					}
					startLogging(tableInUse); //Start logging again
					ros.logInfo("DCC reset request done");
					return r;
				}
			};

			// Let others know
			this.dccDataOut = nh.advertise(SIG_TOPIC, new RosDccData(), 1);
			nh.advertiseService(FREQ_SRV, new FreqChange(), freqHandler);
			nh.advertiseService(RESET_SRV, new Empty(), resetHandler);

		} catch (RosException e1) {
			ros.logError("Could not publish! Perishing!");
			e1.printStackTrace();
			return false;
		}

		//advertise the set freq
		ros.logInfo("DCC starting up with port="+port);
		if (port.contains("null") || simulate){
			dccPort=null;
			ros.logInfo("Startup Done .... NULL port opened.");
			startupComplete = true;

			writeDCC(ESC, LONG_TIME);
			stopReading = true;
			setFrequency(Integer.toString(FREQUENCY));
			stopReading = false;
			waitForData = false;
			return true;
		} 
		dccPort = SerialUtils.openPort(port, BAUD_RATE, 2000);

		try {
			dccPort.addEventListener(this);
		} catch (TooManyListenersException e) {
			ros.logError("Too many listeners");
			System.exit(-1);
			e.printStackTrace();
		}

		writeDCC(ESC, LONG_TIME);
		stopReading = true;

		setFrequency(Integer.toString(FREQUENCY));

		resetDcc(false);

		ros.logInfo("DCC startup done");
		ros.logInfo("Starting logging...");
		startLogging(2);
		stopReading = false;
		waitForData = false;
		ros.logInfo("Startup Done");
		startupComplete = true;
		return true;
	}
	private void startLogging(int table) {
		tableInUse = table; //store which table is currently being logged
		writeDCC(ESC, LONG_TIME);
		writeDCC(LOG, LONG_TIME);
		writeDCC(Integer.toString(table)+"\r", CONTINOUS_TIME);// from frequency table 2
	}

	/** Resets dcc based on rosparams
	 * @param force If TRUE then does not check rosparams and forces reset
	 */
	private void resetDcc(boolean force){

		if (initProgram || force){
			writeDCC( ESC, SHORT_TIME);
			writeDCC( PGRM, LONG_TIME);
			writeDCC( "0\r", LONG_TIME);// Presence/Absence program
			writeDCC( "1\r", SHORT_TIME); // multiple antennas
			writeDCC( "1\r", LONG_TIME);// number of antennas
			writeDCC( "0\r", SHORT_TIME);// don't set max signal strength
		}
		if (setVars || force){
			writeDCC( ESC, LONG_TIME);
			writeDCC( SETUP, SHORT_TIME);
			writeDCC( SCAN_TIME, SHORT_TIME); // scan time for each frequency
			writeDCC( "\r", LONG_TIME);
			writeDCC( "0\r", LONG_TIME); // don't use timeout
			writeDCC( "0\r", LONG_TIME); // log continuosly
			writeDCC( "0\r", LONG_TIME); // store continously
			writeDCC( "0\r", LONG_TIME ); // log number of pulses
			writeDCC( PATTERN_MATCHING, SHORT_TIME);
			writeDCC( "\r", LONG_TIME);
			if(PATTERN_MATCHING.charAt(0) == '0'){
				// dont use pattern matching
				writeDCC( MIN_PULSE_NO, SHORT_TIME);
				writeDCC( "\r", LONG_TIME);
				writeDCC( MAX_PULSE_NO, SHORT_TIME);
				writeDCC( "\r", LONG_TIME);
			}else{
				writeDCC( MIN_PULSE_RATE, SHORT_TIME);
				writeDCC( "\r", LONG_TIME);
				writeDCC( MAX_PULSE_RATE, SHORT_TIME);
				writeDCC( "\r", LONG_TIME);
			}
			writeDCC( LOWEST_RECV_FREQ, SHORT_TIME); // set lowest receiver frequency
			writeDCC( "\r", LONG_TIME );
			writeDCC( "0", SHORT_TIME); // don't log reference transmitter
			writeDCC( "\r", HUGE_TIME); 
			// figure out if there is data in DCC
			if(isDCCFull || force){
				writeDCC( "1\r", VERY_LONG_TIME ); // erase previous data
			}
			writeDCC( "\r",  SHORT_TIME); //time
			writeDCC( "\r", SHORT_TIME);
			writeDCC( "\r", SHORT_TIME);
			writeDCC( "\r", SHORT_TIME);
			writeDCC( "\r", LONG_TIME);
		}
	}

	private boolean localShutdown(){
		if (dccPort == null){
			return false;
		}
		writeDCC(ESC, LONG_TIME);
		stopReading = true;
		//kill subscriptions and advertising.
		try {
			dccPort.removeEventListener();
			dccPort.close();
		} catch (Exception e) {
			ros.logError("Couldn't cose Serial Port: DCC\n");
			ros.logError(e.getMessage());
		}
		return false;
	}

	@Override
	public void serialEvent(SerialPortEvent ev) {
		if (ev.getEventType()==SerialPortEvent.DATA_AVAILABLE){
			//ros.logInfo("Data available!");
			try {
				InputStream in = dccPort.getInputStream();
				int avail = in.available();
				byte[] tempBuf = new byte[avail>0? avail:255];
				in.read(tempBuf);
				gBuffer = gBuffer+new String(tempBuf);
				if (startupComplete){
					parseBuffer(gBuffer.length());
				} else {
					ros.logDebug(gBuffer);
					gBuffer="";
				}
			} catch (IOException e) {
				e.printStackTrace();
			}
		} else {
			ros.logInfo("ignored sp event of type:"+ev.getEventType());
		}


	}

	private int writeDCC(String message, int waitTime) {
		if (simulate){
			//			ros.logInfo("Simulating: "+String.valueOf(message));
			return 0;
		}
		else if (dccPort==null){
			//			ros.logInfo("NULL port; cannot write:"+String.valueOf(message));
			return 0;
		} 
		int count = 0;
		//ros.logInfo("writeDCC: " + message + " Length: "+message.length());
		try {
			OutputStream out = dccPort.getOutputStream();
			for (char c:message.toCharArray()){
				out.write((byte)c);
				count++;

				Thread.sleep(200);
			}
			if (waitTime>0){
				Thread.sleep(waitTime);
			}

			return count;

		} catch (InterruptedException e){
			e.printStackTrace();
			ros.logError("Problem writing to output stream in DCC NODE");
			return count;
		} catch (IOException e) {
			e.printStackTrace();
			ros.logError("Problem writing to output stream in DCC NODE");
			return count;
		}

	}

	RosDccData rdd;
	// formats the dcc data to dcc_data struct and sets the global current_data buffer
	private void parseBuffer( int length){
		int i;

		int indColon=-1, indA=-1, indSlash=-1, indHash =-1, indCloseBrace =-1;

		int indexS = gBuffer.indexOf('S');
		//		RosDccData rdd = new RosDccData();
		//		ros.logInfo("BUF:\n"+gBuffer+"\n");
		//gBuffer[length]=0;

		waitForData= true;
		for(i=0; i< length; i++){
			if(gBuffer.charAt(i) == '/') indSlash = i;
			else if(gBuffer.charAt(i) == ':') indColon = i;
			else if(gBuffer.charAt(i) == 'A') indA = i; 
			else if  (gBuffer.charAt(i) == '#') indHash = i;
			else if  (gBuffer.charAt(i) == ')') indCloseBrace = i;
		}

		if(indSlash != -1 && length-indSlash > 2){
			tempDccData.hour[0] = gBuffer.charAt(indSlash+1);
			tempDccData.hour[1] = gBuffer.charAt(indSlash+2);
			tempDccData.hour[2]=0;
		}

		if(indColon != -1 && length-indColon > 2){
			tempDccData.minute[0] = gBuffer.charAt(indColon+1);
			tempDccData.minute[1] = gBuffer.charAt(indColon+2);
			tempDccData.minute[2]=0;
		}
		if(indColon != -1 && indColon > 1){
			tempDccData.hour[0] = gBuffer.charAt(indColon-2);
			tempDccData.hour[1] = gBuffer.charAt(indColon-1);
			tempDccData.hour[2]=0;
		}
		if(indA != -1 && indA > 3){
			tempDccData.minute[0] = gBuffer.charAt(indA-4);
			tempDccData.minute[1] = gBuffer.charAt(indA-3);
			tempDccData.minute[2]=0;
		}

		if(indCloseBrace !=-1){
			for(i = 0 ; i< (length- indCloseBrace -2) && i < 8; i++){
				tempFreq[i] = gBuffer.charAt(i+indCloseBrace +2);	
			}
		}	
		if(indHash !=-1){
			for(i =7; i>=0; i--){
				if(indHash-i>= 0) tempFreq[7-i]=gBuffer.charAt(indHash-i);
			}
			String temp2 = new String(tempFreq);
			//			ros.logInfo("temp2:"+temp2);
			int k=0;
			temp2=temp2.trim();
			//			ros.logInfo("temp2 is now:"+temp2);
			for( k=0; k<temp2.length(); k++){
				if(temp2.charAt(k)<'0' || temp2.charAt(k)>'9'  ){
					break;
				}
			}
			if(k>0){
				try{
					tempDccData.frequency = Integer.parseInt(temp2.substring(0, k));
					//				ros.logInfo("Found int:"+tempDccData.frequency);
					rdd.frequency=tempDccData.frequency;
				}catch (NumberFormatException e){
					ros.logWarn("Number format exception"+e);
				}
				//			}else{
				//				tempDccData.frequency = 0;
			}

		}


		if(indexS !=-1){
			String temp = gBuffer.substring(indexS+3);
			if (temp.length()>=3){
				//				ros.logInfo("TEMP(s):"+temp);
				int tLen = temp.length()<4?temp.length():4;
				int ss=0;
				try{
					int k=0;
					String temp2 = temp.substring(0, tLen);
					ros.logDebug(temp2);
					for( k=0; k<temp2.length(); k++){
						if(temp2.charAt(k)<'0' || temp2.charAt(k)>'9'  ) {
							break;
						}
					}
					if(k>0){
						ss = Integer.parseInt(temp2.substring(0,k));
					}else{
						ss = 0;
					}
				}catch(NumberFormatException e){
					e.printStackTrace();
				}
				long timeStamp = System.currentTimeMillis();

				// Check if no. of pulses is within the range
				String pulses=gBuffer.substring(indexS-4,indexS-1);
				// No. of pulses when zero the string is "   0" with some no. of whitespaces
				// If nonzero, then the string is "  4s" where 4 is an example no. of pulses
				boolean validPulses=false;
				try {
					if(pulses.contains("s")) {
						//find no. of pulses
						int start=-1;
						for (int p=0; p<pulses.indexOf('s'); p++) {
							if(pulses.charAt(p) != ' ') {
								start = p;
							}
						} 
						if(start != -1) {
							int numPulses = Integer.parseInt(pulses.substring(start,pulses.indexOf('s')));
							if(numPulses >= Integer.parseInt(MIN_PULSE_NO) && numPulses <= Integer.parseInt(MAX_PULSE_NO)) {
								validPulses = true;
							} else {
								ros.logWarn(Integer.toString(numPulses) + "pulses received, which is invalid.");
							}
						}
					}  
				} catch (NumberFormatException e) {
					ros.logWarn("No. format exception");
				}

				// Fill in dcc topic data
				rdd.header.stamp.secs=(int) ((timeStamp/1000)-starttime);
				rdd.header.stamp.nsecs=0;

				rdd.header.seq=seq++;
				rdd.header.frame_id="dcc";

				if(validPulses) {
					rdd.strength = ss;
				} else {
					rdd.strength = 0;
				}

				dccDataOut.publish(rdd.clone());
				gBuffer=gBuffer.substring(indexS+2);
				//			gBuffer = "";
			}
		} else {
			//			ros.logInfo("-1");
		}

	}
	private void setFrequencies(Integer[] array){
		String[] sa = new String[array.length];
		for(int i=0;i<array.length;i++){
			sa[i] = Integer.toString(array[i]);
		}
		setFrequencies(sa);
	}

	/**
	 * Yeah, backwards compatability.
	 * @param array
	 */
	private void setFrequencies(int[] array){
		String[] sa = new String[array.length];
		for(int i=0;i<array.length;i++){
			sa[i] = Integer.toString(array[i]);
		}
		setFrequencies(sa);
	}
	/**
	 * sets the frequencies to scan from the frequency @param array
	 */
	private void setFrequencies(String[] array) {

		// Convert to int vector
		Vector<Integer> intArray = new Vector<Integer>();
		for (int i=0; i<array.length; i++) {
			try {
				intArray.add(Integer.parseInt(array[i]));
			} catch (NumberFormatException e) {
				ros.logError("Invalid frequency number passed! Won't write to dcc");
				return;
			}
		}

		boolean reset=false;
		// Check if all requested are in the list
		for (int i=0; i<array.length; i++) {
			try {
				if(!listTable.contains(Integer.parseInt(array[i]))) {
					reset = true;
					break;
				}
			} catch (NumberFormatException e) {
				ros.logError("Invalid frequency number passed! Won't write to dcc");
				return;
			}
		}
		// Check if all in the list are requested
		Iterator<Integer> iter = listTable.iterator();
		while(iter.hasNext()) {
			if(!intArray.contains(iter.next())) {
				reset = true;
				break;
			}
		}

		if(reset) {
			ros.logInfo("Resetting current frequency to program the list");
			listTable.clear();
			writeDCC(ESC, LONG_TIME);
			writeDCC(FREQ, LONG_TIME);
			writeDCC("1\r", SHORT_TIME);// Frequency Table 1
			writeDCC(String.valueOf(array.length), SHORT_TIME);// Number of
			ros.logInfo("writing frequency list");
			// Frequencies
			writeDCC("\r", SHORT_TIME);
			for (int i = 0; i < array.length; i++) {
				ros.logInfo("dcc writing: "+array[i]);
				writeDCC(array[i], SHORT_TIME);// Frequency
				writeDCC("\r", SHORT_TIME);
				listTable.add(Integer.parseInt(array[i])); //Store in table
			}
		} else {
			ros.logInfo("DCC is already scanning on the same list!");
			ros.logInfo("Will simply start logging again.");
		}
		startLogging(LIST_TABLE);
		ros.logInfo("dcc: Done");
	}

	/**
	 * Writes @param freq to the DCC
	 */
	private int setFrequency(String freq) {
		int count = 0;
		if (Integer.parseInt(freq)==singleTable){
			ros.logInfo("Heard request for "+freq+" but it equals "+singleTable);
			ros.logInfo("Will start logging without rewriting.");
			count = 0;
		} else {
			ros.logInfo("Heard request for "+freq+" and previous was "+singleTable);
			singleTable=Integer.parseInt(freq);
			count = writeDCC(ESC, LONG_TIME);
			count += writeDCC(FREQ, LONG_TIME);
			count += writeDCC("2\r", NORMAL_TIME);// Frequency table 2
			count += writeDCC("1\r", SHORT_TIME);// Number of Frequencies
			count += writeDCC(freq, SHORT_TIME);// Frequency
			count += writeDCC("\r", SHORT_TIME);// Close DCC
			ros.logInfo("Frequency ("+freq+") sent");
		}
		startLogging(SINGLE_TABLE);
		return count;
	}

	public void spin(){
		ros.spin();
	}

	@Override
	public String getService() {
		//		ros.logInfo("get service called");
		return "/dcc/freq_change";
	}

	@Override
	public boolean isValid() {
		//		ros.logInfo("IS valid called");
		return true;
	}

	@Override
	public void shutdown() {
		//		ros.logInfo("Shutdown Called");	
	}

	/**
	 * Main method
	 */
	public static void main(String[] args){		
		DccNode dc = new DccNode();
		Vector<Integer> freqs = new Vector<Integer>();
		for (String s: args){
			//			System.out.println("Found arg: "+s);
			if (s.matches("[0-9]*")){
				dc.ros.logInfo("Found freq request: "+s);
				//				DccParams.FREQUENCY=Integer.parseInt(s);
				freqs.add(Integer.parseInt(s));
			} else if (s.matches("/[a-zA-Z]*/[a-zA-Z]*[0-9]*")){
				dc.ros.logInfo("Found port message: "+s);
				DccParams.port=s;
			} else if (s.matches("(simulate|-s)")){
				dc.ros.logInfo("DCC is Simulating!");
				DccParams.simulate=true;
			} else if (s.matches("[0-9]*.?[0-9]*,[0-9]*.?[0-9]*")){
				String[] ss = s.split(",");
				DccParams.sim_x=Double.parseDouble(ss[0]);
				DccParams.sim_y=Double.parseDouble(ss[1]);
				dc.ros.logInfo("Sim tag location set to "+DccParams.sim_x+","+DccParams.sim_y);
			}
		}

		dc.startup();
		if (freqs.size()>0){
			dc.ros.logInfo("Scanning "+freqs.size()+" frequencies");
			Integer[] tt = new Integer[freqs.size()];
			freqs.toArray(tt);
			dc.setFrequencies(tt);
		}
		dc.spin();
		dc.localShutdown();
		dc.ros.logInfo("Closing DCC");
	}
}


