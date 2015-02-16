package rsn.dcc;
import static rsn.dcc.ROSParams.getParam;

public class DccParams {
	
	public static boolean simulate=false;
	public static final String DCC = "dcc";
	public static double sim_x = 0.0;
	public static double sim_y = 0.0;
	public static final String SIG_TOPIC = "/dcc/signal";
	public static final String FREQ_SRV = "/dcc/freq_change";
	public static final String RESET_SRV = "/dcc/reset";
	public String frequency_File = "./frequency.txt";
	public static int BAUD_RATE = 9600;
	public static int FREQUENCY = 49691;
	public static int FREQ_LENGTH = 5;
	public static String port = "/dev/null";
	public static int HUGE_TIME = 8000;
	public static int VERY_LONG_TIME = 5000;
	public static int LONG_TIME = 3000;
	public static int NORMAL_TIME = 2000;
	public static int SHORT_TIME = 1000;
	public static int VERY_SHORT_TIME = 200;
	public static int CONTINOUS_TIME = -1;
	public static String PGRM = "V";
	public static String SETUP = "A";
	public static String FREQ = "N";
	public static String TIME = "P";
	public static String ESC = "R";
	public static String LOG = "L";
	public static String SCAN_TIME = "2"; // scan rate for each
	// frequency in seconds
	public static String PATTERN_MATCHING = "0"; // don't use pattern
	// matching
	public static String MIN_PULSE_NO = "1";
	public static String MAX_PULSE_NO = "3"; // transmitter pulse
	// frequency is 54 ppm,
	// max_pulse =
	// 54*scantime(in min)
	// +delta
	public static String MIN_PULSE_RATE = "54"; // find the values
	public static String MAX_PULSE_RATE = "56"; // find the values
	public static String LOWEST_RECV_FREQ = "48"; // lowest receiver
	public static boolean stopReading = true;
	public static boolean waitForData = false;
	public static boolean initProgram = false;
	public static boolean setVars = false;
	public static boolean isDCCFull = false;
	public static boolean verbose = false;
	
	/**
	 * Returns the number of parameters successfully retrieved
	 * @return
	 */
	public static void loadFromRos(){
		sim_x = getParam(DCC,"sim_x",sim_x);
		sim_y = getParam(DCC,"sim_y",sim_y);
		simulate = getParam(DCC,"simulate",simulate);
		FREQUENCY = getParam(DCC,"frequency",FREQUENCY);
		SCAN_TIME = getParam(DCC,"scan_time",SCAN_TIME);
		port = getParam(DCC,"port",port);
		initProgram = getParam(DCC,"initProgram",initProgram);
		setVars = getParam(DCC,"setVars",setVars);
		isDCCFull = getParam(DCC,"isDCCFull",isDCCFull);
	}

}
