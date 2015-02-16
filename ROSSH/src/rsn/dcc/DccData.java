package rsn.dcc;

public class DccData{

	public char[] hour = new char[3]; // dcc time when data was logged
	public char[] minute = new char[3]; // dcc time when data was logged
	public int frequency; // scanned frequency
	int max_pluse; // max number of pulse heard
	int signal_strength; // signal strength
	long timeStamp; // laptop time when data was read

	public DccData clone(){
		DccData newClone = new DccData();
		newClone.hour[0] = hour[0];newClone.hour[1] = hour[1];newClone.hour[2] = hour[2];
		newClone.minute[0] = minute[0];newClone.minute[1] = minute[1];newClone.minute[2] = minute[2];
		newClone.frequency = frequency;
		newClone.max_pluse = max_pluse;
		newClone.signal_strength = signal_strength;
		newClone.timeStamp = timeStamp;
		return newClone;
	}
	
	public int getFrequency(){
		return frequency;
	}
	public int getSignalStrength(){
		return signal_strength;
	}
	public long getTimeStamp(){
		return timeStamp;
	}
	public int getPulseCount(){
		return max_pluse;
	}
}
