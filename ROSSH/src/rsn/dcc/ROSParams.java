package rsn.dcc;

import ros.NodeHandle;
import ros.Ros;
import ros.RosException;

/**
 * A helper class to get parameters from ROS. this is NOT the place to create
 * a parameter class.
 * 
 * @author joshua
 *
 */
public class ROSParams {
	
	public static final String ROS_NAMESPACE="";
//	public static final String JOY="/rsn/input/joy";
	
	private static Ros ros = Ros.getInstance();
	/**
	 * Tries to retrieve a param. On fail: returns default.
	 * @param name
	 * @return
	 */
	public static int getParam(String namespace, String name, int def) {
		if (!ros.isInitialized()){
			System.out.println("Cannot retrieve: "+name+". Ros not initialized");
			return def;
		}
		NodeHandle nh = ros.createNodeHandle(namespace);
		if (!nh.hasParam(name)){
			System.out.println("parameter not found: "+name);
			return def;
		}
		try{
			int t =nh.getIntParam(name);
			System.out.println("Found: "+name+"="+t);
			return t;
		} catch (RosException e){
			System.out.println("error retrieving parameter: "+name);
			return def;
		}
	}
	/**
	 * Tries to retrieve a param. On fail: returns default.
	 * @param name
	 * @return
	 */
	public static String getParam(String namespace, String name, String def) {
		if (!ros.isInitialized()){
			System.out.println("Cannot retrieve: "+name+". Ros not initialized");
			return def;
		}
		NodeHandle nh = ros.createNodeHandle(namespace);
		if (!nh.hasParam(name)){
			System.out.println("parameter not found: "+name);
			return def;
		}
		try{
			String t =nh.getStringParam(name);
			System.out.println("Found: "+name+"="+t);
			return t;
		} catch (RosException e){
			System.out.println("error retrieving parameter: "+name);
			return def;
		}
	}
	/**
	 * Tries to retrieve a param. On fail: returns default.
	 * @param name
	 * @return
	 */
	public static double getParam(String namespace, String name, double def) {
		if (!ros.isInitialized()){
			System.out.println("Cannot retrieve: "+name+". Ros not initialized");
			return def;
		}
		NodeHandle nh = ros.createNodeHandle(namespace);
		if (!nh.hasParam(name)){
			System.out.println("parameter not found: "+name);
			return def;
		}
		try{
			double t =nh.getDoubleParam(name);
			System.out.println("Found: "+name+"="+t);
			return t;
		} catch (RosException e){
			System.out.println("error retrieving parameter: "+name);
			return def;
		}
	}
	/**
	 * Tries to retrieve a param. On fail: returns default.
	 * @param name
	 * @return
	 */
	public static boolean getParam(String namespace, String name, boolean def) {
		if (!ros.isInitialized()){
			System.out.println("Cannot retrieve: "+name+". Ros not initialized");
			return def;
		}
		NodeHandle nh = ros.createNodeHandle(namespace);
		if (!nh.hasParam(name)){
			System.out.println("parameter not found: "+name);
			return def;
		}
		try{
			boolean t =nh.getBooleanParam(name);
			System.out.println("Found: "+name+"="+t);
			return t;
		} catch (RosException e){
			System.out.println("error retrieving parameter: "+name);
			return def;
		}
	}
}

