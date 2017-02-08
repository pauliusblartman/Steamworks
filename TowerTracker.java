

import java.io.FileReader;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import edu.wpi.first.wpilibj.networktables.NetworkTable;


/**
 * 
 * @author Elijah Kaufman
 * @version 1.0
 * @description Uses opencv and network table 3.0 to detect the vision targets
 *
 */
//  greatly improved by TJ
//
//

public class TowerTracker {
	//This is not TJ code :(
	//But it is modified by TJ :D

	/**
	 * static method to load opencv and networkTables
	 */
	static{ 
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}
//	constants for the color RGB values
	public static final Scalar 
	RED = new Scalar(0, 0, 255),
	BLUE = new Scalar(255, 0, 0),
	GREEN = new Scalar(0, 255, 0),
	BLACK = new Scalar(0,0,0),
	YELLOW = new Scalar(0, 255, 255);
//	these are the threshold values in order 
	
//	the size for resizing the image
	public static final Size resize = new Size(320,240);
	
//	ignore these
	public static VideoCapture videoCapture;
	public static Mat matOriginal, matHSV, matThresh, clusters, matHeirarchy;
	
//	Constants for known variables
//	the height to the top of the target in first stronghold is 97 inches	
	public static int topTargetHeight;
//	the physical height of the camera lens
	public static int topCameraHeight;
//	projectile speed in mph
	public static double projectileSpeed;
	
//	camera details, can usually be found on the datasheets of the camera
	public static double verticleFOV;
	public static double horizontalFOV;
	public static double cameraAngle;
	
	public static NetworkTable table;
	static boolean networkAvailible = false;
	//CONSTANTS TO BE LOADED
	static Scalar 
	lowerBounds,
	upperBounds,
	lowerBounds_1,
	upperBounds_1;
	static String ip;
	static int averageInterval;
	static int cameraLocation;
	static double cameraOffset;
	static double inputDistance;
	static boolean calibrated;
	static int shootAngle = 0;
	
	static int mode = 0;
	
	public static boolean shouldRun = true;

	/**
	 * 
	 * @param args command line arguments
	 * just the main loop for the program and the entry points
	 */
	public static void main(String[] args) 
	{
		load();
		Object obj = new Object();
		synchronized(obj){
		NetworkTable.setIPAddress(ip);
		NetworkTable.setClientMode();
		table = NetworkTable.getTable("datatable");
		}
		try{
		Thread t100 = new Thread(new MultiDooer(0, table, cameraOffset, verticleFOV, horizontalFOV, topTargetHeight, topCameraHeight, cameraAngle, lowerBounds, upperBounds), "DankTank");
		t100.start();} catch(Exception e){e.printStackTrace();}
		try{
		Thread t1000 = new Thread(new MultiDooer(1, table, cameraOffset, verticleFOV, horizontalFOV, topTargetHeight, topCameraHeight, cameraAngle, lowerBounds, upperBounds), "DankBank");
		t1000.start();}catch(Exception e){e.printStackTrace();}
	}
	/**
	 * 
	 * reads an image from a live image capture and outputs information to the SmartDashboard or a file
	 */
	public static void load()
	{
		JSONParser parser = new JSONParser();
		try 
		{
			System.out.println("Attempting load...");
			JSONObject obj = (JSONObject)parser.parse(new FileReader("constants.txt"));
			ip = (String)obj.get("IP");
			lowerBounds = new Scalar(Math.toIntExact((long)obj.get("LOWER_BOUND_1")), Math.toIntExact((long)obj.get("LOWER_BOUND_2")), Math.toIntExact((long)obj.get("LOWER_BOUND_3")));
			upperBounds = new Scalar(Math.toIntExact((long)obj.get("UPPER_BOUND_1")), Math.toIntExact((long)obj.get("UPPER_BOUND_2")), Math.toIntExact((long)obj.get("UPPER_BOUND_3")));
			lowerBounds_1 = new Scalar(Math.toIntExact((long)obj.get("LOWER_BOUND_1-1")), Math.toIntExact((long)obj.get("LOWER_BOUND_2-1")), Math.toIntExact((long)obj.get("LOWER_BOUND_3-1")));
			upperBounds_1 = new Scalar(Math.toIntExact((long)obj.get("UPPER_BOUND_1-1")), Math.toIntExact((long)obj.get("UPPER_BOUND_2-1")), Math.toIntExact((long)obj.get("UPPER_BOUND_3-1")));
			cameraLocation = Math.toIntExact((long)obj.get("CAMERA_LOCATION"));
			topTargetHeight = Math.toIntExact((long)obj.get("TOP_TARGET_HEIGHT"));
			projectileSpeed = (double)((long)obj.get("PROJECTILE_SPEED"));
			cameraAngle = (double)obj.get("CAMERA_ANGLE");
			averageInterval = Math.toIntExact((long)obj.get("AVERAGE_INTERVAL"));
			verticleFOV = (double)obj.get("VERTICLE_FOV");
			horizontalFOV = (double)obj.get("HORIZONTAL_FOV");
			cameraOffset = (double)obj.get("CAMERA_OFFSET");
			System.out.println("Load complete!");
			
			
		} catch(Exception e)
		{
			System.out.println("ERROR! COULD NOT LOAD; SETTING TO DEFAULTS.");
			e.printStackTrace();
			ip = "10.39.41.6";
			lowerBounds = new Scalar(0,0,0);
			upperBounds = new Scalar(99,255,148);
			cameraLocation = 0;
			topTargetHeight = 97;
			projectileSpeed = 42;
			cameraAngle = 42.0;
			averageInterval = 5;
			verticleFOV = 33.58;
			horizontalFOV = 59.78;
			cameraOffset = 6;
			}
	}
}
