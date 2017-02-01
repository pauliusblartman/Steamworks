import java.awt.Color;
import java.util.ArrayList;
import java.util.Date;
import java.util.Iterator;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class MultiDooer extends Thread
{
	{System.loadLibrary(Core.NATIVE_LIBRARY_NAME);}
	private VideoCapture videoCapture;
	private Mat matOriginal, matThresh, matHeirarchy;
	private Scalar lowerBounds, upperBounds;
	private int id;
	Object obj = new Object();
	private NetworkTable table;
	private int averageInterval = 5;
	private double cameraOffset, verticleFOV, horizontalFOV, topTargetHeight, topCameraHeight, cameraAngle;
	
	public MultiDooer( int ID, NetworkTable Table, double CameraOffset, double VerticleFoV, double HorizontalFoV, double TopTargetHeight, double TopCameraHeight, double CameraAngle, Scalar LowerBounds, Scalar UpperBounds) 
	{
		id = ID;
		System.out.println("Playful " + id);
		Object obj = new Object();
		synchronized(obj){
		table = Table;
		}

		
		cameraOffset = CameraOffset;
		verticleFOV = VerticleFoV;
		horizontalFOV = HorizontalFoV;
		topTargetHeight = TopTargetHeight;
		topCameraHeight = TopCameraHeight;
		cameraAngle = CameraAngle;
		lowerBounds = LowerBounds;
		upperBounds = UpperBounds;
		System.out.println("Broken Dreams");
		//table = NetworkTable.getTable("datatable");
		//if(id != 0)
		//	table = NetworkTable.getTable("datable" + id);
		System.out.println("Tanking the danking");
		matOriginal = new Mat();
		matThresh = new Mat();
		matHeirarchy = new Mat();
			try {
				videoCapture = new VideoCapture(id);
				videoCapture.open(id);
				while(!videoCapture.isOpened()){}
			} catch (Exception e) {
				e.printStackTrace();
			}
		System.out.println("Dank memes");
//		make sure the java process quits when the loop finishes
	}
	public void run() {
		try{
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		ArrayList<MatOfPoint> highTarget = new ArrayList<MatOfPoint>();
		
		ArrayList<Rect> arrayRect = new ArrayList<Rect>();
		
		double y,targetX,distance,azimuth;//		frame counter
		int FrameCount = 0;
		Date date1 = new Date();

//		only run for the specified time
		int goodValues = 0;
		double averageAzimuth = 0, averageDistance = 0;
		boolean frameStop = true;
		while(frameStop){
			Date date2 = new Date();
			System.out.println("Running:"+id);
			//if ( averageInterval != 0 && FrameCount % averageInterval == 0)
			//{
				
				if (goodValues != 0)
				{
					double	averageAzimuthOut = averageAzimuth / goodValues;
					double	averageDistanceOut = averageDistance / goodValues;
					double  paralaxAdjustment = 90 - (Math.toDegrees(Math.atan(averageDistanceOut/cameraOffset)));
					averageAzimuthOut -= paralaxAdjustment;
					System.out.println("Average Azimuth: " + averageAzimuthOut);
					System.out.println("Average Distance: " + averageDistanceOut);
					//Imgcodecs.imwrite("output" + id + ".png", matOriginal);
					//Imgcodecs.imwrite("output" + id + "a" + ".png", matThresh);
					date1 = new Date();
					Object obj = new Object();
					synchronized(obj)
					{
						table.putNumber("averageAzimuthOut-" + id, averageAzimuthOut);
					}
					
				}
				// try that thing later
				goodValues = 0;
				averageAzimuth =0;
				averageDistance = 0;
			//}
			contours.clear();
	        highTarget.clear();
//			capture from the axis camera
			videoCapture.read(matOriginal);	
//			captures from a static file for testing
//			matOriginal = Imgcodecs.imread("someFile.png");
			//Mat matHSV = new Mat();
			//Imgproc.cvtColor(matOriginal, matHSV, Imgproc.COLOR_BGR2HSV);
			Core.inRange(matOriginal, lowerBounds, upperBounds, matThresh);
			//Imgcodecs.imwrite("output1.png", matThresh);
			Imgproc.findContours(matThresh, contours, matHeirarchy, Imgproc.RETR_EXTERNAL, 
					Imgproc.CHAIN_APPROX_SIMPLE);
//			make sure the contours that are detected are at least 20x20 
//			pixels with an area of 400 and an aspect ration greater then 1
//			Get the widest one
			int widestWidth = 0;
			MatOfPoint bestResult = null;
			System.out.println("Blah" + contours.size());
			for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) {
				MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
				Rect rec = Imgproc.boundingRect(matOfPoint);
					if(rec.height < 5 || rec.width < 5){
						iterator.remove();
					continue;
					}
					float aspect = (float)rec.width/(float)rec.height;
					
					if(rec.width > rec.height)
					{
						highTarget.add(matOfPoint);
						iterator.remove();
						continue;
					}
					//System.out.println("Stupidathting"+aspect);
					/*if(aspect < 1.0){
						iterator.remove();
						continue;
					}
					else if(aspect > 4.0){
						//  we should remove if it is too wide compared to the height
						iterator.remove();
						continue;
					}
					
					*/
					//  check the area of the contour compared to the perimeter
					MatOfPoint2f doubleMat = new MatOfPoint2f (matOfPoint.toArray());
					double contourPerimeter = Imgproc.arcLength(doubleMat, true);
					double contourArea = Imgproc.contourArea(matOfPoint);
					double PARatio = contourPerimeter / contourArea;
					//System.out.println("PARatio " + PARatio);
					if(PARatio > 0.65 || PARatio < 0.025)
					{
						iterator.remove();
						continue;
					}
					if(rec.width > widestWidth ){
						widestWidth = rec.width;
						bestResult = matOfPoint;
					}
					
					
					//double contourPerimeter = rec.arc
					//  remove if ratio perimeter  to area is out of bounds					
					// check the hull convex defects 
					//and remove any where the defect is not > .7 height
				} 
			for (Iterator<MatOfPoint> iterator = highTarget.iterator(); iterator.hasNext();)
			{
				MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
				MatOfPoint2f doubleMat = new MatOfPoint2f (matOfPoint.toArray());
				Rect rec = Imgproc.boundingRect(matOfPoint);
				double contourPerimeter = Imgproc.arcLength(doubleMat, true);
				double contourArea = Imgproc.contourArea(matOfPoint);
				double PARatio = contourPerimeter / contourArea;
				//System.out.println("PARatio " + PARatio);
				
				if(PARatio > 2.1 || PARatio < 1.56909091)
				{
					iterator.remove();
					continue;
				}
				/*if(contourArea < 200)
				{
					iterator.remove();
					continue;
				}
				
				if(contourPerimeter < 100)
				{
					iterator.remove();
					continue;
				}
				
				if(rec.width < 30 || rec.width > 200)
				{
					iterator.remove();
					continue;
				}
				
				if(rec.height < 10 || rec.height > 100)
				{
					iterator.remove();
					continue;
				}*/
				float aspect = (float)rec.width/(float)rec.height;
				if(aspect < 2 || aspect > 2.4)
				{
					iterator.remove();
					continue;
				}
			}
				for(MatOfPoint mop : contours){
					Rect rec = Imgproc.boundingRect(mop);
					Imgproc.rectangle(matOriginal, rec.br(), rec.tl(), new Scalar(0, 0, 256));
			}
				if(highTarget.size() != 0){
				for(MatOfPoint mop : highTarget){
					Rect rec = Imgproc.boundingRect(mop);
					
					Imgproc.rectangle(matOriginal, rec.br(), rec.tl(), new Scalar(0, 256, 0));}}
			
//			if there is only 1 target, then we have found the target we want
			//if(contours.size() == 1){
			System.out.println("Conty " + contours.size());
			if(bestResult != null && contours.size() > 1){
				//Rect rec = Imgproc.boundingRect(contours.get(0));
				//bestResult.
				//find the best points for the rectangle
				int lx = matOriginal.width(), ly = matOriginal.height(), lw = matOriginal.width(), lh = matOriginal.height();
				int xz = 0, yz = 0;
				for(int a = 0; a < contours.size(); a++)
				{
					MatOfPoint mop = contours.get(a);
					Rect rect = Imgproc.boundingRect(mop);
					if(rect.x <= lx)
						lx = rect.x;
					if(rect.y <= ly)
						ly = rect.y;
					if(rect.width + rect.x >= xz){
						xz = rect.x + rect.width;}
					if(rect.height + rect.y >= yz){
						yz = rect.y + rect.height;}
					lw = xz -lx;
					lh = yz - ly;
					Rect rectx = rect;
					if(contours.get(a + 1) != null)
					{
						rectx = Imgproc.boundingRect(contours.get(a + 1));
					} 
					
					if(Math.abs(rect.x - rectx.x) > lh * 3 || contours.get(a + 1) == null)
					{
						Rect rec = new Rect(lx,ly,lw,lh);
						Imgproc.rectangle(matOriginal, rec.br(), rec.tl(), new Scalar(256, 0, 0));
						lx = matOriginal.width();
						ly = matOriginal.height();
						lw = matOriginal.width();
						lh = matOriginal.height();
						arrayRect.add(rec);
					}
				}
				
				//lw = xz -lx;
				//lh = yz - ly;
				System.out.println("Cords " + lx + " " + ly + " " + xz + " " + yz + " ");
				Rect rec = new Rect(lx,ly,lw,lh);
				int widestRect= 0;
				for(Rect rect : arrayRect)
				{
					if(rect.width > widestRect)
					{
						rec = rect;
						widestRect = rect.width;
					}
				}
				//Imgproc.rectangle(matOriginal, rec.br(), rec.tl(), new Scalar(256, 0, 0));
//				"fun" math brought to you by miss daisy (team 341)!
				y = rec.br().y + rec.height / 2;
				y = -((2 * (y / matOriginal.height())) - 1);
				distance = (topTargetHeight - topCameraHeight) / 
						Math.tan((y * verticleFOV / 2.0 + cameraAngle) * Math.PI / 180);
//				angle to target...would not rely on this
				
				targetX = rec.tl().x + rec.width / 2;
				targetX = (2 * (targetX / matOriginal.width())) - 1;
				azimuth = normalize360(targetX*horizontalFOV / 2.0 + 0);
//				drawing info on target
				Point center = new Point(rec.br().x-rec.width / 2 - 15,rec.br().y - rec.height / 2);
				Point centerw = new Point(rec.br().x-rec.width / 2 - 15,rec.br().y - rec.height / 2 - 20);
				Imgproc.putText(matOriginal, ""+(int)distance, center, Core.FONT_HERSHEY_PLAIN, 1, new Scalar(0, 0, 256));
				Imgproc.putText(matOriginal, ""+(int)azimuth, centerw, Core.FONT_HERSHEY_PLAIN, 1, new Scalar(0, 256, 0));
				// averaging
				goodValues += 1;
				averageAzimuth += azimuth;
				averageDistance += distance;
				
			}
			System.out.println("Blankety");
			Object obj = new Object();
			synchronized(obj)
			{
			table.putNumber("sinceLastUpdate-" + id, (date2.getTime() - date1.getTime()) / 1000);
			}
//			output an image for debugging
			FrameCount++;
			System.out.println("Framecount:" + FrameCount);
			System.out.println((date2.getTime() - date1.getTime()) / 1000);
			if ((int)((date2.getTime() - date1.getTime()) / 1000) % 5 == 0)
			{
				Imgcodecs.imwrite("output" + id + ".png", matOriginal);
				Imgcodecs.imwrite("output" + id + "a" + ".png", matThresh);
				Imgcodecs.imwrite("output" + id + "b" + ".png", matHeirarchy);
				System.out.println("Image");
			}
		}
		videoCapture.release();
		System.exit(0);
		} catch(Exception e){e.printStackTrace();;}
	}
	
	public static double normalize360(double angle){
		while(angle >= 180.0)
        {
            angle -= 360.0;
        }
        while(angle < -180.0)
        {
            angle += 360.0;
        }
        return angle;
	}
}
	

