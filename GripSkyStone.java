package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.Blob;
import org.opencv.core.*;
import org.opencv.core.Point;
import org.opencv.imgproc.*;

/**
* GripSkyStone class.
*
* <p>An OpenCV pipeline generated by GRIP.
*
* @author GRIP
*/
@Disabled
public class GripSkyStone {


	//Outputs
	private Mat resizeImageOutput = new Mat();
	private Mat hsvThresholdOutput = new Mat();
	private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();
	private boolean filterContoursOutputIsReady = false;
	private ArrayList<Point> findBlobsOutput = new ArrayList<>();
	ArrayList<Blob> blobs  = new ArrayList<>();
	public int croper = 0;
	private Mat croppedMat = new Mat();


	/**
	 * This is the primary method that runs the entire pipeline and updates the outputs.
	 */
	public void process(Mat source0) {
		crop(source0);
		Mat hsvThresholdInput = croppedMat;
//		double[] hsvThresholdHue = {0.0, 180.0};
//		double[] hsvThresholdSaturation = {216.10169491525423, 255.0};
//		double[] hsvThresholdValue = {103.24858757062147, 255.0};
		double[] hsvThresholdHue = {0.0, 180.0};
		double[] hsvThresholdSaturation = {0.0, 200.68124134169247};
		double[] hsvThresholdValue = {0.0, 40.250072669137424};
		hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

		// Step Find_Contours0:
		Mat findContoursInput = hsvThresholdOutput;
		boolean findContoursExternalOnly = true;
		findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

		// Step Filter_Contours0:
		ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
		double filterContoursMinArea = 20;
		double filterContoursMinPerimeter = 0.0;
		double filterContoursMinWidth = 0.0;
		double filterContoursMaxWidth = 1000.0;
		double filterContoursMinHeight = 0.0;
		double filterContoursMaxHeight = 1000.0;
		double[] filterContoursSolidity = {0, 100};
		double filterContoursMaxVertices = 1000000.0;
		double filterContoursMinVertices = 0.0;
		double filterContoursMinRatio = 0.0;
		double filterContoursMaxRatio = 1000.0;
		filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
		findBlobs();
//		ClearBlobs(30);
	}

	/**
	 * This method is a generated getter for the output of a Resize_Image.
	 * @return Mat output from Resize_Image.
	 */
	public Mat resizeImageOutput() {
		return resizeImageOutput;
	}

	/**
	 * This method is a generated getter for the output of a HSV_Threshold.
	 * @return Mat output from HSV_Threshold.
	 */
	public Mat hsvThresholdOutput() {
		return hsvThresholdOutput;
	}

	/**
	 * This method is a generated getter for the output of a Find_Contours.
	 * @return ArrayList<MatOfPoint> output from Find_Contours.
	 */
	public ArrayList<MatOfPoint> findContoursOutput() {
		return findContoursOutput;
	}

	private ArrayList<Point> ContoursToArray(){
		ArrayList<Point> r = new ArrayList<>();
		for (MatOfPoint p : filterContoursOutput){
			r.addAll(p.toList());
		}
		return r;
	}
	public ArrayList<Point> findContoursArray(){
		ArrayList<Point> countours = ContoursToArray();
		ArrayList<Point> newCountours = new ArrayList<>();
		Log.d("size", String.valueOf(countours.size()));
		for(int i = 0;i<countours.size();i++){
			if(countours.get(i).y > 300 && countours.get(i).x > 160 ){
				Log.d("y", String.valueOf(countours.get(i).y));
				newCountours.add(countours.get(i));
			}else{
				Log.d("by", String.valueOf(countours.get(i).y));

			}
		}
			return newCountours;

	}
	/**
	 * This method is a generated getter for the output of a Filter_Contours.
	 * @return ArrayList<MatOfPoint> output from Filter_Contours.
	 */
	public synchronized void filterContoursOutput(List<MatOfPoint> newContours) {
		if (true == filterContoursOutputIsReady) {
			if (!(findContoursOutput.isEmpty())) {
				for (int i = 0 ; i < findContoursOutput.size() ; i++) {
					newContours.add(findContoursOutput.get(i)) ;
				}
			}
			//return contoursGold;

		}
	}


	/**
	 * Scales and image to an exact size.
	 * @param input The image on which to perform the Resize.
	 * @param width The width of the output in pixels.
	 * @param height The height of the output in pixels.
	 * @param interpolation The type of interpolation.
	 * @param output The image in which to store the output.
	 */
	private void resizeImage(Mat input, double width, double height,
							 int interpolation, Mat output) {
		Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
	}

	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue
	 * @param sat The min and max saturation
	 * @param val The min and max value
	 * //@param output The image in which to store the output.
	 */
	private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
							  Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV);
		Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
				new Scalar(hue[1], sat[1], val[1]), out);
	}

	/**
	 * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
	 * @param input The image on which to perform the Distance Transform.
	 * //@param type The Transform.
	 * //@param maskSize the size of the mask.
	 * //@param output The image in which to store the output.
	 */
	private void findContours(Mat input, boolean externalOnly,
							  List<MatOfPoint> contours) {
		Mat hierarchy = new Mat();
		contours.clear();
		int mode;
		if (externalOnly) {
			mode = Imgproc.RETR_EXTERNAL;
		}
		else {
			mode = Imgproc.RETR_LIST;
		}
		int method = Imgproc.CHAIN_APPROX_SIMPLE;
		Imgproc.findContours(input, contours, hierarchy, mode, method);
	}


	/**
	 * Filters out contours that do not meet certain criteria.
	 * @param inputContours is the input list of contours
	 * @param output is the the output list of contours
	 * @param minArea is the minimum area of a contour that will be kept
	 * @param minPerimeter is the minimum perimeter of a contour that will be kept
	 * @param minWidth minimum width of a contour
	 * @param maxWidth maximum width
	 * @param minHeight minimum height
	 * @param maxHeight maximimum height
	 * //@param Solidity the minimum and maximum solidity of a contour
	 * @param minVertexCount minimum vertex Count of the contours
	 * @param maxVertexCount maximum vertex Count
	 * @param minRatio minimum ratio of width to height
	 * @param maxRatio maximum ratio of width to height
	 */
	private void filterContours(List<MatOfPoint> inputContours, double minArea,
								double minPerimeter, double minWidth, double maxWidth, double minHeight, double
										maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
										minRatio, double maxRatio, List<MatOfPoint> output) {
		filterContoursOutputIsReady = false;
		final MatOfInt hull = new MatOfInt();
		output.clear();
		//operation
		for (int i = 0; i < inputContours.size(); i++) {
			final MatOfPoint contour = inputContours.get(i);
			final Rect bb = Imgproc.boundingRect(contour);
			if (bb.width < minWidth || bb.width > maxWidth) continue;
			if (bb.height < minHeight || bb.height > maxHeight) continue;
			final double area = Imgproc.contourArea(contour);
			if (area < minArea) continue;
			if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
			Imgproc.convexHull(contour, hull);
			MatOfPoint mopHull = new MatOfPoint();
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++) {
				int index = (int)hull.get(j, 0)[0];
				double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
				mopHull.put(j, 0, point);
			}
			final double solid = 100 * area / Imgproc.contourArea(mopHull);
			if (solid < solidity[0] || solid > solidity[1]) continue;
			if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
			final double ratio = bb.width / (double)bb.height;
			if (ratio < minRatio || ratio > maxRatio) continue;
			output.add(contour);
		}
		filterContoursOutputIsReady = true;
	}
	public ArrayList<Point> findBlobsOutput() {
		return findBlobsOutput;
	}
	private void findBlobs() {
		/*FeatureDetector blobDet = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
		try {
			File tempFile = File.createTempFile("config", ".xml");

			StringBuilder config = new StringBuilder();

			config.append("<?xml version=\"1.0\"?>\n");
			config.append("<opencv_storage>\n");
			config.append("<thresholdStep>10.</thresholdStep>\n");
			config.append("<minThreshold>50.</minThreshold>\n");
			config.append("<maxThreshold>220.</maxThreshold>\n");
			config.append("<minRepeatability>2</minRepeatability>\n");
			config.append("<minDistBetweenBlobs>10.</minDistBetweenBlobs>\n");
			config.append("<filterByColor>1</filterByColor>\n");
			config.append("<blobColor>");
			config.append((darkBlobs ? 0 : 255));
			config.append("</blobColor>\n");
			config.append("<filterByArea>1</filterByArea>\n");
			config.append("<minArea>");
			config.append(minArea);
			config.append("</minArea>\n");
			config.append("<maxArea>");
			config.append(Integer.MAX_VALUE);
			config.append("</maxArea>\n");
			config.append("<filterByCircularity>1</filterByCircularity>\n");
			config.append("<minCircularity>");
			config.append(circularity[0]);
			config.append("</minCircularity>\n");
			config.append("<maxCircularity>");
			config.append(circularity[1]);
			config.append("</maxCircularity>\n");
			config.append("<filterByInertia>1</filterByInertia>\n");
			config.append("<minInertiaRatio>0.1</minInertiaRatio>\n");
			config.append("<maxInertiaRatio>" + Integer.MAX_VALUE + "</maxInertiaRatio>\n");
			config.append("<filterByConvexity>1</filterByConvexity>\n");
			config.append("<minConvexity>0.95</minConvexity>\n");
			config.append("<maxConvexity>" + Integer.MAX_VALUE + "</maxConvexity>\n");
			config.append("</opencv_storage>\n");
			FileWriter writer;
			writer = new FileWriter(tempFile, false);
			writer.write(config.toString());
			writer.close();
			//blobDet.read(tempFile.getPath());
		} catch (IOException e) {
			e.printStackTrace();
		}

		blobDet.detect(input, blobList);*/
		blobs.clear();
		ArrayList<Point> points = findContoursArray();
		boolean found = false;
		Log.d("x", "reset");
		for(Point p : points){
			for(Blob blob : blobs){
				if(blob.isNear(p.x,p.y) && p.x > 60 + croper){
					blob.add(p.x,p.y);
					found = true;
				}
			}
			if(!found && p.x > 60 + croper){
				blobs.add(new Blob(p.x,p.y));
			}
		}
		for(int i = 0; i < blobs.size();i++){
			for(int j = 0; j < blobs.size();j++) {
				if (blobs.get(i).verPath.size() > blobs.get(j).verPath.size() && i > j){
					Blob b = blobs.get(j);
					blobs.set(j,blobs.get(i));
					blobs.set(i,b);
				}
			}
		}
		ArrayList<Point> centers = new ArrayList<>();
		for(Blob b : blobs){
			centers.add(b.getCenter());
		}
		findBlobsOutput = centers;

	}
	public void ClearBlobs(int minArea){
		for(int i = 0; i < blobs.size();i++){
			if(blobs.get(i).verPath.size() < minArea){
				blobs.remove(i);
			}
		}
	}

	private void crop(Mat input){
		int deadArea = (int)(input.height()/2) - 25;
		Rect roi = new Rect(0, 0, (int)((input.width()/3)*2.5) + 15,input.height() - deadArea);
//		Rect roi = new Rect(0, 0, (int)((input.width())),input.height() - deadArea);
		croppedMat = new Mat(input, roi);
	}
}
