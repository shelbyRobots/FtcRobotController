package org.firstinspires.ftc.teamcode.image;

import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("unused")
public class GripPipeline {

	private static final String TAG = "SJH_GPL";
	//Outputs
	private Mat resizeImageOutput;
	private Mat roiMat;
	private Mat blurOutput;
	private Mat hsvThresholdOutput = new Mat();
	private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
	private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<>();
	private ArrayList<MatOfPoint> convexHullsOutput = new ArrayList<>();

//	static {
//		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
//	}

	public void process(Mat source0)
    {
    	RobotLog.dd(TAG, "Processing image WXH= %dx%d", source0.cols(), source0.rows());
        roiMat = new Mat(source0, new Rect(0, source0.height()/3,
                                           source0.width(), source0.height()/3));
		RobotLog.dd(TAG, " roiMat image WXH= %dx%d", roiMat.cols(), roiMat.rows());

        int resizeImageWidth = 512;
        int resizeImageHeight = (int)(resizeImageWidth*((double)roiMat.height()/roiMat.width()));

        if(resizeImageOutput == null)
            resizeImageOutput = new Mat(resizeImageHeight, resizeImageWidth, source0.type());

		int resizeImageInterpolation = Imgproc.INTER_LINEAR;

		resizeImage(roiMat, resizeImageWidth, resizeImageHeight,
                    resizeImageInterpolation, resizeImageOutput);

		// Step Blur0:
		Mat blurInput = resizeImageOutput;
		BlurType blurType = BlurType.get("Gaussian Blur");
		double blurRadius = 4.0;
		if(blurOutput == null)
		    blurOutput = new Mat(blurInput.rows(), blurInput.cols(), blurInput.type());
		blur(blurInput, blurType, blurRadius, blurOutput);

		// Step HSV_Threshold0:
		Mat hsvThresholdInput = blurOutput;
		double[] hsvThresholdHue =        {  0.0,  44.0};
		double[] hsvThresholdSaturation = {165.0, 255.0};
		double[] hsvThresholdValue =      { 24.0, 255.0};
//		if(hsvThresholdOutput == null)
//		    hsvThresholdOutput = new Mat(hsvThresholdInput.rows(),
//                                         hsvThresholdInput.cols(), CV_8UC1);
		hsvThreshold(hsvThresholdInput,
                     hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue,
                     hsvThresholdOutput);

		// Step Find_Contours0:
		Mat findContoursInput = hsvThresholdOutput;
		boolean findContoursExternalOnly = false;
		//noinspection ConstantConditions
		findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

		// Step Filter_Contours0:
		ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
		double filterContoursMinArea = 1500.0;
		double filterContoursMinPerimeter = 0;
		double filterContoursMinWidth = 0;
		double filterContoursMaxWidth = 1000;
		double filterContoursMinHeight = 0.0;
		double filterContoursMaxHeight = 150.0;
		double[] filterContoursSolidity = {0, 100};
		double filterContoursMaxVertices = 1000000;
		double filterContoursMinVertices = 0;
		double filterContoursMinRatio = 0.5;
		double filterContoursMaxRatio = 1.6;
		filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);

		// Step Convex_Hulls0:
		ArrayList<MatOfPoint> convexHullsContours = filterContoursOutput;
		convexHulls(convexHullsContours, convexHullsOutput);
	}

	public Mat roiMat()                                 { return roiMat; }
 	public Mat resizeImageOutput()                      { return resizeImageOutput; }
	public Mat blurOutput()                             { return blurOutput; }
	public Mat hsvThresholdOutput()                     { return hsvThresholdOutput; }
	public ArrayList<MatOfPoint> findContoursOutput()   { return findContoursOutput; }
	public ArrayList<MatOfPoint> filterContoursOutput() {
		return filterContoursOutput;
	}
	public ArrayList<MatOfPoint> convexHullsOutput()    { return convexHullsOutput; }

	private void resizeImage(Mat input, double width, double height,
		int interpolation, Mat output)
	{
		RobotLog.dd(TAG, " resizeImage image inWXH= %dx%d outWXH= %dx%d ", input.cols(),
				input.rows(), output.cols(), output.rows());
		Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
	}

	enum BlurType{
		BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
			BILATERAL("Bilateral Filter");

		private final String label;

		BlurType(String label) {
			this.label = label;
		}

		public static BlurType get(String type) {
			if (BILATERAL.label.equals(type)) {
				return BILATERAL;
			}
			else if (GAUSSIAN.label.equals(type)) {
			return GAUSSIAN;
			}
			else if (MEDIAN.label.equals(type)) {
				return MEDIAN;
			}
			else {
				return BOX;
			}
		}

		@Override
		public String toString() {
			return this.label;
		}
	}

	private void blur(Mat input, BlurType type, double doubleRadius,
		Mat output) {
		int radius = (int)(doubleRadius + 0.5);
		int kernelSize;

		RobotLog.dd(TAG, " blur image inWXH= %dx%d outWXH= %dx%d", input.cols(),
				input.rows(), output.cols(), output.rows());

		switch(type){
			case BOX:
				kernelSize = 2 * radius + 1;
				Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
				break;
			case GAUSSIAN:
				kernelSize = 6 * radius + 1;
				Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
				break;
			case MEDIAN:
				kernelSize = 2 * radius + 1;
				Imgproc.medianBlur(input, output, kernelSize);
				break;
			case BILATERAL:
				Imgproc.bilateralFilter(input, output, -1, radius, radius);
				break;
		}
	}

	private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
	    Mat out) {
		RobotLog.dd(TAG, " blur image inWXH = %dx%d", input.cols(),
				input.rows());
		Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV);
		Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
			new Scalar(hue[1], sat[1], val[1]), out);
	}

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

	private void filterContours(List<MatOfPoint> inputContours, double minArea,
		double minPerimeter, double minWidth, double maxWidth, double minHeight, double
		maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
		minRatio, double maxRatio, List<MatOfPoint> output) {
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
	}

	/**
	 * Compute the convex hulls of contours.
	 * @param inputContours The contours on which to perform the operation.
	 * @param outputContours The contours where the output will be stored.
	 */
	private void convexHulls(List<MatOfPoint> inputContours,
		ArrayList<MatOfPoint> outputContours) {
		final MatOfInt hull = new MatOfInt();
		outputContours.clear();
		for (int i = 0; i < inputContours.size(); i++) {
			final MatOfPoint contour = inputContours.get(i);
			final MatOfPoint mopHull = new MatOfPoint();
			Imgproc.convexHull(contour, hull);
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++) {
				int index = (int) hull.get(j, 0)[0];
				double[] point = new double[] {contour.get(index, 0)[0], contour.get(index, 0)[1]};
				mopHull.put(j, 0, point);
			}
			outputContours.add(mopHull);
		}
	}
}

