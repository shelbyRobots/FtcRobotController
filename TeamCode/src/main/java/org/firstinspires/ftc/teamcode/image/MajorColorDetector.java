package org.firstinspires.ftc.teamcode.image;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import hallib.HalDashboard;

/**
 * Created by crazy on 9/27/2017.
 */

public class MajorColorDetector extends Detector {
    private static final double THRESHOLD = 0.4;
    private static final int BINS = 8;
    private static final float MIN_VALUE = 0.0f;
    private static final float MAX_VALUE = 255.0f;
    private Color foundColor = Color.NONE;

    private double redPct = 0.0;
    private double bluPct = 0.0;

    private HalDashboard dashboard;
    private static final String TAG = "SJH_MCD";

    public enum Color {
        RED, BLUE, NONE
    }

    public MajorColorDetector()
    {
        dashboard = CommonUtil.getInstance().getDashboard();
    }

    static int lcnt = 0;
    public void logTelemetry()
    {
        dashboard.displayPrintf(4,"Color Detected: %s", foundColor);
        dashboard.displayPrintf(5,"RedPct %4.2f", redPct);
        dashboard.displayPrintf(6,"BluPct %4.2f", bluPct);
        dashboard.displayPrintf(7,"lcnt   %d", lcnt);
    }

    static int dcnt = 0;
    public void logDebug()
    {
        RobotLog.ii(TAG, "Color Detected %s redPct %4.2f bluPct %4.2f %d",
                foundColor, redPct, bluPct, dcnt++);
    }

    public void setImage( Mat img )
    {
        super.setImage(img);
        extract();
    }

    public Color getMajorColor()
    {
        return foundColor;
    }

    private void extract() {
        Mat hsvImage = new Mat(showImg.width(), showImg.height(), showImg.type());
        Mat histHue = new Mat();
        Mat histSaturation = new Mat();

        Imgproc.cvtColor(showImg, hsvImage, Imgproc.COLOR_RGB2HSV);
        List<Mat> channels = new ArrayList<>();
        Core.split(hsvImage, channels);

        //Histogram for hue
        Imgproc.calcHist(Arrays.asList( new Mat[]{channels.get(0)} ), new MatOfInt(0),
                new Mat(), histHue, new MatOfInt(BINS), new MatOfFloat(MIN_VALUE, 179));

        //Histogram for saturation
        Imgproc.calcHist(Arrays.asList( new Mat[]{channels.get(1)} ), new MatOfInt(0),
                new Mat(), histSaturation, new MatOfInt(BINS), new MatOfFloat(MIN_VALUE, MAX_VALUE));


        double sum = Core.sumElems(histHue).val[0];
        double[] values = new double[histHue.height()+histSaturation.height()];

        RobotLog.dd(TAG, "SUM_Hue %f", sum);
        RobotLog.dd(TAG, "HueMat %d x %d", histHue.width(), histHue.height());
        int k = 0;
        for (int i = 0; i < histHue.height(); ++i ) {
            values[k] = histHue.get(i, 0)[0]/sum;
            RobotLog.dd(TAG, "Val_hue %d %f", k, values[k]);
            k++;
        }
        sum = Core.sumElems(histSaturation).val[0];
        //RobotLog.dd(TAG, "SUM_Sat %f", sum);
        //RobotLog.dd(TAG, "SatMat %d x %d", histSaturation.width(), histSaturation.height());
        for ( int i = 0; i < histSaturation.height(); ++i) {
            values[k] = histSaturation.get(i, 0)[0]/sum;
            //RobotLog.dd(TAG, "Val_sat %d %f", k, values[k]);
            k++;
        }
        // 0 + 7 == red
        // 4 + 5 == blue
       double total = 0.0;

        for ( int i = 0; i<BINS; i++){

            total+=values[i];
            RobotLog.dd(TAG, "Total %4.2f", total);
        }
        double red =  values[0] + values[7];
        double blue = values[4] + values[5];
        double redp = red/total;
        double bluep = blue/total;
        redPct = redp;
        bluPct = bluep;

        if(redp >= THRESHOLD && redp > bluep){
            //yayRed!!! :D
            foundColor = Color.RED;
        }
        else if(bluep >= THRESHOLD && bluep > redp)
        {
            //yayBlue!!! :D
            foundColor = Color.BLUE;
        }
        else{
            //booNone!!!:(
            foundColor = Color.NONE;
        }
    }

    Mat cvImage;
    public void hightlightCorners(Bitmap fullImag, List<Point2d> pts)
    {
        int cvt = CvType.CV_8UC1;
        int inHeight = fullImag.getHeight();
        int inWidth = fullImag.getWidth();

        if(cvImage == null) cvImage = new Mat(inHeight, inWidth, cvt);

        Utils.bitmapToMat(fullImag, cvImage);

        for(Point2d pt : pts)
        {
            Point p = new Point(pt.getX(), pt.getY());
            Imgproc.circle(cvImage, p, 5, new Scalar(1, 0, 0));
            RobotLog.dd(TAG, "Corner at %4.2f %4.2f", p.x, p.y);
        }

        saveImage(cvImage);
    }
}
