package org.firstinspires.ftc.teamcode.image;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Iterator;

import hallib.HalDashboard;

@SuppressWarnings("unused")
public class RingDetector extends Detector {

    private String bName = "GTO1";

    public enum Position
    {
        CENTER, RIGHT, LEFT, NONE
    }

    private static RingDetector.Position foundPosition = RingDetector.Position.NONE;

    private HalDashboard dashboard;
    private static final String TAG = "SJH_SCD";

    @SuppressWarnings("FieldCanBeLocal")
    private boolean saveSizedImage  = false;
    @SuppressWarnings("FieldCanBeLocal")
    private boolean saveMaskedImage = false;
    @SuppressWarnings("FieldCanBeLocal")
    private boolean saveThreshImage = true;

    @SuppressWarnings("WeakerAccess")
    public RingDetector()
    {
        name = "StoneDetector";
        dashboard = CommonUtil.getInstance().getDashboard();
    }

    public RingDetector(String name)
    {
        this();
        this.bName = name;
    }

    public void logTelemetry()
    {
        dashboard.displayPrintf(4,"Ring Location Detected: %s", foundPosition);
    }

    public void logDebug()
    {
        RobotLog.ii(TAG, "Ring Location Detected %s", foundPosition);
    }

    public void setImage( Mat img )
    {
        super.setImage(img);
        extract();
    }

    public RingDetector.Position getRingPos()
    {
        return foundPosition;
    }

    private void extract()
    {
        RobotLog.dd(TAG, "RingDetector.extract()");
        RingPipeline gpl = new RingPipeline();
        gpl.setName(bName);

        if(showImg == null)
        {
            RobotLog.ee(TAG, "RingDetector.extract() null image");
        }

        RobotLog.dd(TAG, "RingDetector.extract imgSize %dx%d",
                showImg.cols(), showImg.rows());

        gpl.sizeSource(showImg);
        Mat sizedImage = gpl.resizeImageOutput();
        RobotLog.dd(TAG, "Saving resize image");
        if (saveSizedImage) saveImage(sizedImage, "sized");

        ArrayList<MatOfPoint> goldcntrs;

//        RobotLog.dd(TAG, "Saving masked image");
//        if (saveMaskedImage) saveImage(sizedImage, "maskedSrc");

        RobotLog.dd(TAG, "Digging for gold");
        gpl.processGold(sizedImage);

        RobotLog.dd(TAG, "Saving minThresh image");
        if (saveThreshImage) saveImage(gpl.hsvThresholdOutput(), "minThresh");

        goldcntrs = gpl.filterContoursOutput();

        Iterator<MatOfPoint> each = goldcntrs.iterator();

        Rect bounded_box;
        int found_x_ptr = 0;
        foundPosition = Position.NONE;

        RobotLog.dd(TAG, "Processing %d gold contours", goldcntrs.size());

        int numGtTwoThird = 0;
        int StackHeight;
        int ThreshHeight = 30;


        if (goldcntrs.size() == 0)
        {
            RobotLog.dd(TAG, "No Countours found,");
            foundPosition = RingDetector.Position.LEFT;
        }
        else if (goldcntrs.size() == 1)
        {
            RobotLog.ee(TAG, " found 1 contour",
                    goldcntrs.size());
            for (MatOfPoint pts : goldcntrs)
            {
                MatOfPoint contour = each.next();
                bounded_box = Imgproc.boundingRect(contour);
                StackHeight = bounded_box.height;
                if(StackHeight > ThreshHeight){

                    foundPosition = Position.RIGHT;

                }else if (StackHeight < ThreshHeight){

                    foundPosition = Position.CENTER;
                }

            }

        }
        else if (goldcntrs.size() >= 2)
        {
            RobotLog.ee(TAG, " found 2 contours, defaulting to C box",
                    goldcntrs.size());
            foundPosition = RingDetector.Position.RIGHT;
        }
    }
}