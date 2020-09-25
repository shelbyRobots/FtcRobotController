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
public class StoneDetector extends Detector {

    private String bName = "GTO1";

    public enum Position
    {
        CENTER, RIGHT, LEFT, NONE
    }

    private static StoneDetector.Position foundPosition = StoneDetector.Position.NONE;

    private HalDashboard dashboard;
    private static final String TAG = "SJH_SCD";

    @SuppressWarnings("FieldCanBeLocal")
    private boolean saveSizedImage  = false;
    @SuppressWarnings("FieldCanBeLocal")
    private boolean saveMaskedImage = false;
    @SuppressWarnings("FieldCanBeLocal")
    private boolean saveThreshImage = true;

    @SuppressWarnings("WeakerAccess")
    public StoneDetector()
    {
        name = "StoneDetector";
        dashboard = CommonUtil.getInstance().getDashboard();
    }

    public StoneDetector(String name)
    {
        this();
        this.bName = name;
    }

    public void logTelemetry()
    {
        dashboard.displayPrintf(4,"Stone Location Detected: %s", foundPosition);
    }

    public void logDebug()
    {
        RobotLog.ii(TAG, "Stone Location Detected %s", foundPosition);
    }

    public void setImage( Mat img )
    {
        super.setImage(img);
        extract();
    }

    public StoneDetector.Position getStonePos()
    {
        return foundPosition;
    }

    private void extract()
    {
        RobotLog.dd(TAG, "StoneDetector.extract()");
        StonePipeline gpl = new StonePipeline();
        gpl.setName(bName);

        if(showImg == null)
        {
            RobotLog.ee(TAG, "StoneDetector.extract() null image");
        }

        RobotLog.dd(TAG, "StoneDetector.extract imgSize %dx%d",
                showImg.cols(), showImg.rows());

        gpl.sizeSource(showImg);
        Mat sizedImage = gpl.resizeImageOutput();
        RobotLog.dd(TAG, "Saving resize image");
        if (saveSizedImage) saveImage(sizedImage, "sized");

        ArrayList<MatOfPoint> goldcntrs;

        RobotLog.dd(TAG, "Saving masked image");
        if (saveMaskedImage) saveImage(sizedImage, "maskedSrc");

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

        int midX = sizedImage.width()/2;
        int oneThirdX = sizedImage.width()/3;
        int twoThirdX = 2*sizedImage.width()/3;
        int numLtOneThird = 0;
        int numLtTwoThird = 0;
        int numGtOneThird = 0;
        int numGtTwoThird = 0;

        if (goldcntrs.size() == 0)
        {
            RobotLog.dd(TAG, "No Countours found.");
        }
        else if (goldcntrs.size() == 1)
        {
            RobotLog.ee(TAG, " found 1 contour",
                    goldcntrs.size());
            for (MatOfPoint pts : goldcntrs)
            {
                MatOfPoint contour = each.next();
                bounded_box = Imgproc.boundingRect(contour);
                found_x_ptr = (bounded_box.width/2 + bounded_box.x);
            }

            if (found_x_ptr < midX)
            {
                foundPosition = StoneDetector.Position.RIGHT;
            }
            else
            {
                foundPosition = StoneDetector.Position.LEFT;
            }
        }
        else if (goldcntrs.size() == 2)
        {
            RobotLog.ee(TAG, " found 2 contours",
                    goldcntrs.size());
            for (MatOfPoint pts : goldcntrs)
            {
                MatOfPoint contour = each.next();
                bounded_box = Imgproc.boundingRect(contour);
                found_x_ptr = (bounded_box.width/2 + bounded_box.x);
                if (found_x_ptr < twoThirdX)
                {
                    numLtTwoThird++;
                }
                else
                {
                    numGtTwoThird++;
                }
                if (found_x_ptr < oneThirdX)
                {
                    numLtOneThird++;
                }
                else
                {
                    numGtOneThird++;
                }
            }

            if (numLtTwoThird == 0)
            {
                foundPosition = StoneDetector.Position.RIGHT;
            }
            else if (numGtOneThird == 0)
            {
                foundPosition = StoneDetector.Position.LEFT;
            }
            else
            {
                foundPosition = StoneDetector.Position.CENTER;
            }

        }
    }
}