package org.firstinspires.ftc.teamcode.image;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Iterator;

import hallib.HalDashboard;

/**
 * Created by crazy on 9/20/2018.
 */

@SuppressWarnings("unused")
public class MineralDetector extends Detector {
//        private static final double THRESHOLD = 0.4;
//        private static final int BINS = 8;
//        private static final float MIN_VALUE = 0.0f;
//        private static final float MAX_VALUE = 255.0f;

    private String bName = "GTO1";

    private static MineralDetector.Position foundPosition = MineralDetector.Position.NONE;

    private HalDashboard dashboard;
    private static final String TAG = "SJH_MCD";

    public enum Position
    {
        CENTER, RIGHT, LEFT, NONE, GOLDAHEAD, SILVERAHEAD
    }

    public MineralDetector()
    {
        name = "MineralDetector";
        dashboard = CommonUtil.getInstance().getDashboard();
    }

    public MineralDetector(String name)
    {
        this();
        this.bName = name;
    }

    public void logTelemetry()
    {
        dashboard.displayPrintf(4,"Mineral Location Detected: %s", foundPosition);
    }

    public void logDebug()
    {
        RobotLog.ii(TAG, "Mineral Location Detected %s", foundPosition);
    }

    public void setImage( Mat img )
    {
        super.setImage(img);
        extract();
    }

    public MineralDetector.Position getMineralPos()
    {
        return foundPosition;
    }

    private boolean ctrScan = false;
    public  void setCenterScan(boolean ctrScan) { this.ctrScan = ctrScan; }
    private boolean pitScan = false;
    public  void setPitScan(boolean pitScan) { this.pitScan = pitScan; }


    private void extract()
    {
        RobotLog.dd(TAG, "MineralDetector.extract()");
        //GripPipeline gpl = new GripPipeline();
        GripPipelineLonger gpl = new GripPipelineLonger();
        gpl.setName(bName);

//        RobotLog.dd(TAG, "Saving source image");
//        saveImage(showImg, "source");

        gpl.sizeSource(showImg);
        Mat sizedImage = gpl.resizeImageOutput();
        RobotLog.dd(TAG, "Saving resize image");
        saveImage(sizedImage, "sized");

        ArrayList<MatOfPoint> pitcntrs;
        ArrayList<MatOfPoint> goldcntrs;
        ArrayList<MatOfPoint> silvercntrs;

        if (pitScan)
        {
            gpl.processPit(sizedImage);

//            RobotLog.dd(TAG, "Saving pitBlur image");
//            saveImage(gpl.blurOutput(), "pitBlur");
//            RobotLog.dd(TAG, "Saving pitThres image");
//            saveImage(gpl.hsvThresholdOutput(), "pitThresh");
//
//            pitcntrs = gpl.filterContoursOutput();
//
//            RobotLog.dd(TAG, "Finding pit mask");
//            Rect pitMask = gpl.findMask(pitcntrs);
            Rect ctrMask = gpl.centerMask();
            //Imgproc.rectangle(sizedImage, pitMask.tl(), pitMask.br(), new Scalar(0, 0, 0), -1);
            Imgproc.rectangle(sizedImage, ctrMask.tl(), ctrMask.br(), new Scalar(0, 0, 0), -1);
        }

        if(ctrScan)
        {
            RobotLog.dd(TAG, "Finding ctr masks");
            Rect lMask = gpl.leftMask();
            Rect rMask = gpl.rightMask();
            Imgproc.rectangle(sizedImage, lMask.tl(), lMask.br(), new Scalar(0,0,0), -1);
            Imgproc.rectangle(sizedImage, rMask.tl(), rMask.br(), new Scalar(0,0,0), -1);
        }

        RobotLog.dd(TAG, "Saving masked image");
        saveImage(sizedImage, "maskedSrc");

        RobotLog.dd(TAG, "Digging for gold");
        gpl.processGold(sizedImage);

//        RobotLog.dd(TAG, "Saving minBlur image");
//        saveImage(gpl.blurOutput(), "minBlur");
        RobotLog.dd(TAG, "Saving minThresh image");
        saveImage(gpl.hsvThresholdOutput(), "minThresh");

        goldcntrs = gpl.filterContoursOutput();

        Iterator<MatOfPoint> each = goldcntrs.iterator();

        Rect bounded_box;
        int found_x_ptr = 0;
        foundPosition = Position.LEFT;

        RobotLog.dd(TAG, "Processing %d gold contours", goldcntrs.size());
        if(ctrScan)
        {
            if(goldcntrs.size() == 0)
            {
                RobotLog.dd(TAG, "Digging for silver");
                gpl.processSilver(sizedImage);
                silvercntrs = gpl.filterContoursOutput();
                RobotLog.dd(TAG, "Processing %d silver contours", silvercntrs.size());

                if(silvercntrs.size() == 0)
                {
                    foundPosition = Position.NONE;
                    RobotLog.dd(TAG, "No Silver Countours in center scan.");
                }
                else
                {
                    foundPosition = Position.SILVERAHEAD;
                    RobotLog.dd(TAG, "Found silver");
                }

            }
            else
            {
                foundPosition = Position.GOLDAHEAD;
                RobotLog.dd(TAG, "In ctrScan mode and %d countours found", goldcntrs.size());
            }
        }
        else // pitScan
        {
            if (goldcntrs.size() == 0)
            {
                foundPosition = Position.LEFT;
                RobotLog.dd(TAG, "No Countours.  Assuming left is gold");
            }
            else if (goldcntrs.size() > 1)
            {
                RobotLog.ee(TAG, "Too Many Countours=%d.  I don't know which is gold",
                        goldcntrs.size());
                foundPosition = Position.CENTER;
            }
            else //goldcntrs.size() == 1
            {
                for (MatOfPoint pts : goldcntrs)
                {
                    MatOfPoint contour = each.next();
                    bounded_box = Imgproc.boundingRect(contour);
                    found_x_ptr = (bounded_box.width/2 + bounded_box.x);
                }

                RobotLog.dd(TAG, "Center of Gold = " + found_x_ptr);

                int midX = gpl.cvDilateOutput().width()/2;

                if (found_x_ptr < midX)
                {
                    foundPosition = MineralDetector.Position.CENTER;
                    //Yay it in deh center!!!!!!!!!!
                }
                else
                {
                    foundPosition = MineralDetector.Position.RIGHT;
                    //Yay it in deh right!!!!!!!!!!!
                }
            }
        }
    }
}
