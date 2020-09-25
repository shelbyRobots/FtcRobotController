package org.firstinspires.ftc.teamcode.image;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.opencv.core.Mat;

import java.util.Random;

import hallib.HalDashboard;

/**
 * Created by crazy on 9/27/2017.
 */

public class MineralLocationDetector extends Detector {
    private static final double THRESHOLD = 0.4;
    private static final int BINS = 8;
    private static final float MIN_VALUE = 0.0f;
    private static final float MAX_VALUE = 255.0f;
    private Position mineralPos = Position.CENTER;

    private HalDashboard dashboard;
    private static final String TAG = "SJH_MLD";

    public enum Position
    {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN
    }

    public MineralLocationDetector()
    {
        dashboard = CommonUtil.getInstance().getDashboard();
        //TODO:  Temporary - remove
        Random rand = new Random();
        int randomNum = rand.nextInt(3);
        mineralPos = Position.values()[randomNum];
        RobotLog.dd(TAG, "CTOR randomMineralPos = " + mineralPos);
    }

    static int lcnt = 0;
    public void logTelemetry()
    {
        dashboard.displayPrintf(4,"Gold Detected: %s", mineralPos);
        dashboard.displayPrintf(7,"lcnt   %d", lcnt);
    }

    static int dcnt = 0;
    public void logDebug()
    {
        RobotLog.ii(TAG, "Mineral Detected %s %d", mineralPos, dcnt++);
    }

    public void setImage( Mat img )
    {
        super.setImage(img);
        extract();
    }

    public Position getMineralPos() { return mineralPos; }

    private void extract()
    {
    }
}
