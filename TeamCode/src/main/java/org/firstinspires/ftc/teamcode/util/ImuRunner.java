package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ImuRunner extends Thread
{
    private static final String TAG = "SJH_IRN";
    private static ImuRunner INSTANCE = null;
    private static boolean isRunning = false;
    private CommonUtil com;
    private BNO055IMU imu;
    private Orientation angles;

    private ImuRunner(BNO055IMU imu)
    {
        com = CommonUtil.getInstance();
        this.imu = imu;
    }

    public static ImuRunner getInstance(BNO055IMU imu)
    {
        if(INSTANCE == null)
        {
            INSTANCE = new ImuRunner(imu);
        }
        return INSTANCE;
    }

    private void getAngles()
    {
        Orientation tmpAngles = imu.getAngularOrientation();
        setOrientation(tmpAngles);
    }

    private synchronized void setOrientation(Orientation angles)
    {
        this.angles = angles;
    }

    public synchronized Orientation getOrientation()
    {
        return angles;
    }

    @Override
    public void run()
    {
        if(imu == null)
        {
            RobotLog.ee(TAG, "IMU THREAD ALREADY RUNNING");
            return;
        }

        if(isRunning)
        {
            RobotLog.dd(TAG, "IMU THREAD ALREADY RUNNING");
            return;
        }

        LinearOpMode lop = com.getLinearOpMode();

        isRunning = true;
        while(!lop.isStopRequested())
        {
            getAngles();
        }
        isRunning = false;
    }
}
