package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.ImuRunner;

import java.util.Locale;

class ShelbyImuBot extends ShelbyBot
{
    private Orientation angles;
    //private Acceleration gravity;

    private ImuRunner imuRunner;

    private ElapsedTime imuTimer = new ElapsedTime();

    private boolean useImuThread = false;

    private static final String TAG = "SJH_Imu";

    @Override
    protected void initSensors(boolean initDirSensor)
    {
        RobotLog.dd(TAG, "In TilerunnerGtoBot.initSensors");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMU_IMUCalibration.json";
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        try
        {
            imu = (BNO055IMU) com.getHardwareMap().get("imu");
            if(initDirSensor) imu.initialize(parameters);

            if(useImuThread)
            {
                imuRunner = ImuRunner.getInstance(imu);
                imuRunner.run();
            }

            colorSensor = hwMap.get(ColorSensor.class, "color1");

            capMap.put("sensor", true);
        }
        catch(Exception e)
        {
            RobotLog.ee(TAG, "ERROR get imu\n" + e.toString());
        }
    }

    @Override
    public boolean calibrateGyro()
    {
        //Callibration performed internally and assisted by loading file from offline calib.
        RobotLog.dd(TAG, "CalibrateGyro isAccelCal %s isGyroCal %s calStatus %s sysStatus %s",
                imu.isAccelerometerCalibrated(),
                imu.isGyroCalibrated(),
                imu.getCalibrationStatus(),
                imu.getSystemStatus());

        RobotLog.ii(TAG, "Calibration drive dir = %s", calibrationDriveDir);
        gyroReady = true;
        return true;
    }

    public void resetGyro()
    {
    }

    @Override
    public double getGyroHdg()
    {
        double startTime = imuTimer.milliseconds();
        getGyroAngles();
        double yaw = angles.firstAngle;
        double endTime = imuTimer.milliseconds();
        double imuTime = endTime - startTime;
        RobotLog.dd("IMU", String.format(Locale.US,
                "%.2f,%.4f", imuTime, yaw));

        return yaw;
    }

    private void getGyroAngles()
    {
        if(useImuThread) angles = imuRunner.getOrientation();
        else             angles = imu.getAngularOrientation();
    }
}
