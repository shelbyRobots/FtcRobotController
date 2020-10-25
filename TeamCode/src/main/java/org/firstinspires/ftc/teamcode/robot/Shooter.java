package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

public class Shooter {

    public Shooter(HardwareMap map){
        super();
        this.hwMap = map;
    }

    public boolean init()
    {
        boolean success = false;
        try
        {
            shooter = hwMap.get(DcMotorEx.class, "shoot");
            success = true;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initShooter\n" + e.toString());
        }

        if(shooter != null)
        {
            shooter.setDirection(DcMotor.Direction.FORWARD);
            shooter.setPower(0);
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(RUN_USING_ENCODER);
        }

        return success;
    }

    protected HardwareMap hwMap;
    public DcMotorEx shooter;
    private static final String TAG = "SJH_SHT";
}