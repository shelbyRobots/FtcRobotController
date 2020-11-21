package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

public class Intake {

    public Intake(HardwareMap map){
        super();
        this.hwMap = map;
    }

    public boolean init()
    {
        boolean success = false;
            try
            {
                intaker = hwMap.get(DcMotorEx.class, "intake");
                intaker.setDirection(DcMotor.Direction.REVERSE);
                intaker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intaker.setPower(0);
                success = true;
            }
            catch (Exception e)
            {
              RobotLog.ee(TAG, "ERROR get hardware map initIntake\n" + e.toString());
            }

        return success;

    }

    public String toString(){
        return String.format(Locale.US,
                "intake %5d %4.2f",
                encPos, curSpd);
    }
    public void update(){
        encPos = intaker.getCurrentPosition();
        curSpd = intaker.getVelocity();
    }

    public void stop(){
        intaker.setPower(0);
    }

    public void suck(double pwr){

        intaker.setPower(pwr);
    }
    private DcMotorEx intaker;
    protected HardwareMap hwMap;
    private static final String TAG = "SJH_INT";
    private int encPos;
    private double curSpd;
}
