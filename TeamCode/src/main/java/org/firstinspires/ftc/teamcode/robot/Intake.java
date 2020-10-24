package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

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
                intakeLeft = hwMap.get(DcMotorEx.class, "inLeft");
                intakeRight = hwMap.get(DcMotorEx.class, "inRight");
                success = true;
            }
            catch (Exception e)
            {
              RobotLog.ee(TAG, "ERROR get hardware map initIntake\n" + e.toString());
            }

            if(intakeLeft != null && intakeRight != null){

                intakeLeft.setDirection(DcMotor.Direction.REVERSE);
                intakeLeft.setPower(0);

                intakeRight.setDirection(DcMotor.Direction.FORWARD);
                intakeRight.setPower(0);
            }
        return success;

    }


    public void suck(double pwr){

        intakeLeft.setPower(pwr);
        intakeRight.setPower(pwr);


    }
    private DcMotorEx intakeLeft;
    private DcMotorEx intakeRight;
    protected HardwareMap hwMap;
    private static final String TAG = "SJH_LFT";
}
