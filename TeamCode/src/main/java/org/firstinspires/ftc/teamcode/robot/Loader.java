package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

public class Loader {

    public Loader(HardwareMap map){
        super();
        this.hwMap = map;
    }

    public boolean init()
    {
        boolean success = false;
        try
        {
            loadMotor = hwMap.get(DcMotorEx.class, "loader");
            loadMotor.setDirection(DcMotor.Direction.FORWARD);
            loadMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            loadMotor.setPower(0);
            success = true;
        }
        catch (Exception e)
        {
          RobotLog.ee(TAG, "ERROR in loader init\n" + e.toString());
        }

        return success;
    }


    public void load (double pwr)
    {
        loadMotor.setPower(pwr);
    }

    private DcMotorEx loadMotor;
    protected HardwareMap hwMap;
    private static final String TAG = "SJH_LDR";
}
