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
            success = true;
        }
        catch (Exception e)
        {
          RobotLog.ee(TAG, "ERROR get hardware map init\n" + e.toString());
        }

        if(loadMotor != null){

            loadMotor.setDirection(DcMotor.Direction.FORWARD);
            loadMotor.setPower(0);
        }
        return success;

    }


    public void load (double pwr){

        loadMotor.setPower(pwr);
    }
    private DcMotorEx loadMotor;
    protected HardwareMap hwMap;
    private static final String TAG = "SJH_LDR0";
}
