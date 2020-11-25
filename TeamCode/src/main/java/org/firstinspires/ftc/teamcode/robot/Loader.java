package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

import androidx.annotation.NonNull;

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

    public void update(){
        encPos = loadMotor.getCurrentPosition();
        curSpd = loadMotor.getVelocity();
    }

    @NonNull
    public String toString(){
        return String.format(Locale.US, "shoot %5d %4.2f",
                encPos, curSpd);
    }

    public void load (double pwr)
    {
        loadMotor.setPower(pwr);
    }

    private DcMotorEx loadMotor;
    protected HardwareMap hwMap;
    private static final String TAG = "SJH_LDR";
    private int encPos;
    private double curSpd;
}
