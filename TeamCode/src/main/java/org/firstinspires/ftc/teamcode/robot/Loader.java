package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

import androidx.annotation.NonNull;
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

        try
        {
            ringGate = hwMap.get(Servo.class, "ringGate");
            setGatePos(gatePos.CLOSE);

        } catch (Exception e){
            success = false;
            RobotLog.ee(TAG, "ERROR in loader init\n" + e.toString());
        }

        return success;
    }

    public void setGatePos(gatePos pos){
        curGatePos = pos;
        if(ringGate != null) ringGate.setPosition(curGatePos.srvPos);
    }

    public enum gatePos{
        OPEN(0.0),
        CLOSE(0.3);

        public final double srvPos;

        gatePos (double srvPos){
            this.srvPos = srvPos;
        }
    }

    public void update()
    {
        if(loadMotor != null)
        {
            encPos = loadMotor.getCurrentPosition();
            curSpd = loadMotor.getVelocity();
        }
    }

    public void pass(){
        if (curGatePos != gatePos.OPEN) setGatePos(gatePos.OPEN);
        //TODO: add sleep to close ringGate
    }

    @NonNull
    public String toString(){
        return String.format(Locale.US, "shoot %5d %4.2f",
                encPos, curSpd);
    }

    public void load (double pwr)
    {
        if(loadMotor != null) loadMotor.setPower(pwr);
    }

    private gatePos curGatePos;
    private DcMotorEx loadMotor;
    protected HardwareMap hwMap;
    public Servo ringGate;
    private static final String TAG = "SJH_LDR";
    private int encPos;
    private double curSpd;
}
