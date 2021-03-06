package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

import androidx.annotation.NonNull;
public class Loader
{
    public Loader(HardwareMap map)
    {
        this.hwMap = map;
    }

    public boolean init()
    {
        boolean success = false;
        try
        {
            loadMotor = hwMap.get(DcMotorEx.class, "loader");
            loadMotor.setDirection(RobotConstants.LD_PUSH_DIR);
            loadMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            loadMotor.setPower(0.0);
            curLdrMotorPwr = 0.0;

            success = true;
        }
        catch (Exception e)
        {
          RobotLog.ee(TAG, "ERROR in loader init\n" + e.toString());
        }

        try
        {
            ringGate = hwMap.get(Servo.class, "ringGate");
            ringGate.setPosition(gatePos.CLOSE.srvPos);
            curGatePos = gatePos.CLOSE;
        }
        catch (Exception e)
        {
            success = false;
            RobotLog.ee(TAG, "ERROR in loader init gate\n" + e.toString());
        }

        try
        {
            ldrSrvo = hwMap.get(CRServo.class, "ldrSrvo");
            ldrSrvo.setDirection(DcMotorSimple.Direction.REVERSE);
            ldrSrvo.setPower(0.0);
            curLdrSrvoPwr = 0.0;
        }
        catch (Exception e)
        {
            success = false;
            RobotLog.ee(TAG, "ERROR in loader init ldrSrvo\n" + e.toString());
        }

        return success;
    }

    public void update()
    {
        if(loadMotor != null)
        {
            encPos = loadMotor.getCurrentPosition();
            curSpd = loadMotor.getVelocity();
        }
    }

    public void setGatePos(gatePos pos)
    {
        if(ringGate != null && pos != curGatePos)
        {
            ringGate.setPosition(pos.srvPos);
            curGatePos = pos;
        }
    }

    public void pass()
    {
        if (curGatePos != gatePos.OPEN) setGatePos(gatePos.OPEN);
    }

    public void load(double pwr)
    {
        if(loadMotor != null && pwr != curLdrMotorPwr)
        {
            loadMotor.setPower(pwr);
            curLdrMotorPwr = pwr;
        }
    }

    public void whlLoad(double pwr)
    {
        if(ldrSrvo != null && pwr != curLdrSrvoPwr)
        {
            ldrSrvo.setPower(pwr);
            curLdrSrvoPwr = pwr;
        }
    }

    public void whlFwd()
    {
        whlLoad(1.0);
    }

    public void whlBak()
    {
        whlLoad(-1.0);
    }

    public void whlStp()
    {
        whlLoad(0.0);
    }

    @NonNull
    public String toString(){
        return String.format(Locale.US, "shoot %5d %4.2f",
                encPos, curSpd);
    }

    public enum gatePos
    {
        OPEN(RobotConstants.LD_GATE_OPEN),
        CLOSE(RobotConstants.LD_GATE_CLOSED);

        public final double srvPos;

        gatePos (double srvPos){
            this.srvPos = srvPos;
        }
    }

    private gatePos curGatePos;
    private double curLdrMotorPwr;
    private double curLdrSrvoPwr;
    private DcMotorEx loadMotor;
    protected HardwareMap hwMap;
    public Servo ringGate;
    public CRServo ldrSrvo;
    private static final String TAG = "SJH_LDR";
    private int encPos;
    private double curSpd;
}
