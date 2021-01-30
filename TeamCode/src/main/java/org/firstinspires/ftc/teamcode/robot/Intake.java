package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

public class Intake
{
    public Intake(HardwareMap map)
    {
        this.hwMap = map;
    }

    public boolean init()
    {
        boolean success = false;
        try
        {
            intaker = hwMap.get(DcMotorEx.class, "intake");
            intaker.setDirection(RobotConstants.IN_PUSH_DIR);
            intaker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intaker.setPower(0);
            success = true;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initIntake\n" + e.toString());
        }

        try
        {
            dropservo = hwMap.get(Servo.class, "drop");
            dropPos = DropPos.CLOSED;
            setDropPos(dropPos);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR lifter - no dropservo drop\n" + e.toString());
            success = false;
        }

        return success;

    }

    public String toString(){
        return String.format(Locale.US,
                "intake %5d %4.2f",
                encPos, curSpd);
    }

    public void update()
    {
        if(intaker != null)
        {
            encPos = intaker.getCurrentPosition();
            curSpd = intaker.getVelocity();
        }
    }

    public void stop(){
        intaker.setPower(0);
    }

    public void suck(double pwr)
    {
        if(intaker != null) intaker.setPower(pwr);
    }


    public void setDropPos(DropPos pos)
    {
        dropPos = pos;
        dropLoc = dropPos.srvPos;
        if(dropservo != null) dropservo.setPosition(dropLoc);
    }

    public void adjDropPos(double incr)
    {
        dropLoc +=incr;
        if(dropservo != null) dropservo.setPosition(dropLoc);
    }

    public void toggleDropPos ()
    {
        if (dropPos == DropPos.OPEN) setDropPos(DropPos.CLOSED);
        else                          setDropPos(DropPos.OPEN);
    }

    public enum DropPos
    {
        OPEN(RobotConstants.LD_DROP_OPEN),
        CLOSED(RobotConstants.LD_DROP_CLOSE),
        MID(RobotConstants.LD_DROP_MID);

        public final double srvPos;

        DropPos(double srvPos) { this.srvPos = srvPos; }
    }



    private DcMotorEx intaker;
    public Servo dropservo;
    private DropPos dropPos = DropPos.CLOSED;
    private double dropLoc = DropPos.CLOSED.srvPos;
    protected HardwareMap hwMap;
    private static final String TAG = "SJH_INT";
    private int encPos;
    private double curSpd;
}
