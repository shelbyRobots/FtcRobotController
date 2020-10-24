package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

public class Lifter
{
    public Lifter(HardwareMap map)
    {
        super();
        this.hwMap = map;
    }

    public boolean init()
    {
        boolean success = false;
        try
        {
            liftMotor = hwMap.get(DcMotorEx.class, "elev");
            clampServo = hwMap.get(Servo.class, "clamp");
            success = true;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initCollector\n" + e.toString());
        }

        if(liftMotor != null)
        {
            liftMotor.setDirection(DcMotor.Direction.FORWARD);
            liftMotor.setPower(0);
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(RUN_USING_ENCODER);
            lastRunMode = RUN_USING_ENCODER;

            clmpPos = ClampPos.CLOSED;
            setClampPos(clmpPos);
        }

        return success;
    }

    public void setLiftPos(LiftPos pos)
    {
        final double stwDeg = 0.0;
        final double grbDeg = 180.0;
        final double hldDeg = 90.0;
        final double drpDeg = 150.0;

        switch (pos)
        {
            case STOW : liftMotor.setTargetPosition((int)(stwDeg*LIFTER_CPD)); break;
            case GRAB : liftMotor.setTargetPosition((int)(grbDeg*LIFTER_CPD)); break;
            case DROP : liftMotor.setTargetPosition((int)(hldDeg*LIFTER_CPD)); break;
            case HOLD : liftMotor.setTargetPosition((int)(drpDeg*LIFTER_CPD)); break;
            case HERE : liftMotor.setTargetPosition(lftCnts); break;
        }

        if(lastRunMode != RUN_TO_POSITION) liftMotor.setMode(RUN_TO_POSITION);
        lastRunMode = RUN_TO_POSITION;

        liftMotor.setVelocity(LIFTER_CPD * 10);
    }

    public void setLiftSpd(double pwr)
    {
        if(Math.abs(pwr) < 0.05 && lastRunMode != RUN_TO_POSITION)
        {
            setLiftPos(LiftPos.HERE);
        }
        else
        {
            if(lastRunMode != RUN_USING_ENCODER)
            {
                liftMotor.setMode(RUN_USING_ENCODER);
                lastRunMode = RUN_USING_ENCODER;
            }
            liftMotor.setVelocity(pwr * LIFTER_CPD * 20);
        }
    }

    public void setClampPos(ClampPos pos)
    {
        double cPos;
        if (pos == ClampPos.OPEN) cPos = 0.3;
        else                      cPos = 0.6;

        clmpPos = pos;
        clampServo.setPosition(cPos);
    }

    public void toggleClampPos ()
    {
        if (clmpPos == ClampPos.OPEN) setClampPos(ClampPos.CLOSED);
        else                          setClampPos((ClampPos.OPEN));
    }

    public void update()
    {
        lftCnts = liftMotor.getCurrentPosition();
    }

    public String toString()
    {
        return String.format(Locale.US,
                "elevCnts %5d %4.2f deg", lftCnts, lftCnts/LIFTER_CPD);
    }

    public enum LiftPos
    {
        STOW,
        GRAB,
        DROP,
        HOLD,
        HERE
    }

    public enum ClampPos
    {
        OPEN,
        CLOSED
    }

    private DcMotorEx liftMotor;
    private Servo clampServo;

    private DcMotor.RunMode lastRunMode;
    private ClampPos clmpPos;

    protected HardwareMap hwMap;

    private final double LIFTER_CPER = 28; //quad encoder cnts/encoder rev
    private final double LIFTER_INT_GEAR = 19.2; //Neverest 20
    private final double LIFTER_EXT_GEAR = 1.0;
    private final double LIFTER_CPR = LIFTER_CPER * LIFTER_INT_GEAR * LIFTER_EXT_GEAR; // cnts/outShaftRev
    private final double LIFTER_CPD = LIFTER_CPR / 360.0;

    private int lftCnts = 0;

    private static final String TAG = "SJH_LFT";
}
