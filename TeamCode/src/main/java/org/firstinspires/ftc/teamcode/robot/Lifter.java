package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

import androidx.annotation.NonNull;

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
        boolean success = true;
        try
        {
            liftMotor = hwMap.get(DcMotorEx.class, "lift");
            liftMotor.setDirection(DcMotor.Direction.FORWARD); //bevel gear reverses
            liftMotor.setPower(0);
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(RUN_USING_ENCODER);
            lastRunMode = RUN_USING_ENCODER;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR lifter - no liftMotor lift\n" + e.toString());
            success = false;
        }

        try
        {
            clampServo = hwMap.get(Servo.class, "clamp");
            clmpPos = ClampPos.MID;
            setClampPos(clmpPos);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR lifter - no clampServo clamp\n" + e.toString());
            success = false;
        }

        return success;
    }

    private int convLiftpos(LiftPos pos)
    {
        double degPos = pos.posDeg;
        int encPos = (int)(degPos*LIFTER_CPD);
        if(pos == LiftPos.HERE) encPos = lftCnts;
        return encPos;
    }

    public void setLiftPos(LiftPos pos)
    {
        tgtPos = pos;
        int encPos = convLiftpos(tgtPos);
        if(liftMotor != null) liftMotor.setTargetPosition(encPos);

        if(lastRunMode != RUN_TO_POSITION)
        {
            if(liftMotor != null) liftMotor.setMode(RUN_TO_POSITION);
            lastRunMode = RUN_TO_POSITION;
        }

        tgtVel = LIFTER_CPD * 90;
        if(liftMotor != null) liftMotor.setVelocity(tgtVel);
    }

    public void setLiftSpd(double pwr)
    {
        double power = pwr;
        if(Math.abs(power) < 0.05 && lastRunMode != RUN_TO_POSITION)
        {
            setLiftPos(LiftPos.HERE);
        }
        else if (Math.abs(power) >= 0.05)
        {
            if(lastRunMode != RUN_USING_ENCODER)
            {
                if(liftMotor != null) liftMotor.setMode(RUN_USING_ENCODER);
                lastRunMode = RUN_USING_ENCODER;
            }

            //safetycheck
            if (lftCnts <= convLiftpos(LiftPos.STOW) - SAFE_DEG * LIFTER_CPD ||
                lftCnts >= convLiftpos(LiftPos.GRAB) + SAFE_DEG * LIFTER_CPD) power = 0;

            tgtVel = power * LIFTER_CPD * 45;
            if(liftMotor != null) liftMotor.setVelocity(tgtVel);
        }
    }

    public void setClampPos(ClampPos pos)
    {
        clmpPos = pos;
        clmpLoc = clmpPos.srvPos;
        if(clampServo != null) clampServo.setPosition(clmpLoc);
    }

    public void adjClampPos(double incr)
    {
        clmpLoc +=incr;
        if(clampServo != null) clampServo.setPosition(clmpLoc);
    }

    public void toggleClampPos ()
    {
        if (clmpPos == ClampPos.OPEN) setClampPos(ClampPos.CLOSED);
        else                          setClampPos(ClampPos.OPEN);
    }

    public void update()
    {
        if(liftMotor != null) lftCnts = liftMotor.getCurrentPosition();
    }

    @NonNull
    public String toString()
    {
        return String.format(Locale.US,
                "lift %5d %4.2f %s %4.2f %s srvPos %s %4.2f",
                lftCnts, lftCnts/LIFTER_CPD, tgtPos, tgtVel, lastRunMode, clmpPos, clmpLoc);
    }

    public enum LiftPos
    {
        STOW(0.0),
        GRAB(180.0),
        DROP(150.0),
        HOLD(90.0),
        HERE(0.0);

        public final double posDeg;

        LiftPos(double posDeg) { this.posDeg = posDeg; }
    }

    public enum ClampPos
    {
        OPEN(0.3),
        CLOSED(0.6),
        MID(0.5);

        public final double srvPos;

        ClampPos(double srvPos) { this.srvPos = srvPos; }
    }

    public DcMotorEx liftMotor;
    public Servo clampServo;
    protected HardwareMap hwMap;

    private DcMotor.RunMode lastRunMode;
    private ClampPos clmpPos = ClampPos.MID;
    private double clmpLoc = ClampPos.MID.srvPos;
    private LiftPos tgtPos = LiftPos.STOW;
    private int lftCnts = convLiftpos(tgtPos);
    private double tgtVel = 0.0;
    private static final double SAFE_DEG = 5.0;

    private static final double LIFTER_CPER = 28; //quad encoder cnts/encoder rev
    private static final double LIFTER_INT_GEAR = 19.2; //Neverest 20
    private static final double LIFTER_EXT_GEAR = 1.0; //1:1 bevel
    private static final double LIFTER_CPR = LIFTER_CPER * LIFTER_INT_GEAR * LIFTER_EXT_GEAR; // cnts/outShaftRev
    private static final double LIFTER_CPD = LIFTER_CPR / 360.0;

    private static final String TAG = "SJH_LFT";
}
