package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
        this.hwMap = map;
    }

    public boolean init()
    {
        boolean success = true;
        try
        {
            liftMotor = hwMap.get(DcMotorEx.class, "lift");
            liftMotor.setDirection(RobotConstants.WA_DIR);
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
            clmpPos = ClampPos.CLOSED;
            setClampPos(clmpPos);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR lifter - no clampServo clamp\n" + e.toString());
            success = false;
        }

        try
        {
            wobGuide = hwMap.get(Servo.class, "wobGuide");
            guidePos = GuidePos.OPEN;
            setGuidePos(guidePos);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR lifter - no wobGuide clamp\n" + e.toString());
            success = false;
        }

        return success;
    }

    private int convLiftpos(LiftPos pos)
    {
        double degPos = pos.posDeg;
        int encPos = (int) (degPos * LIFTER_CPD) - LIFTER_OFST;
        if (pos == LiftPos.HERE) encPos = lftCnts;
        return encPos;
    }

    public void setLiftPos(LiftPos pos)
    {
        tgtPos = pos;
        int encPos = convLiftpos(tgtPos);
        if (liftMotor != null) liftMotor.setTargetPosition(encPos);

        if (lastRunMode != RUN_TO_POSITION)
        {
            if (liftMotor != null) liftMotor.setMode(RUN_TO_POSITION);
            lastRunMode = RUN_TO_POSITION;
        }

        tgtVel = LIFTER_CPD * 180;
        if (liftMotor != null) liftMotor.setVelocity(tgtVel);
    }

    public void setLiftSpd(double pwr)
    {
        if (Math.abs(pwr) < 0.05 && lastRunMode != RUN_TO_POSITION)
        {
            setLiftPos(LiftPos.HERE);
        }
        else if (Math.abs(pwr) >= 0.05)
        {
            if (lastRunMode != RUN_USING_ENCODER)
            {
                if (liftMotor != null) liftMotor.setMode(RUN_USING_ENCODER);
                lastRunMode = RUN_USING_ENCODER;
            }

            //safetycheck
//            if (lftCnts <= convLiftpos(LiftPos.STOW) - SAFE_DEG * LIFTER_CPD ||
//                lftCnts >= convLiftpos(LiftPos.GRAB) + SAFE_DEG * LIFTER_CPD) power = 0;

            tgtVel = pwr * LIFTER_CPD * 180;
            if (liftMotor != null) liftMotor.setVelocity(tgtVel);
        }
    }

    public void setPctSpd(double pwr)
    {
        if (liftMotor != null) liftMotor.setVelocity(pwr*LIFTER_MAXCPS);
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

    public void setGuidePos(GuidePos pos)
    {
        guidePos = pos;
        guideLoc = guidePos.srvPos;
        if(wobGuide != null) wobGuide.setPosition(guideLoc);
    }

    public void adjGuidePos(double incr)
    {
        guideLoc +=incr;
        if(wobGuide != null) wobGuide.setPosition(guideLoc);
    }

    public void toggleGuidePos ()
    {
        if (guidePos == GuidePos.OPEN) setGuidePos(GuidePos.CLOSED);
        else                           setGuidePos(GuidePos.OPEN);
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
        STOW(RobotConstants.WA_ARM_STOW),
        GRAB(RobotConstants.WA_ARM_GRAB),
        DROP(RobotConstants.WA_ARM_DROP),
        HOLD(RobotConstants.WA_ARM_HOLD),
        HERE(0.0);

        public final double posDeg;

        LiftPos(double posDeg) { this.posDeg = posDeg; }
    }

    public enum ClampPos
    {
        OPEN(RobotConstants.WA_CLAMP_OPEN),
        CLOSED(RobotConstants.WA_CLAMP_GRAB),
        MID(RobotConstants.WA_CLAMP_MID);

        public final double srvPos;

        ClampPos(double srvPos) { this.srvPos = srvPos; }
    }

    public enum GuidePos
    {
        OPEN(RobotConstants.WG_CLAMP_OPEN),
        CLOSED(RobotConstants.WG_CLAMP_GRAB),
        MID(RobotConstants.WG_CLAMP_MID);

        public final double srvPos;

        GuidePos(double srvPos) { this.srvPos = srvPos; }
    }

    public DcMotorEx liftMotor;
    public Servo clampServo;
    public Servo wobGuide;
    protected HardwareMap hwMap;

    private DcMotor.RunMode lastRunMode;
    private ClampPos clmpPos = ClampPos.MID;
    private double clmpLoc = ClampPos.MID.srvPos;
    private GuidePos guidePos = GuidePos.MID;
    private double guideLoc = GuidePos.MID.srvPos;
    private LiftPos tgtPos = LiftPos.STOW;
    private int lftCnts = convLiftpos(tgtPos);
    private double tgtVel = 0.0;
    private static final double SAFE_DEG = 5.0;

    private static final double LIFTER_CPER = 28; //quad encoder cnts/encoder rev
    private static final double LIFTER_INT_GEAR = 19.2; //Neverest 20
    private static final double LIFTER_EXT_GEAR = RobotConstants.WA_GEAR; //1:1 bevel
    private static final double LIFTER_CPR = LIFTER_CPER * LIFTER_INT_GEAR * LIFTER_EXT_GEAR; // cnts/outShaftRev
    private static final double LIFTER_CPD = LIFTER_CPR / 360.0;
    private static final double LIFTER_MAXCPS = Motors.MotorModel.AM_NEVEREST_ORBITAL_20.getRpm();
    private static final int    LIFTER_OFST = (int)(LiftPos.STOW.posDeg * LIFTER_CPD);

    private static final String TAG = "SJH_LFT";
}
