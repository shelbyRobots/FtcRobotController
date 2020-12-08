package org.firstinspires.ftc.teamcode.test;

import android.util.SparseArray;
import android.util.SparseBooleanArray;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Motors;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

import java.util.ArrayList;
import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

/**
 * This OpMode steps n motors speed up and down based on D-Pad user inputs.
 * This code assumes a DC motor configured with the name "testmotor#".
 * If motors are on a bot, left side motors should be odd and right side even
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 */
@TeleOp(name = "TestMotors", group = "Test")
//@Disabled
public class StepMotorTest extends InitLinearOpMode
{
    private static final double INCREMENT = 0.02;     // amount to step motor each CYCLE_MS cycle
    private static final int     CYCLE_MS = 50;       // period of each cycle
    private static final double   MAX_FWD =  1.0;     // Maximum FWD power applied to motor
    private static final double   MAX_REV = -1.0;     // Maximum REV power applied to motor

    // Define class members
    private static final DcMotor.Direction LFT_DIR  = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction RGT_DIR  = DcMotor.Direction.FORWARD;

    private static final int MAX_MOTORS = 4;

    private double power = 0;
    private final ArrayList<Double> motPwrs = new ArrayList<>(MAX_MOTORS);

    public static Motors.MotorModel DT_MOTOR = Motors.MotorModel.GOBILDA_5202_19_2;
    private final double MAX_CPS = DT_MOTOR.getCpr() * DT_MOTOR.getRpm() / 60.0;
    private boolean useSpd = false;

    private static final String TAG = "SJH_RMT";

    @Override
    public void runOpMode()
    {
        initCommon(this, false, false, false, false);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        LynxModule.BulkCachingMode bcm = LynxModule.BulkCachingMode.AUTO;
        String bcmStr = "A";
        for (LynxModule module : allHubs)
        {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        SparseArray<DcMotorEx> motors = new SparseArray<>(MAX_MOTORS);
        SparseBooleanArray   actLst = new SparseBooleanArray(MAX_MOTORS);

        int p;

        for(int m = 0; m < MAX_MOTORS; m++)
        {
            String motorName = "testmotor" + m;
            DcMotorEx mot;
            try
            {
                mot = hardwareMap.get(DcMotorEx.class, motorName);

                PIDFCoefficients pid;
                pid = mot.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
                RobotLog.dd(TAG, "RUN_TO_POS Motor %s PIDs. P:%.2f I:%.2f D:%.2f F:%.2f",
                        motorName, pid.p, pid.i, pid.d, pid.f);
                pid = mot.getPIDFCoefficients(RUN_USING_ENCODER);
                RobotLog.dd(TAG, "RUN_USING_ENC Motor %s PIDs. P:%.2f I:%.2f D:%.2f F:%.2f",
                        motorName, pid.p, pid.i, pid.d, pid.f);

            }
            catch(IllegalArgumentException e)
            {
                RobotLog.ee(TAG, "Problem finding motor " + motorName);
                continue;
            }

            motors.put(m, mot);
            actLst.put(m, true);
            motPwrs.add(0.0);
            RobotLog.dd(TAG, "Found motor " + motorName);

            if(m%2 == 0) mot.setDirection(LFT_DIR);
            else         mot.setDirection(RGT_DIR);
            mot.setMode(RUN_USING_ENCODER);
        }

        // Wait for the start button
        dashboard.displayPrintf(0, "Press Start to run Motors.");
        ElapsedTime timer = new ElapsedTime();
        double t1;
        double t2;
        double dt;
        double tt = 0.0;
        double at;
        int encPos;
        int numCycles = 0;
        while(!isStarted())
        {
            p=0;

            gpad1.update();
            boolean toggle_bcm = gpad1.just_pressed(ManagedGamepad.Button.L_STICK_BUTTON);
            if(toggle_bcm)
            {
                if(bcm == LynxModule.BulkCachingMode.AUTO)
                {
                    bcm = LynxModule.BulkCachingMode.OFF;
                    bcmStr = "O";
                }
                else
                {
                    bcm = LynxModule.BulkCachingMode.AUTO;
                    bcmStr = "A";
                }
                for (LynxModule module : allHubs)
                {
                    module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
                }
            }

            for(int m = 0; m < MAX_MOTORS; m++)
            {
                DcMotorEx mot = motors.get(m);
                if(mot != null)
                {
                    t1 = timer.milliseconds();
                    encPos = mot.getCurrentPosition();
                    t2 = timer.milliseconds();
                    dt = t2 - t1;
                    dashboard.displayPrintf(m, "CNT_%d %d encTime=%4.3f", m, encPos, dt);
                    dt = t2 - t1;
                    tt += dt;
                }
            }
            at = tt / ++numCycles;
            dashboard.displayPrintf(MAX_MOTORS + p, "AvgEncTime %4.3f", at);
            sleep(10);
        }
        waitForStart();

        while(opModeIsActive())
        {
            p=0;
            tt = 0.0;
            numCycles = 0;
            gpad1.update();
            boolean fwd        = gpad1.pressed(ManagedGamepad.Button.D_UP);
            boolean rev        = gpad1.pressed(ManagedGamepad.Button.D_DOWN);
            boolean ltrn       = gpad1.pressed(ManagedGamepad.Button.D_LEFT);
            boolean rtrn       = gpad1.pressed(ManagedGamepad.Button.D_RIGHT);

            boolean tglAct0    = gpad1.just_pressed(ManagedGamepad.Button.A);
            boolean tglAct1    = gpad1.just_pressed(ManagedGamepad.Button.B);
            boolean tglAct2    = gpad1.just_pressed(ManagedGamepad.Button.X);
            boolean tglAct3    = gpad1.just_pressed(ManagedGamepad.Button.Y);

            boolean incrPwr    = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
            boolean decrPwr    = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);
            boolean toggle_bcm = gpad1.just_pressed(ManagedGamepad.Button.L_STICK_BUTTON);
            boolean toggle_dir = gpad1.just_pressed(ManagedGamepad.Button.R_STICK_BUTTON);
            boolean toggle_mod = gpad1.just_pressed(ManagedGamepad.Button.L_TRIGGER);
            boolean toggle_spd = gpad1.just_pressed(ManagedGamepad.Button.R_TRIGGER);

            if(toggle_bcm)
            {
                if(bcm == LynxModule.BulkCachingMode.AUTO)
                {
                    bcm = LynxModule.BulkCachingMode.OFF;
                    bcmStr = "O";
                }
                else
                {
                    bcm = LynxModule.BulkCachingMode.AUTO;
                    bcmStr = "A";
                }
                for (LynxModule module : allHubs)
                {
                    module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
                }
            }

            if(toggle_mod)
            {
                DcMotor.RunMode newMode = RUN_USING_ENCODER;
                DcMotor.RunMode curMode = motors.get(0).getMode();
                if(curMode == RUN_USING_ENCODER)
                    newMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
                for(int m = 0; m < MAX_MOTORS; m++)
                {
                    DcMotorEx mot = motors.get(m);
                    if(mot != null) mot.setMode(newMode);
                }
            }

            if(toggle_dir)
            {
                for(int m = 0; m < MAX_MOTORS; m++)
                {
                    DcMotorEx mot = motors.get(m);
                    if(mot != null)
                    {
                        DcMotor.Direction dir = mot.getDirection();
                        if(dir == DcMotorSimple.Direction.FORWARD)
                            dir = DcMotorSimple.Direction.REVERSE;
                        else
                            dir = DcMotorSimple.Direction.FORWARD;
                        mot.setDirection(dir);
                    }
                }
            }

            if(toggle_spd) useSpd = !useSpd;

            if     (incrPwr)  power += INCREMENT;
            else if(decrPwr)  power -= INCREMENT;
            power = Range.clip(power, MAX_REV, MAX_FWD);

            int lScl = 0;
            int rScl = 0;
            if(fwd)        { lScl =  1; rScl =  1; }
            else if(rev)   { lScl = -1; rScl = -1; }
            else if (rtrn) { lScl =  1; rScl = -1; }
            else if (ltrn) { lScl = -1; rScl =  1; }

            if(tglAct0) actLst.put(0, !actLst.get(0));
            if(tglAct1) actLst.put(1, !actLst.get(1));
            if(tglAct2) actLst.put(2, !actLst.get(2));
            if(tglAct3) actLst.put(3, !actLst.get(3));

            for(int m = 0; m < MAX_MOTORS; m++)
            {
                if(m%2 == 0) motPwrs.set(m, lScl * power);
                else         motPwrs.set(m, rScl * power);
                DcMotorEx mot = motors.get(m);
                if(mot != null)
                {
                    t1 = timer.milliseconds();
                    encPos = mot.getCurrentPosition();
                    if(actLst.get(m))
                    {
                        if(useSpd)
                        {
                            mot.setVelocity(MAX_CPS * motPwrs.get(m));
                        }
                        else
                        {
                            mot.setPower(motPwrs.get(m));
                        }
                    }
                    t2 = timer.milliseconds();
                    dt = t2 - t1;
                    tt += dt;
                    String rMode;
                    switch (mot.getMode())
                    {
                        case RUN_USING_ENCODER:   rMode = "U"; break;
                        case RUN_WITHOUT_ENCODER: rMode = "W"; break;
                        default:                  rMode = "X"; break;
                    }
                    String dirStr = "U";
                    switch (mot.getDirection())
                    {
                        case FORWARD: dirStr = "F"; break;
                        case REVERSE: dirStr = "R"; break;
                    }
                    dashboard.displayPrintf(m, "C:%5d P:%.2f RM:%s D:%s",
                        encPos, motPwrs.get(m), rMode, dirStr);
                }
            }
            dashboard.displayPrintf(MAX_MOTORS + p++, "Pwr %4.2f BM:%s SM:%s",
                power, bcmStr, useSpd);

            at = tt / ++numCycles;
            dashboard.displayPrintf(MAX_MOTORS + p++, "AvgGetSetTime %4.3f", at);

            dashboard.displayPrintf(MAX_MOTORS + p++, "Press Stop to end test.");
            dashboard.displayPrintf(MAX_MOTORS + p++, "Incr power : R bump");
            dashboard.displayPrintf(MAX_MOTORS + p++, "Decr power : L bump");
            dashboard.displayPrintf(MAX_MOTORS + p++, "TglAct 0,1,2,3 : A,B,X,Y");
            dashboard.displayPrintf(MAX_MOTORS + p++, "FWD      : Dpad Up");
            dashboard.displayPrintf(MAX_MOTORS + p++, "REV      : Dpad Down");
            dashboard.displayPrintf(MAX_MOTORS + p++, "LFT Turn : Dpad Left");
            dashboard.displayPrintf(MAX_MOTORS + p++, "RGT Turn : Dpad Right");
            dashboard.displayPrintf(MAX_MOTORS + p,   "Tgl Blk mode: L plunge");
            dashboard.displayPrintf(MAX_MOTORS + p,   "Tgl Dirs    : R plunge");
            dashboard.displayPrintf(MAX_MOTORS + p,   "Tgl Run mode: L trig");
            dashboard.displayPrintf(MAX_MOTORS + p,   "Tgl Spd mode: R trig");

            sleep(CYCLE_MS);
        }

        for(int m = 0; m < MAX_MOTORS; m++)
        {
            DcMotorEx mot = motors.get(m);
            if(mot != null)
            {
                mot.setPower(0.0);
            }
        }

        dashboard.displayText(MAX_MOTORS + 1, "Done." );
    }
}
