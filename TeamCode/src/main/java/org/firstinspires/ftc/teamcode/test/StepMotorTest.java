package org.firstinspires.ftc.teamcode.test;

import android.util.SparseArray;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

/**
 * This OpMode steps n motors speed up and down based on D-Pad user inputs.
 * This code assumes a DC motor configured with the name "testmotor#".
 * If motors are on a bot, left side motors should be odd and right side even
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 */
@Autonomous(name = "Concept: Step Motor Speed", group = "Test")
//@Disabled
public class StepMotorTest extends InitLinearOpMode
{

    private static final double INCREMENT = 0.02;     // amount to step motor each CYCLE_MS cycle
    private static final int     CYCLE_MS = 50;       // period of each cycle
    private static final double   MAX_FWD =  1.0;     // Maximum FWD power applied to motor
    private static final double   MAX_REV = -1.0;     // Maximum REV power applied to motor

    // Define class members
    private double power = 0;

    private static DcMotor.Direction LEFT_DIR  = DcMotor.Direction.FORWARD;

    private static final int MAX_MOTORS = 4;

    private static final String TAG = "SJH_RMT";

    @Override
    public void runOpMode()
    {
        initCommon(this, false, false, false, false);

        SparseArray<DcMotor> motors = new SparseArray<>(MAX_MOTORS);

        int p = 0;

        for(int m = 0; m < MAX_MOTORS; m++)
        {
            String motorName = "testmotor" + m;
            DcMotor mot;
            try
            {
                mot = hardwareMap.dcMotor.get(motorName);

                if (mot instanceof DcMotorEx)
                {
                    DcMotorEx lex = (DcMotorEx) mot;
                    PIDFCoefficients pid;
                    pid = lex.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
                    RobotLog.dd(TAG, "RUN_TO_POS Motor %s PIDs. P:%.2f I:%.2f D:%.2f F:%.2f",
                            motorName, pid.p, pid.i, pid.d, pid.f);
                    pid = lex.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                    RobotLog.dd(TAG, "RUN_USING_ENC Motor %s PIDs. P:%.2f I:%.2f D:%.2f F:%.2f",
                            motorName, pid.p, pid.i, pid.d, pid.f);
                    //pid = lex.getPIDCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //RobotLog.dd(TAG, "RUN_WITHOUT_ENC Motor %s PIDs. P:%.2f I:%.2f D:%.2f",
                    //        motorName, pid.p, pid.i, pid.d);
                }
            }
            catch(IllegalArgumentException e)
            {
                RobotLog.ee(TAG, "Problem finding motor " + motorName);
                continue;
            }

            motors.put(m, mot);
            RobotLog.dd(TAG, "Found motor " + motorName);
            if(mot != null)
            {
                mot.setDirection(LEFT_DIR);
                mot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
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
            for(int m = 0; m < MAX_MOTORS; m++)
            {
                DcMotor mot = motors.get(m);
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
            dashboard.displayPrintf(MAX_MOTORS + p++, "AvgEncTime %4.3f", at);
            sleep(10);
        }
        waitForStart();

        // Ramp motor speeds till stop pressed.
        int lmult = 1;
        while(opModeIsActive())
        {
            p=0;
            tt = 0.0;
            numCycles = 0;
            gpad1.update();
            boolean step_up    = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean step_down  = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean zeroize    = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);
            boolean toggle_trn = gpad1.just_pressed(ManagedGamepad.Button.D_LEFT);
            boolean toggle_mod = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);

            if(step_up && power < MAX_FWD)         power += INCREMENT;
            else if(step_down && power > MAX_REV)  power -= INCREMENT;
            else if(zeroize)                       power = 0.0;

            if(toggle_trn) lmult *=-1;

            if(toggle_mod)
            {
                DcMotor.RunMode newMode = DcMotor.RunMode.RUN_USING_ENCODER;
                DcMotor.RunMode curMode = motors.get(0).getMode();
                if(curMode == DcMotor.RunMode.RUN_USING_ENCODER)
                    newMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
                for(int m = 0; m < MAX_MOTORS; m++)
                {
                    DcMotor mot = motors.get(m);
                    if(mot != null) mot.setMode(newMode);
                }
            }

            // Display the current value
            dashboard.displayPrintf(MAX_MOTORS + p++, "Motor Power %4.2f", power);
            for(int m = 0; m < MAX_MOTORS; m++)
            {
                double opwr = power;
                DcMotor mot = motors.get(m);
                if(mot != null)
                {
                    if(m%2 == 0) opwr = power * lmult;

                    encPos = mot.getCurrentPosition();
                    dashboard.displayPrintf(m, "Mot_%d CNT:%d PWR:%.2f MOD:%s",
                            m, encPos, opwr, mot.getMode());
                    t1 = timer.milliseconds();
                    mot.setPower(opwr);
                    t2 = timer.milliseconds();
                    dt = t2 - t1;
                    tt += dt;
                }
            }

            at = tt / ++numCycles;
            dashboard.displayPrintf(MAX_MOTORS + p++, "AvgSetTime %4.3f", at);

            dashboard.displayPrintf(MAX_MOTORS + p++, "Press Stop to end test.");
            dashboard.displayPrintf(MAX_MOTORS + p++, "Incr power : Dpad up");
            dashboard.displayPrintf(MAX_MOTORS + p++, "Decr power : Dpad down");
            dashboard.displayPrintf(MAX_MOTORS + p++, "Zero power : Dpad right");
            dashboard.displayPrintf(MAX_MOTORS + p++, "Toggle turn: Dpad left");
            dashboard.displayPrintf(MAX_MOTORS + p,   "Toggle mode: Right bumper");

            sleep(CYCLE_MS);
        }

        for(int m = 0; m < MAX_MOTORS; m++)
        {
            DcMotor mot = motors.get(m);
            if(mot != null)
            {
                mot.setPower(0.0);
            }
        }

        dashboard.displayText(MAX_MOTORS + 1, "Done." );
    }
}
