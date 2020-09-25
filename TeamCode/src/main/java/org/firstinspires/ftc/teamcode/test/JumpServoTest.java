package org.firstinspires.ftc.teamcode.test;

import android.util.SparseArray;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

/**
 * This OpMode jumps n servos to preset positions based on D-Pad user inputs.
 * This code assumes a servo configured with the name "testservo#".
 *
 * CYCLE_MS sets the update period.
 */
@Autonomous(name = "Concept: Jump Servo Position", group = "Test")
//@Disabled
public class JumpServoTest extends InitLinearOpMode
{
    private static final int     MICRO_MIN = 600;
    private static final int     MICRO_MAX = 2400;
    private static final int     MICRO_RNG = MICRO_MAX - MICRO_MIN;
    private static final int     CYCLE_MS  = 50;       // period of each cycle
    private static final double  MAX_POS   =  1.0;     // Maximum servo pos
    private static final double  MIN_POS   =  0.0;     // Minimum servo pos
    private static final double  MID_POS   =  0.5;
    private static final double  POS_1000  = (1000-MICRO_MIN)/MICRO_RNG;
    private static final double  POS_2000  = (2000-MICRO_MIN)/MICRO_RNG;
    private double pos = MID_POS;

    // Define class members
    private static final int MAX_SERVOS = 4;

    private static final String TAG = "SJH_SJT";

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, false, false, false);

        SparseArray<Servo> servos = new SparseArray<>(MAX_SERVOS);

        for(int m = 0; m < MAX_SERVOS; m++)
        {
            String servoName = "testservo" + m;
            Servo srv;
            try
            {
                srv = hardwareMap.servo.get(servoName);
            }
            catch(Exception e)
            {
                RobotLog.ee(TAG, "Problem finding servo " + servoName);
                continue;
            }

            servos.put(m, srv);
            RobotLog.dd(TAG, "Found motor " + servoName);
            if(srv != null)
            {
                srv.setPosition(pos);
            }
        }

        // Wait for the start button
        dashboard.displayText(0, "Press Start to run servos.");
        while(!isStarted())
        {
            for(int m = 0; m < MAX_SERVOS; m++)
            {
                Servo srv = servos.get(m);
                if(srv != null)
                {
                    double cpos = srv.getPosition();
                    int micros = (int) (cpos * MICRO_RNG) + MICRO_MIN;
                    dashboard.displayPrintf(m, "SRV_%d %.2f %d", m, cpos, micros);
                }
            }
            sleep(10);
        }
        waitForStart();

        while(opModeIsActive())
        {
            gpad1.update();
            boolean jump_max    = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean jump_min    = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean jump_mid    = gpad1.just_pressed(ManagedGamepad.Button.A);
            boolean jump_1000   = gpad1.just_pressed(ManagedGamepad.Button.D_LEFT);
            boolean jump_2000   = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);

            if     (jump_max)    pos = MAX_POS;
            else if(jump_min)    pos = MIN_POS;
            else if(jump_mid)    pos = MID_POS;
            else if(jump_1000)   pos = POS_1000;
            else if(jump_2000)   pos = POS_2000;


            for(int m = 0; m < MAX_SERVOS; m++)
            {
                Servo srv = servos.get(m);
                if(srv != null)
                {
                    int micros = (int)(pos * MICRO_RNG) + MICRO_MIN;
                    dashboard.displayPrintf(m, "Servo pos %4.2f %d", pos, micros);
                    srv.setPosition(pos);
                }
            }

            dashboard.displayText(MAX_SERVOS + 1, "Press Stop to end test.");
            dashboard.displayText(MAX_SERVOS + 2, "Max pos : Dpad up");
            dashboard.displayText(MAX_SERVOS + 3, "Min pos : Dpad down");
            dashboard.displayText(MAX_SERVOS + 4, "Mid pos (midpt) : A");
            dashboard.displayText(MAX_SERVOS + 5, "1000 (0.22) pos : Dpad left");
            dashboard.displayText(MAX_SERVOS + 6, "2000 (0.78) pos : Dpad right");

            sleep(CYCLE_MS);
        }

        dashboard.displayText(MAX_SERVOS + 1, "Done." );
    }
}
