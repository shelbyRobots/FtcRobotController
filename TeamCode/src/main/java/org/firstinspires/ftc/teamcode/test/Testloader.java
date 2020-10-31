package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
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
public class Testloader extends InitLinearOpMode
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

        int p;

        String motorName = "loader";
        DcMotor mot = null;

        try {
            mot = hardwareMap.dcMotor.get(motorName);
        }
        catch(IllegalArgumentException e)
        {
            RobotLog.ee(TAG, "Problem finding motor " + motorName);
        }

        RobotLog.dd(TAG, "Found motor " + motorName);
        if(mot != null)
        {
            mot.setDirection(LEFT_DIR);
            mot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Wait for the start button
        dashboard.displayPrintf(0, "Press Start to run Motors.");

        int encPos = 0;

        while(!isStarted())
        {
            if(mot != null) {
                encPos = mot.getCurrentPosition();
            }
            dashboard.displayPrintf(1, "CNT %d", encPos);

            sleep(10);
        }
        waitForStart();

        if(mot == null) return;

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive())
        {
            p=0;

            gpad1.update();
            boolean step_up    = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean step_down  = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean zeroize    = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);
            boolean toggle_mod = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);

            if(step_up && power < MAX_FWD)         power += INCREMENT;
            else if(step_down && power > MAX_REV)  power -= INCREMENT;
            else if(zeroize)                       power = 0.0;

            if(toggle_mod)
            {
                DcMotor.RunMode newMode = DcMotor.RunMode.RUN_USING_ENCODER;
                DcMotor.RunMode curMode = mot.getMode();
                if(curMode == DcMotor.RunMode.RUN_USING_ENCODER)
                    newMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
                mot.setMode(newMode);
            }

            // Display the current value
            dashboard.displayPrintf(MAX_MOTORS + p++, "Motor Power %4.2f", power);
            encPos = mot.getCurrentPosition();
            dashboard.displayPrintf(1, "Mot CNT:%d PWR:%.2f MOD:%s",
                 encPos, power, mot.getMode());

            mot.setPower(power);

            dashboard.displayPrintf(MAX_MOTORS + p++, "Press Stop to end test.");
            dashboard.displayPrintf(MAX_MOTORS + p++, "Incr power : Dpad up");
            dashboard.displayPrintf(MAX_MOTORS + p++, "Decr power : Dpad down");
            dashboard.displayPrintf(MAX_MOTORS + p++, "Zero power : Dpad right");
            dashboard.displayPrintf(MAX_MOTORS + p,   "Toggle mode: Right bumper");

            sleep(CYCLE_MS);
        }

        mot.setPower(0.0);

        dashboard.displayText(MAX_MOTORS + 1, "Done." );
    }
}
