package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Loader;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;


@TeleOp(name = "Testloader", group = "Test")
//@Disabled
public class Testloader extends InitLinearOpMode
{
    private static final double INCREMENT = 0.02;     // amount to step motor each CYCLE_MS cycle
    private static final int     CYCLE_MS = 20;       // period of each cycle
    private static final double   MAX_FWD =  1.0;     // Maximum FWD power applied to motor
    private static final double   MAX_REV = -1.0;     // Maximum REV power applied to motor

    // Define class members
    private double power = 0;

    private static final String TAG = "SJH_TLD";

    @Override
    public void runOpMode()
    {
        initCommon(this, false, false, false, false);

        Loader loader = new Loader(hardwareMap);
        loader.init();

        int p;

        // Wait for the start button
        dashboard.displayPrintf(0, "Press Start to run Motors.");

        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive())
        {
            p=0;

            gpad1.update();
            boolean step_up    = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean step_down  = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean zeroize    = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);

            if(step_up && power < MAX_FWD)         power += INCREMENT;
            else if(step_down && power > MAX_REV)  power -= INCREMENT;
            else if(zeroize)                       power = 0.0;

            loader.load(power);

            // Display the current value
            dashboard.displayPrintf(p++, "Motor Power %4.2f", power);
            RobotLog.dd(TAG, "Motor Power %4.2f", power);

            dashboard.displayPrintf(p++, "Press Stop to end test.");
            dashboard.displayPrintf( p++, "Incr power : Dpad up");
            dashboard.displayPrintf( p++, "Decr power : Dpad down");
            dashboard.displayPrintf( p, "Zero power : Dpad right");

            sleep(CYCLE_MS);
        }

        loader.load(0.0);

        dashboard.displayText(  1, "Done." );
    }
}
