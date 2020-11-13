package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;


@TeleOp(name = "TestShooter", group = "Test")
//@Disabled
public class TestShooter extends InitLinearOpMode
{
    private static final double INCREMENT = 3.0;     // amount to step motor each CYCLE_MS cycle
    private static final int     CYCLE_MS = 20;       // period of each cycle
    private static final double   MAX_DIST = 136;     // Maximum that we can shoot
    private static final double   MIN_DIST = 60;     // Minimum that we can shoot.
    private static final double   FAV_DIST = 70;
    // Define class members

    private static final String TAG = "SJH_TSH";

    @Override
    public void runOpMode()
    {
        initCommon(this, false, false, false, false);

        Shooter shooter = new Shooter(hardwareMap);
        shooter.init();

        double distance = FAV_DIST;


        // Wait for the start button
        dashboard.displayPrintf(0, "Press Start to run Motors.");
        while (!isStarted()) {
            shooter.update();
            dashboard.displayPrintf(1, shooter.toString());
            sleep(CYCLE_MS);
        }
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive())
        {

            shooter.update();
            gpad1.update();

            boolean step_up    = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean step_down  = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean zeroize    = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);
            boolean normal     = gpad1.just_pressed(ManagedGamepad.Button.D_LEFT);


            if (step_up && distance < MAX_DIST) {
                distance += INCREMENT;
                shooter.shoot(distance);
            }
            if (step_down && distance > MIN_DIST) {
                distance -= INCREMENT;
                shooter.shoot(distance);
            }
            if (zeroize) shooter.stop();
            if (normal){
                distance = FAV_DIST;
                shooter.shoot(distance);
            }

            dashboard.displayPrintf(1, shooter.toString());
            RobotLog.dd(TAG, shooter.toString());
            sleep(CYCLE_MS);
        }
        shooter.stop();
        dashboard.displayText(  1, "Done." );
    }
}
