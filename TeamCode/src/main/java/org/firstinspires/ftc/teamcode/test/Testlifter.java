package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Lifter;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

@TeleOp(name = "Testlifter", group = "Test")

public class Testlifter extends InitLinearOpMode {
    private static final int CYCLE_MS = 20;       // period of each cycle

    // Define class members
    private final ElapsedTime period  = new ElapsedTime();

    private static final String TAG = "SJH_TLF";

    @Override
    public void runOpMode() {
        initCommon(this, false, false, false, false);

        Lifter lifter = new Lifter(hardwareMap);
        lifter.init();
        int p;
        final double srvIncr = 0.05;

        // Wait for the start button
        dashboard.displayPrintf(0, "Press Start to run Motors.");

        while (!isStarted()) {
            lifter.update();

            String lStr = lifter.toString();
            dashboard.displayPrintf(3, lStr);
            RobotLog.dd(TAG, lStr);

            waitForTick(CYCLE_MS);
        }
        waitForStart();

        if (lifter.liftMotor == null || lifter.clampServo == null) return;

        // Ramp motor speeds till stop pressed.
        boolean usePctSpd = false;
        double pctSpdSafety = 0.25;
        while (opModeIsActive()) {
            p = 0;

            lifter.update();
            gpad1.update();
            double lftPwr = -gpad1.value(ManagedGamepad.AnalogInput.L_STICK_Y);
            boolean ungrip = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);
            boolean grip = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
            boolean stow = gpad1.just_pressed(ManagedGamepad.Button.A);
            boolean grab = gpad1.just_pressed(ManagedGamepad.Button.B);
            boolean hold = gpad1.just_pressed(ManagedGamepad.Button.X);
            boolean drop = gpad1.just_pressed(ManagedGamepad.Button.Y);
            boolean lsrv = gpad1.just_pressed(ManagedGamepad.Button.D_LEFT);
            boolean rsrv = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);
            boolean stop = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean tglS = gpad1.just_pressed(ManagedGamepad.Button.R_TRIGGER);

            if (grip) lifter.setClampPos(Lifter.ClampPos.CLOSED);
            else if (ungrip) lifter.setClampPos(Lifter.ClampPos.OPEN);
            else if (lsrv) lifter.adjClampPos(-srvIncr);
            else if (rsrv) lifter.adjClampPos(srvIncr);

            if(tglS) usePctSpd = !usePctSpd;

            if(usePctSpd)
            {
                lifter.setPctSpd(lftPwr * pctSpdSafety);
            }
            else
            {
                if (stow) lifter.setLiftPos(Lifter.LiftPos.STOW);
                else if (grab) lifter.setLiftPos(Lifter.LiftPos.GRAB);
                else if (hold) lifter.setLiftPos(Lifter.LiftPos.HOLD);
                else if (drop) lifter.setLiftPos(Lifter.LiftPos.DROP);
                else lifter.setLiftSpd(lftPwr);
            }

            if(stop) lifter.liftMotor.setVelocity(0.0);

            // Display the current value
            String lStr = lifter.toString();
            dashboard.displayPrintf(p++, lStr);
            RobotLog.dd(TAG, lStr);

            dashboard.displayPrintf(p++, "Press Stop to end test.");
            dashboard.displayPrintf(p++, "Decr grip : Dpad left");
            dashboard.displayPrintf(p++, "Incr grip : Dpad right");
            dashboard.displayPrintf(p++, "Open  grp : L_BUMP");
            dashboard.displayPrintf(p++, "Close grp : R_BUMP");
            dashboard.displayPrintf(p++, "Toggle spdMod : R_TRG");
            dashboard.displayPrintf(p++, "Arm Speed : L_Stick_Y");
            dashboard.displayPrintf(p++, "Stow : A");
            dashboard.displayPrintf(p++, "Grab : B");
            dashboard.displayPrintf(p++, "Hold : X");
            dashboard.displayPrintf(p++, "Drop : Y");

            dashboard.displayPrintf(p, "Zero power : Dpad down");

            waitForTick(CYCLE_MS);
        }

        lifter.liftMotor.setPower(0.0);
        dashboard.displayText(1, "Done.");
    }

    public void waitForTick(long periodMs)
    {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

}