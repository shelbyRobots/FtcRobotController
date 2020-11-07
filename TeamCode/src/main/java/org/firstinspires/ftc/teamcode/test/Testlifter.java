package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Lifter;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

@TeleOp(name = "Testlifter", group = "Test")

public class Testlifter extends InitLinearOpMode {
    private static final int CYCLE_MS = 20;       // period of each cycle

    // Define class members

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

        int encPos = 0;
        double srvPos = 0;
        while (!isStarted()) {
            if (lifter.liftMotor != null) {
                encPos = lifter.liftMotor.getCurrentPosition();
            }
            if (lifter.clampServo != null){
                srvPos = lifter.clampServo.getPosition();
            }
            dashboard.displayPrintf(1, "CNT %d", encPos);
            dashboard.displayPrintf(2, "CNT %d", srvPos);

            sleep(10);
        }
        waitForStart();

        if (lifter.liftMotor == null || lifter.clampServo == null) return;

        // Ramp motor speeds till stop pressed.
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

            if (grip) lifter.setClampPos(Lifter.ClampPos.CLOSED);
            else if (ungrip) lifter.setClampPos(Lifter.ClampPos.OPEN);
            else if (lsrv) lifter.adjClampPos(-srvIncr);
            else if (rsrv) lifter.adjClampPos(srvIncr);

            double safety = 0.5;
            if(lftPwr > safety) lftPwr = safety;
            if(lftPwr < -safety) lftPwr = -safety;

            if(stop) lifter.liftMotor.setVelocity(0.0);
            else
            {
                if (stow) lifter.setLiftPos(Lifter.LiftPos.STOW);
                else if (grab) lifter.setLiftPos(Lifter.LiftPos.GRAB);
                else if (hold) lifter.setLiftPos(Lifter.LiftPos.HOLD);
                else if (drop) lifter.setLiftPos(Lifter.LiftPos.DROP);
                else lifter.setLiftSpd(lftPwr);
            }

            // Display the current value
            String lStr = lifter.toString();
            dashboard.displayPrintf(p++, lStr);
            RobotLog.dd(TAG, lStr);

            dashboard.displayPrintf(p++, "Press Stop to end test.");
            dashboard.displayPrintf(p++, "Decr srv : Dpad left");
            dashboard.displayPrintf(p++, "Incr srv : Dpad right");
            dashboard.displayPrintf(p, "Zero power : Dpad down");

            sleep(CYCLE_MS);
        }

        lifter.liftMotor.setPower(0.0);
        dashboard.displayText(1, "Done.");
    }
}