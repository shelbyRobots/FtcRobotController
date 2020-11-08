package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Lifter;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

@TeleOp(name = "Testlifter", group = "Test")

public class Testlifter extends InitLinearOpMode {
    private static final double INCREMENT = 0.02;     // amount to step motor each CYCLE_MS cycle
    private static final int CYCLE_MS = 50;       // period of each cycle
    private static final double MAX_FWD = 1.0;     // Maximum FWD power applied to motor
    private static final double MAX_REV = -1.0;     // Maximum REV power applied to motor

    // Define class members
    private double power = 0;

    private static final DcMotor.Direction LEFT_DIR = DcMotor.Direction.FORWARD;

    private static final int MAX_MOTORS = 4;
    private Lifter lifter = null;

    private static final String TAG = "SJH_RMT";

    private static final double openGrab = 1000;
    private static final double closeGrab = 2000;

    @Override
    public void runOpMode() {
        initCommon(this, false, false, false, false);

        lifter = new Lifter(hardwareMap);
        lifter.init();
        int p;


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

            gpad1.update();
            double lftPwr = -gpad1.value(ManagedGamepad.AnalogInput.L_STICK_Y);
            boolean ungrip = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);
            boolean grip = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
            boolean stow = gpad1.just_pressed(ManagedGamepad.Button.A);
            boolean grab = gpad1.just_pressed(ManagedGamepad.Button.B);
            boolean hold = gpad1.just_pressed(ManagedGamepad.Button.X);
            boolean drop = gpad1.just_pressed(ManagedGamepad.Button.Y);

            if (grip) lifter.clampServo.setPosition(closeGrab);
            else if (ungrip) lifter.clampServo.setPosition(openGrab);

            double safety = 0.5;
            if(lftPwr > safety) lftPwr = safety;
            if(lftPwr < -safety) lftPwr = -safety;

            if(stow)lifter.setLiftPos(Lifter.LiftPos.STOW);
            else if(grab)lifter.setLiftPos(Lifter.LiftPos.GRAB);
            else if(hold)lifter.setLiftPos(Lifter.LiftPos.HOLD);
            else if(drop)lifter.setLiftPos(Lifter.LiftPos.DROP);
            else if(lftPwr != 0.0) lifter.setLiftSpd( lftPwr);

            // Display the current value
            dashboard.displayPrintf(MAX_MOTORS + p++, "Motor Power %4.2f", power);
            encPos = lifter.liftMotor.getCurrentPosition();
            dashboard.displayPrintf(1, "Mot CNT:%d PWR:%.2f MOD:%s",
                    encPos, power, lifter.liftMotor.getMode());

            srvPos = lifter.clampServo.getPosition();
            dashboard.displayPrintf(1, "Mot CNT:%d PWR:%.2f MOD:%s",
                    srvPos);

            lifter.liftMotor.setPower(power);

            dashboard.displayPrintf(MAX_MOTORS + p++, "Press Stop to end test.");
            dashboard.displayPrintf(MAX_MOTORS + p++, "Incr power : Dpad up");
            dashboard.displayPrintf(MAX_MOTORS + p++, "Decr power : Dpad down");
            dashboard.displayPrintf(MAX_MOTORS + p++, "Zero power : Dpad right");
            dashboard.displayPrintf(MAX_MOTORS + p, "Toggle mode: Right bumper");

            sleep(CYCLE_MS);
        }

        lifter.liftMotor.setPower(0.0);
        dashboard.displayText(MAX_MOTORS + 1, "Done.");
    }
}