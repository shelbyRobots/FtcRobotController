package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
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

    private static final String TAG = "SJH_RMT";

    private static final double openGrab = 1000;
    private static final double closeGrab = 2000;

    @Override
    public void runOpMode() {
        initCommon(this, false, false, false, false);

        int p;

        String motorName = "lifter";
        String servoName = "grabber";
        DcMotor mot = null;
        Servo srv = null;

        try {
            mot = hardwareMap.dcMotor.get(motorName);
            srv = hardwareMap.servo.get(servoName);
        } catch (IllegalArgumentException e) {
            RobotLog.ee(TAG, "Problem finding motor " + motorName);
        }

        RobotLog.dd(TAG, "Found motor " + motorName + " and " + servoName);
        if (mot != null) {
            mot.setDirection(LEFT_DIR);
            mot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Wait for the start button
        dashboard.displayPrintf(0, "Press Start to run Motors.");

        int encPos = 0;
        double srvPos = 0;
        while (!isStarted()) {
            if (mot != null) {
                encPos = mot.getCurrentPosition();
            }
            if (srv != null){
                srvPos = srv.getPosition();
            }
            dashboard.displayPrintf(1, "CNT %d", encPos);
            dashboard.displayPrintf(2, "CNT %d", srvPos);

            sleep(10);
        }
        waitForStart();

        if (mot == null || srv == null) return;

        // Ramp motor speeds till stop pressed.
        while (opModeIsActive()) {
            p = 0;

            gpad1.update();
            boolean step_up = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean step_down = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean ungrab = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);
            boolean grab = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
            if (step_up && power < MAX_FWD) power += INCREMENT;
            else if (step_down && power > MAX_REV) power -= INCREMENT;
            else if (grab) srv.setPosition(closeGrab);
            else if (ungrab) srv.setPosition(openGrab);

            // Display the current value
            dashboard.displayPrintf(MAX_MOTORS + p++, "Motor Power %4.2f", power);
            encPos = mot.getCurrentPosition();
            dashboard.displayPrintf(1, "Mot CNT:%d PWR:%.2f MOD:%s",
                    encPos, power, mot.getMode());

            srvPos = srv.getPosition();
            dashboard.displayPrintf(1, "Mot CNT:%d PWR:%.2f MOD:%s",
                    srvPos);

            mot.setPower(power);

            dashboard.displayPrintf(MAX_MOTORS + p++, "Press Stop to end test.");
            dashboard.displayPrintf(MAX_MOTORS + p++, "Incr power : Dpad up");
            dashboard.displayPrintf(MAX_MOTORS + p++, "Decr power : Dpad down");
            dashboard.displayPrintf(MAX_MOTORS + p++, "Zero power : Dpad right");
            dashboard.displayPrintf(MAX_MOTORS + p, "Toggle mode: Right bumper");

            sleep(CYCLE_MS);
        }

        mot.setPower(0.0);
        dashboard.displayText(MAX_MOTORS + 1, "Done.");
    }
}