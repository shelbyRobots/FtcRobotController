package org.firstinspires.ftc.teamcode.learn;
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="JE_TEST", group="Pushbot")
@Disabled
public class JE_Test extends  LinearOpMode{
    /* Declare OpMode members. */
    JE_bot robot   = new JE_bot();   // Use a Pushbot's hardware
    private final ElapsedTime     runtime = new ElapsedTime();

    static final double     WHEEL_BASE              = 15.96;
    static final double     COUNTS_PER_MOTOR_REV    = 537.6;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 96.0 / 25.4;     // For figuring circumference
    static final double     COUNTS_PER_INCH         =
        (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Start",  "Starting at %7d %7d %7d %7d",
                robot.lfMotor.getCurrentPosition(),
                robot.lrMotor.getCurrentPosition(),
                robot.rfMotor.getCurrentPosition(),
                robot.rrMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        double turnDist = WHEEL_BASE * Math.PI * (180.0/360.0);
        encoderDrive(DRIVE_SPEED,  24,  24, 8.0);
        encoderDrive(TURN_SPEED,   turnDist, -turnDist, 8.0);
        encoderDrive(DRIVE_SPEED, -24, -24, 8.0);
        encoderDrive(TURN_SPEED,   -turnDist, turnDist, 8.0);
    }

    private int pth = 0;
    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int tgtLfPos;
        int tgtLrPos;
        int tgtRfPos;
        int tgtRrPos;
        int curLfPos;
        int curLrPos;
        int curRfPos;
        int curRrPos;

        // Ensure that the opmode is still active
        if (opModeIsActive())
        {
            curLfPos = robot.lfMotor.getCurrentPosition();
            curLrPos = robot.lrMotor.getCurrentPosition();
            curRfPos = robot.rfMotor.getCurrentPosition();
            curRrPos = robot.rrMotor.getCurrentPosition();
            // Determine new target position, and pass to motor controller
            tgtLfPos = curLfPos + (int)(leftInches  * COUNTS_PER_INCH);
            tgtLrPos = curLrPos + (int)(leftInches  * COUNTS_PER_INCH);
            tgtRfPos = curRfPos + (int)(rightInches * COUNTS_PER_INCH);
            tgtRrPos = curRrPos + (int)(rightInches * COUNTS_PER_INCH);

            robot.lfMotor.setTargetPosition(tgtLfPos);
            robot.lrMotor.setTargetPosition(tgtLrPos);
            robot.rfMotor.setTargetPosition(tgtRfPos);
            robot.rrMotor.setTargetPosition(tgtRrPos);


            // Turn On RUN_TO_POSITION
            robot.lfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.lfMotor.setPower(Math.abs(speed));
            robot.lrMotor.setPower(Math.abs(speed));
            robot.rfMotor.setPower(Math.abs(speed));
            robot.rrMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.lfMotor.isBusy() &&
                     robot.lrMotor.isBusy() &&
                     robot.rfMotor.isBusy() &&
                     robot.rrMotor.isBusy()))
            {
                curLfPos = robot.lfMotor.getCurrentPosition();
                curLrPos = robot.lrMotor.getCurrentPosition();
                curRfPos = robot.rfMotor.getCurrentPosition();
                curRrPos = robot.rrMotor.getCurrentPosition();

                // Display it for the driver.
                telemetry.addData("seg" + pth++,  "Driving %.2f %.2f %.2f %.2f",
                    speed, leftInches, rightInches, timeoutS);
                telemetry.addData("Tgt",  "%7d %7d %7d %7d",
                    tgtLfPos,tgtLrPos,tgtRfPos,tgtRrPos);
                telemetry.addData("Cur",  "%7d %7d  %7d %7d",
                        curLfPos,
                        curLrPos,
                        curRfPos,
                        curRrPos);
                telemetry.update();
            }

            // Stop all motion;
            robot.lfMotor.setPower(0);
            robot.lrMotor.setPower(0);
            robot.rfMotor.setPower(0);
            robot.rrMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
}
