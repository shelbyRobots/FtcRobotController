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

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

@Autonomous(name = "Concept: Scan Servo", group = "Concept")
@Disabled
public class ConceptScanServo extends InitLinearOpMode
{
    private static final double INCREMENT   = 0.05;     // amount to slew servo each CYCLE_MS cycle
    private static final double MAX_POS     =  1.0;     // Maximum rotational position
    private static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    private Servo   gripper;
    private Servo   gPitch;
    private Servo   jFlicker;
    private double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    @Override
    public void runOpMode()
    {
        initCommon(this, false, false, false, false);

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        gripper = hardwareMap.get(Servo.class, "gripper");
        gPitch = hardwareMap.get(Servo.class, "gpitch");
        jFlicker = hardwareMap.get(Servo.class, "jflicker");

        // Wait for the start button
        dashboard.displayText(0, "Press Start to scan Servo.");
        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive())
        {
            gpad1.update();
            gpad2.update();

            boolean step_up    = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean step_down  = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);

            // slew the servo, according to the rampUp (direction) variable.
            if(step_down) position -= INCREMENT;
            if(step_up)   position += INCREMENT;

            position = Math.max(position, MIN_POS);
            position = Math.min(position, MAX_POS);

            // Display the current value
            dashboard.displayPrintf(1,"Servo Position %5.2f", position);

            // Set the servo to the new position and pause;
            gripper.setPosition(position);
            gPitch.setPosition(position);
            jFlicker.setPosition(position);
            idle();
        }
    }
}
