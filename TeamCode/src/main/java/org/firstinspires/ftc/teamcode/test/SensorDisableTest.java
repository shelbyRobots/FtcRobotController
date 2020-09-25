/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 *
 * This is an example LinearOpMode that tests the update speed of the gyro sensor
 * when a color sensor is enabled for callbacks and disabled.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "color" and the gyro with a name of "gyro".
 *
 * You can use the X button on gamepad1 to toggle the color's sensor's LED state.
 * Turning the LED off disables the color sensor's callback.
 *
 */
@TeleOp(name = "Gyro Heading with Color Sensor", group = "Sensor")
@Disabled
public class SensorDisableTest extends LinearOpMode {

    public void runOpMode() {

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // The time since the color or gyro sensor was last updated
        double dPrevColorMillis = 0;
        double dCurrColorMillis = 0;
        double dPrevGyroMillis = 0;
        double dCurrGyroMillis = 0;

        double totalGyroSampleTime = 0;
        int    numGyroSamples = 0;
        double totalColorSampleTime = 0;
        int    numColorSamples = 0;
        int noGyroCnt  = 0;
        int noColorCnt = 0;

        // The previous and current values of the color sensor (red only) and gyro heading
        int prevRed = 0;
        int currRed = 0;
        int iPrevAngleZ = 0;
        int iCurrAngleZ = 0;

        // The previous and current state of gamepad1's X button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // The state of the LED on the color sensor.
        boolean bLedOn = false;

        // get a reference to our ColorSensor object.
        ModernRoboticsI2cColorSensor colorSensor = (ModernRoboticsI2cColorSensor)this.hardwareMap.get("color");
        colorSensor.resetDeviceConfigurationForOpMode();

        // Set the LED to ON / true in the beginning
        colorSensor.enableLed(false);
        //TODO: SBH - Figure out how to register/deregister if timing shows its needed
        //colorSensor.getI2cController().deregisterForPortReadyCallback(colorSensor.getPort());

        ModernRoboticsI2cGyro gyro;   // Hardware Device Object

        // get a reference to a Modern Robotics GyroSensor object.
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        waitForStart();

        timer.reset();
        // while the op mode is active, loop and read the RGB data.
        while (opModeIsActive()) {

            bCurrState = gamepad1.x;
            dCurrColorMillis = timer.milliseconds();
            dCurrGyroMillis = dCurrColorMillis;
            currRed = colorSensor.red();
            iCurrAngleZ  = gyro.getIntegratedZValue();

            // check for button state transitions.
            if ((bCurrState) && (!bPrevState))  {

                // button is transitioning its state. So toggle the bLedOn
                bLedOn = !bLedOn;

                RobotLog.ii("SJH", "Turning color sensor %s", bLedOn ? "on" : "off");
                if(numGyroSamples > 0)
                {
                    RobotLog.ii("SJH", "AVERAGE gyro sample time %4.3f",
                            totalGyroSampleTime/numGyroSamples);
                }
                else
                {
                    RobotLog.ii("SJH", "No gyro samples for cycle");
                }
                if(numColorSamples > 0)
                {
                    RobotLog.ii("SJH", "AVERAGE color sample time %4.3f",
                            totalColorSampleTime/numColorSamples);
                }
                else
                {
                    RobotLog.ii("SJH", "No color samples for cycle");
                }
                numGyroSamples = 0;
                totalGyroSampleTime = 0;
                numColorSamples = 0;
                totalColorSampleTime = 0;

                // Enable or disable the color sensor and LED at the same time
                if (bLedOn) {
                    //TODO: SBH - Figure out how to register/deregister if timing shows its needed
                    //colorSensor.getI2cController().registerForI2cPortReadyCallback(colorSensor, colorSensor.getPort());
                    colorSensor.enableLed(true);
                }
                else {
                    colorSensor.enableLed(false);
                    sleep(50);
                    //TODO: SBH - Figure out how to register/deregister if timing shows its needed
                    //colorSensor.getI2cController().deregisterForPortReadyCallback(colorSensor.getPort());
                }

                timer.reset();
                prevRed = currRed;
                dPrevColorMillis = dCurrColorMillis;
                dPrevGyroMillis = dCurrGyroMillis;
                iPrevAngleZ = iCurrAngleZ;
            }

            // update previous state variable.
            bPrevState = bCurrState;

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED/Color Callback", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("R/G/B", (Double.toString(currRed) + "/" + Double.toString(colorSensor.green()) + "/" + Double.toString(colorSensor.blue())));

            // If the red value changes, show the milliseconds since last update
            // Write a line of telemetry in any case so the values don't jump around the screen
            if (prevRed == currRed) {
                telemetry.addData("No Color Update", 0);
                noColorCnt++;
                if(bLedOn && noColorCnt % 10 == 0)
                {
                    RobotLog.ii("SJH", "No Color Update for %d loops", noColorCnt);
                }
            }
            else {
                noColorCnt = 0;
                double sampleTime = dCurrColorMillis - dPrevColorMillis;
                totalColorSampleTime += sampleTime;
                numColorSamples++;
                telemetry.addData("Color Update Time", sampleTime);
                RobotLog.ii("SJH", "Color Update %d/%d/%d Time %4.3f",
                        currRed, colorSensor.green(), colorSensor.blue(), sampleTime);
                //RobotLog.ii("SJH", "Clear %d", colorSensor.alpha());
                dPrevColorMillis = dCurrColorMillis;
                prevRed = currRed;
            }

            // If the heading changes, show the milliseconds since last update
            telemetry.addData("1", "Int. Ang. %03d", iCurrAngleZ);
            if (iPrevAngleZ == iCurrAngleZ) {
                telemetry.addData("No Heading Update", 0);
                noGyroCnt++;
                if(noGyroCnt % 10 == 0)
                {
                    RobotLog.ii("SJH", "No Gyro Update for %d loops", noGyroCnt);
                }
            }
            else {
                noGyroCnt = 0;
                double sampleTime = dCurrGyroMillis - dPrevGyroMillis;
                totalGyroSampleTime += sampleTime;
                numGyroSamples++;
                telemetry.addData("Heading Update Time", sampleTime);
                RobotLog.ii("SJH", "Heading %d Update Time %4.3f", iCurrAngleZ, sampleTime);
                dPrevGyroMillis = dCurrGyroMillis;
                iPrevAngleZ = iCurrAngleZ;
            }

            telemetry.update();

            waitForTick(10);
        }
    }

    void waitForTick(long periodMs)
    {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
    private ElapsedTime period  = new ElapsedTime();
}
