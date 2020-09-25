/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.util.CommonUtil;

@SuppressWarnings("FieldCanBeLocal")
@Autonomous(name="Auto Drive By Encoder", group="Test")
@Disabled
public class AutoDriveByEncoder_Linear extends InitLinearOpMode
{
    private CommonUtil com = CommonUtil.getInstance();
    private ElapsedTime runtime = new ElapsedTime();
    private ShelbyBot robot = new ShelbyBot();

    private static final double COUNTS_PER_MOTOR_REV = 28;
    private static final double DRIVE_GEARS[] = {19.2, 1};
    private static double getTotalGearRatio()
    {
        double gr = 1.0;
        for(double g : DRIVE_GEARS) gr *= g;
        return gr;
    }
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double TUNE = 1.00;
    private static final double CPI = (COUNTS_PER_MOTOR_REV *
                               getTotalGearRatio())/
                              (WHEEL_DIAMETER_INCHES * TUNE * Math.PI);

    private static final double DRIVE_SPEED = 0.6;
    private static final double TURN_SPEED = 0.5;

    private ElapsedTime datalogtimer = new ElapsedTime();
    private boolean logData = true;

    private boolean colorOn = false;
    private int r, g, b;

    private final double DD = 24.0;
    private final double TD = 14.9/2.0 * Math.PI;

    private static int frame = 0;

    private ShelbyBot.DriveDir startDir = ShelbyBot.DriveDir.INTAKE;

    private final static String TAG = "SJH_ADBE";

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, false, false, false);
        super.runOpMode();
        if (logData)
        {
            dl = com.getDataLogger();
            if(dl != null)
            {
                dl.addField("Gyro");
                dl.addField("LENC");
                dl.addField("RENC");
                dl.addField("LPWR");
                dl.addField("RPWR");
                dl.addField("RED");
                dl.addField("GRN");
                dl.addField("BLU");
                dl.newLine();
            }
        }

        RobotLog.dd(TAG, "CPI %4.2f", CPI);

        robot.init(this);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.setDriveDir(startDir);
        //Currently setup for tilerunner with 1 gear after motor gearbox
        robot.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        ElapsedTime tmpTimer = new ElapsedTime();
        boolean first = true;

        while (!isStarted() && tmpTimer.seconds() < 60)
        {
            if(robot.gyro != null)
                telemetry.addData(":", "FHDG = %6.3f", robot.getGyroFhdg());
            telemetry.addData("<", "LENC = %5d RENC = %5d",
                    robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());
            if(robot.gyro != null)
            {
                telemetry.addData(")", "ZInt = %d", robot.gyro.getIntegratedZValue());
                telemetry.addData(")", "GHdg = %d", robot.gyro.getHeading());
            }
            telemetry.addData("0", "DDIR = %s", robot.getDriveDir());
            telemetry.addData("0", "IDIR = %s", robot.calibrationDriveDir);
            telemetry.update();

            if(tmpTimer.seconds() > 30 && first)
            {
                robot.invertDriveDir();
                first = false;
            }
            sleep(50);
        }

        waitForStart();
        if(robot.gyro != null) robot.gyro.resetZAxisIntegrator();

        ShelbyBot.DriveDir dir = startDir;

        doTestCycle(dir, DRIVE_SPEED, TURN_SPEED);
//        sleep(1000);
//        doTestCycle(robot.invertDriveDir(), DRIVE_SPEED, TURN_SPEED);
//        sleep(1000);
//        doTestCycle(robot.invertDriveDir(), DRIVE_SPEED/2, TURN_SPEED/2);
//        sleep(1000);
//        doTestCycle(robot.invertDriveDir(), DRIVE_SPEED/2, TURN_SPEED/2);
//        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    private void doTestCycle(ShelbyBot.DriveDir ddir, double spd, double trnspd)
    {
        //Temporary comment out robot.setDriveDir(ddir);
        if(logData && dl != null)
        {
            dl.addField(ddir.toString());
            dl.newLine();
        }

        robot.turnColorOn();
        telemetry.addData("SEG", "INTAKE FWD " + DD);
        telemetry.update();
        encoderDrive(spd,  DD,  DD, 30.0);
        robot.turnColorOff();
        r = 0;
        g = 0;
        b = 0;
        telemetry.addData("SEG", "INTAKE LFT " + TD);
        telemetry.update();
        encoderDrive(trnspd, TD, -TD, 30.0);
        robot.turnColorOn();
        telemetry.addData("SEG", "INTAKE BCK " + DD);
        telemetry.update();
        encoderDrive(spd, -DD, -DD, 30.0);
        //turnColorOff();
        //telemetry.addData("SEG", "INTAKE RGT " + TD);
        //telemetry.update();
        //encoderDrive(trnspd, -TD, TD, 30.0);
    }

    private void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS)
    {
        int newLeftTarget;
        int newRightTarget;

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (opModeIsActive())
        {
            int lmoveCnt = (int)(leftInches  * CPI);
            int rmoveCnt = (int)(rightInches  * CPI);

            if(logData && dl != null)
            {
                dl.addField("encoderDrive");
                dl.addField("", leftInches);
                dl.addField("", rightInches);
                dl.addField("", speed);
                dl.addField("", lmoveCnt);
                dl.addField("", rmoveCnt);
                dl.newLine();
            }

            newLeftTarget  = robot.leftMotor.getCurrentPosition()  + lmoveCnt;
            newRightTarget = robot.rightMotor.getCurrentPosition() + rmoveCnt;
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            RobotLog.ii("SJH", "TGT : %7d %7d", newLeftTarget, newRightTarget);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                if(frame%5 == 0)
                {
                    logData();
                    telemetry.addData("TGT", "TGT %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("POS", "POS %7d :%7d",
                            robot.leftMotor.getCurrentPosition(),
                            robot.rightMotor.getCurrentPosition());
                    if(robot.gyro != null)
                        telemetry.addData("ROT", "%6.3f", robot.getGyroFhdg());
                    if(robot.colorSensor != null)
                        telemetry.addData("RGB", "RGB %3d %3d %3d", r, g, b);
                    RobotLog.dd("SJH", "Pos : %7d %7d",
                            robot.leftMotor.getCurrentPosition(),
                            robot.rightMotor.getCurrentPosition());
                    telemetry.update();
                }
                frame++;
            }

            logData();

            RobotLog.ii("SJH", "Done: %7d %7d",
                    robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            if(logData && dl != null)
            {
                dl.addField("DONE");
                dl.newLine();
            }

            ElapsedTime et = new ElapsedTime();
            while(et.seconds() < 0.5)
            {
                logData();
                sleep(10);
            }

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void logData()
    {
        double dlTimeout = 0.002;
        if (datalogtimer.seconds() <dlTimeout) return;

        datalogtimer.reset();

        if(logData && dl != null)
        {
            if(colorOn  && robot.colorSensor != null)
            {
                r = robot.colorSensor.red();
                g = robot.colorSensor.green();
                b = robot.colorSensor.blue();
            }
            if(robot.gyro != null) dl.addField(robot.getGyroFhdg());
            else                   dl.addField("");
            dl.addField(robot.leftMotor.getCurrentPosition());
            dl.addField(robot.rightMotor.getCurrentPosition());
            dl.addField("",robot.leftMotor.getPower());
            dl.addField("",robot.rightMotor.getPower());
            if(robot.colorSensor != null)
            {
                dl.addField(r);
                dl.addField(g);
                dl.addField(b);
            }
            dl.newLine();
        }
    }
}
