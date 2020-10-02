package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.TilerunnerMecanumBot;
import org.firstinspires.ftc.teamcode.util.Input_Shaper;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

@TeleOp(name = "Mecanum")
//@Disabled
public class MecanumTeleop extends InitLinearOpMode
{
    @SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, false, false, false);

        robot.setName(pmgr.getBotName());
        robot.init(this);
        //robot.setDriveDir(ShelbyBot.DriveDir.INTAKE);
        RobotLog.dd(TAG, "Start Hdg %.2f", robot.getAutonEndHdg());

        dashboard.displayPrintf(0, "%s is ready", robot.getName());

        while (!isStarted()) {
            gpad1.update();
            gpad2.update();
            gpad1.log(1);
            gpad2.log(2);
            idle();
        }

        RobotLog.dd(TAG, "Mecanum_Driver starting");

        while (opModeIsActive())
        {
            gpad1.update();
            gpad2.update();

            double raw_lr_x =  gpad1.value(ManagedGamepad.AnalogInput.R_STICK_X);
            double raw_fb_y = -gpad1.value(ManagedGamepad.AnalogInput.R_STICK_Y);
            double raw_turn =  gpad1.value(ManagedGamepad.AnalogInput.L_STICK_X);

            boolean rgt  = gpad1.pressed(ManagedGamepad.Button.D_RIGHT);
            boolean lft  = gpad1.pressed(ManagedGamepad.Button.D_LEFT);
            boolean fwd  = gpad1.pressed(ManagedGamepad.Button.D_UP);
            boolean bak  = gpad1.pressed(ManagedGamepad.Button.D_DOWN);
            boolean incr = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
            boolean decr = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);
            boolean tglV = gpad1.just_pressed(ManagedGamepad.Button.Y);
            //boolean step_driveType = gpad1.just_pressed(ManagedGamepad.Button.A);

            //if (step_driveType) fieldAlign = !fieldAlign;
            if (tglV) useSetVel = !useSetVel;

            double lr_x = ishaper.shape(raw_lr_x, 0.1);
            double fb_y = ishaper.shape(raw_fb_y, 0.1);
            double turn = ishaper.shape(raw_turn, 0.1);

            if      (incr) dSpd += dStp;
            else if (decr) dSpd -= dStp;
            dSpd = Range.clip(dSpd, 0.0, 1.0);

            if (lft || rgt || fwd || bak)
            {
                lr_x = lft ? -dSpd : rgt ?  dSpd : 0.0;
                fb_y = bak ? -dSpd : fwd ?  dSpd : 0.0;
                if((lft || rgt)  && (fwd || bak))
                {
                    lr_x /= spdScl;
                    fb_y /= spdScl;
                }
            }

            //Both trig and non-trig versions add turn to left and subtract from right
            double lf =  turn;
            double rf = -turn;
            double lr =  turn;
            double rr = -turn;
            double speed = 0.0;
            double direction = 0.0;

            if(trig)
            {
                //Start of trig based version - allows fieldAlign
                speed = spdScl * Math.sqrt(lr_x * lr_x + fb_y * fb_y);
                direction = Math.atan2(lr_x, fb_y) + rlrAng +
                        (fieldAlign ? Math.toRadians(90.0 - robot.getGyroFhdg()) : 0.0);

                double spdSin = speed * Math.sin(direction);
                double spdCos = speed * Math.cos(direction);

                lf += spdSin;
                rf += spdCos;
                lr += spdCos;
                rr += spdSin;
                //End of trig version
            }
            else
            {
                //Start of non-trig version (w/o fieldAlign):
                lf += fb_y + lr_x;
                rf += fb_y - lr_x;
                rr += fb_y + lr_x;
                lr += fb_y - lr_x;
                //End of non-trig version
            }
            double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                                  Math.max(Math.abs(lr), Math.abs(rr)));

            if (max > 1.0)
            {
                lf /= max;
                rf /= max;
                lr /= max;
                rr /= max;
            }

            if (useSetVel)
            {
                ((DcMotorEx) robot.lfMotor).setVelocity(lf * maxDPS, AngleUnit.DEGREES);
                ((DcMotorEx) robot.rfMotor).setVelocity(rf * maxDPS, AngleUnit.DEGREES);
                ((DcMotorEx) robot.lrMotor).setVelocity(lr * maxDPS, AngleUnit.DEGREES);
                ((DcMotorEx) robot.rrMotor).setVelocity(rr * maxDPS, AngleUnit.DEGREES);
            } else
            {
                robot.lfMotor.setPower(lf);
                robot.rfMotor.setPower(rf);
                robot.lrMotor.setPower(lr);
                robot.rrMotor.setPower(rr);
            }

            int l = 1;
            dashboard.displayPrintf(l++, "RAW LR_X %4.2f FB_Y %4.2f TRN %4.2f",
                    raw_lr_x, raw_fb_y, raw_turn);
            dashboard.displayPrintf(l++, "SHP LR_X %4.2f FB_Y %4.2f TRN %4.2f",
                    lr_x, fb_y, turn);
            dashboard.displayPrintf(l++, "SPD %4.2f DIR %4.2f DSPD: %3.1f FALGN %s USEVEL %s",
                    speed, direction, dSpd, fieldAlign, useSetVel);
            dashboard.displayPrintf(l++, "LFC %d RFC %d LRC %d RRC %d",
                    robot.lfMotor.getCurrentPosition(),
                    robot.rfMotor.getCurrentPosition(),
                    robot.lrMotor.getCurrentPosition(),
                    robot.rrMotor.getCurrentPosition());
            dashboard.displayPrintf(l, "OUT: lf %4.2f rf %4.2f lr %4.2f rr %4.2f",
                    lf, rf, lr, rr);
        }
    }

    double dSpd = 0.0;
    double dStp = 0.1;
    static final double diam   = 4.0;  //Inches
    static final double maxIPS = 30.0;
    static final double maxDPS = 360.0 * maxIPS/(diam*Math.PI);
    static final double rlrAng = Math.PI/4.0;
    static final double spdScl = Math.sqrt(2.0);
    static final boolean trig = true;
    Input_Shaper ishaper = new Input_Shaper();
    @SuppressWarnings("FieldCanBeLocal")
    private boolean fieldAlign = false;
    private boolean useSetVel = true;
    private TilerunnerMecanumBot robot = new TilerunnerMecanumBot();
    private static final String TAG = "SJH_MTD";
}