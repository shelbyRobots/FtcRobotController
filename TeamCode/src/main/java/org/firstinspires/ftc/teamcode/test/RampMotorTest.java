package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Concept: Ramp Motor Speed", group = "Test")
@Disabled
public class RampMotorTest extends LinearOpMode {

    private static final double INCREMENT   = 0.05;     // amount to ramp motor each CYCLE_MS cycle
    private static final int    CYCLE_MS    =  500;     // period of each cycle
    private static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    private static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

    private int lastLeft  = 0;
    private int lastRight = 0;
    private double lastPower = 0.0;

    // Define class members
    private double  power   = 0;
    private boolean rampUp  = true;

    private static final DcMotor.Direction  LEFT_DIR = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction RIGHT_DIR = DcMotor.Direction.FORWARD;

    private static final String TAG = "SJH_RMT";

    @Override
    public void runOpMode() {

        DcMotor left_drive = null;
        DcMotor right_drive = null;
        DcMotorEx lex = null;
        DcMotorEx rex = null;

        try
        {
            left_drive = hardwareMap.dcMotor.get("leftdrive");
            left_drive.setDirection(LEFT_DIR);
            left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lex = (DcMotorEx)left_drive;
        }
        catch(Exception e)
        {
            RobotLog.ee(TAG, "Problem finding left motor");
        }

        try
        {
            right_drive = hardwareMap.dcMotor.get("rightdrive");
            right_drive.setDirection(RIGHT_DIR);
            right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rex = (DcMotorEx)right_drive;
        }
        catch(Exception e)
        {
            RobotLog.ee(TAG, "Problem finding right motor");
        }

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();

        PIDFCoefficients pcu = null;
        PIDFCoefficients pcn = null;
        int ltpt = 0;
        int rtpt = 0;
        double lspd;
        double rspd;

        if (lex != null)
        {
            pcu = lex.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            pcn = lex.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            ltpt = lex.getTargetPositionTolerance();
        }
        if (rex != null)
        {
            rtpt = rex.getTargetPositionTolerance();
        }

        while(!isStarted())
        {
            if(left_drive != null)
            {
                telemetry.addData("LCNT", "%d", left_drive.getCurrentPosition());
            }
            if(right_drive != null)
            {
                telemetry.addData("RCNT", "%d", right_drive.getCurrentPosition());
            }
            if (pcu != null && pcn != null)
            {
                telemetry.addData("LTP", "%d", ltpt);
                telemetry.addData("RTP", "%d", rtpt);
                telemetry.addData("PCU", "P:%4.2f I:%4.2f D:%4.2f F:%4.2f",
                        pcu.p, pcu.i, pcu.d, pcu.f);
                telemetry.addData("PCN", "P:%4.2f I:%4.2f D:%4.2f F:%4.2f",
                        pcn.p, pcn.i, pcn.d, pcn.f);
            }
            telemetry.update();
            sleep(10);
        }
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {

            // Ramp the motors, according to the rampUp variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += INCREMENT ;
                if (power >= MAX_FWD ) {
                    power = MAX_FWD;
                    rampUp = false;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                power -= INCREMENT ;
                if (power <= MAX_REV ) {
                    power = MAX_REV;
                    rampUp = true;  // Switch ramp direction
                }
            }

            // Display the current value
            telemetry.addData("Motor Power", "%5.2f", power);

            if(left_drive != null && lex !=null)
            {
                int curLeft = left_drive.getCurrentPosition();

                telemetry.addData("LCNT", "%d", curLeft);
                telemetry.addData("LLST", "%d", curLeft - lastLeft);
                RobotLog.dd(TAG, "Power %4.2f LCounts %d", lastPower, curLeft - lastLeft);
                lastLeft = curLeft;
            }
            if(lex != null)
            {
                lspd = lex.getVelocity(AngleUnit.DEGREES);
                telemetry.addData("LSPD", "%4.2f", lspd);
            }
            if(right_drive != null && rex != null)
            {
                int curRight = right_drive.getCurrentPosition();
                telemetry.addData("RCNT", "%d", curRight);
                telemetry.addData("RLST", "%d", curRight - lastRight);
                RobotLog.dd(TAG, "Power %4.2f RCounts %d", lastPower, curRight - lastRight);
                lastRight = curRight;
            }
            if(rex != null)
            {
                rspd = rex.getVelocity(AngleUnit.DEGREES);
                telemetry.addData("RSPD", "%4.2f", rspd);
            }

            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the motor to the new power and pause;
            if(left_drive !=null) left_drive.setPower(power);
            if(right_drive != null) right_drive.setPower(power);
            //if(lex !=null)   lex.setVelocity(1500, AngleUnit.DEGREES);
            //if(rex != null) rex.setVelocity(1500, AngleUnit.DEGREES);
            lastPower = power;
            sleep(CYCLE_MS);
            idle();
        }

        if(left_drive !=null) left_drive.setPower(0.0);
        if(right_drive != null) right_drive.setPower(0.0);
        // Turn off motor and signal done;
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
