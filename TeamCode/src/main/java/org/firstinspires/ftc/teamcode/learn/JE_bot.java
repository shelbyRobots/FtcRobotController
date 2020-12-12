package org.firstinspires.ftc.teamcode.learn;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class JE_bot {
    /* Public OpMode members. */
    public DcMotorEx lfMotor = null;
    public DcMotorEx lrMotor = null;
    public DcMotorEx rfMotor = null;
    public DcMotorEx rrMotor = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private final ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public JE_bot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        lfMotor = hwMap.get(DcMotorEx.class, "FL");
        rfMotor = hwMap.get(DcMotorEx.class, "FR");
        lrMotor = hwMap.get(DcMotorEx.class, "BL");
        rrMotor = hwMap.get(DcMotorEx.class, "BR");
        rfMotor.setDirection(DcMotor.Direction.FORWARD);
        rrMotor.setDirection(DcMotor.Direction.FORWARD);
        lfMotor.setDirection(DcMotor.Direction.REVERSE);
        lrMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        lrMotor.setPower(0);
        lfMotor.setPower(0);
        rrMotor.setPower(0);
        rfMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        lrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
