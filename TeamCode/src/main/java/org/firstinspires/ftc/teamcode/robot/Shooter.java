package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

public class Shooter {

    public Shooter(HardwareMap map){
        super();
        this.hwMap = map;
    }

    public boolean init()
    {
        boolean success = false;
        try
        {
            shooter = hwMap.get(DcMotorEx.class, "shoot");
            shooter.setDirection(DcMotor.Direction.FORWARD);
            shooter.setPower(0);
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(RUN_USING_ENCODER);
            success = true;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initShooter\n" + e.toString());
        }   

        return success;
    }

    private double calcRps(double distance){
        double height = 35;
        double heightOfShooter = 12;

        double g = 9.81 *3.28084 *12;
        //diameter of the wheels
        double dia = 4;
        //cicumference of the wheels
        double cir = dia * Math.PI;

        double theta = Math.toRadians(35);
        double v0 = Math.sqrt((-g*Math.pow(distance,2))/
                (2*Math.pow(Math.cos(theta),2)*(height-distance*Math.tan(theta)-heightOfShooter)));
        return 2 * (v0 / cir);
    }

    public void shoot(double distance)
    {
        shooter.setVelocity(calcRps(distance));
    }

    private final double SHOOTER_CPER = 6000; //quad encoder cnts/encoder rev
    private final double SHOOTER_INT_GEAR = 1; //Neverest 20
    private final double SHOOTER_EXT_GEAR = 1.0;
    private final double LIFTER_CPR = SHOOTER_CPER * SHOOTER_INT_GEAR * SHOOTER_EXT_GEAR;
    protected HardwareMap hwMap;
    public DcMotorEx shooter;
    private static final String TAG = "SJH_SHT";
}