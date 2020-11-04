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
            success = true;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initShooter\n" + e.toString());
        }   

        if(shooter != null)
        {
            shooter.setDirection(DcMotor.Direction.FORWARD);
            shooter.setPower(0);
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(RUN_USING_ENCODER);
        }
        return success;
    }

    private double calculate(double distance, double hight, double hightOfShooter){
        //hight
        double h = hight - hightOfShooter;

        double g = 9.81 *3.28084 *12;
        //diameter of the wheels
        double dia = 4;
        //cicumpherence of the wheels
        double cir = dia * Math.PI;
        //starting y velocity
        double Vynot = Math.sqrt(2 * h * g);
        //starting x velocity
        double Vxnot = ((distance * g) / Vynot);
        double atanconst = Vynot/Vxnot;
        double theta = Math.atan(atanconst);
        double Vnot = Math.sqrt(Math.pow(Vxnot, 2) + Math.pow(Vynot, 2));
        double rpm = 2 * (Vnot / cir) * 60;

        double t = Vynot/g + Math.sqrt(Math.pow(Vynot, 2)/Math.pow(g, 2) - 2 * (-hightOfShooter)/g);
        double xDist = Vxnot * t;

        return rpm;
    }

    //Josh was here
    public void shoot()
    {

    }

    private final double SHOOTER_CPER = 6000; //quad encoder cnts/encoder rev
    private final double SHOOTER_INT_GEAR = 1; //Neverest 20
    private final double SHOOTER_EXT_GEAR = 1.0;
    private final double LIFTER_CPR = SHOOTER_CPER * SHOOTER_INT_GEAR * SHOOTER_EXT_GEAR;
    protected HardwareMap hwMap;
    public DcMotorEx shooter;
    private static final String TAG = "SJH_SHT";
}