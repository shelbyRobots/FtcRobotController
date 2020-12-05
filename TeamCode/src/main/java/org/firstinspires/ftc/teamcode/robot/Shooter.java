package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

import androidx.annotation.NonNull;

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
            shooter.setDirection(DcMotor.Direction.REVERSE);
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

    public void update()
    {
        if(shooter != null)
        {
            encPos = shooter.getCurrentPosition();
            curSpd = shooter.getVelocity();
        }
    }

    @NonNull
    public String toString()
    {
        return String.format(Locale.US, "shoot %5d %4.2f %4.2f %4.2f",
                encPos, curSpd, cps, dist);
    }

    public void stop(){
        shooter.setVelocity(0);
    }

    private double calcCps(double distance){
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
        cps = 2 * (v0 / cir) * SHOOTER_CPR;
        return cps;
    }

    public void shotSpeed(double distance)
    {
        dist = distance;
        if(shooter != null) shooter.setVelocity(calcCps(distance));
    }

    private final double SHOOTER_CPER = 28; //quad encoder cnts/encoder rev
    private final double SHOOTER_INT_GEAR = 1; //1:1 motor - approx 6000 rpm (no load)
    private final double SHOOTER_EXT_GEAR = 1.0;
    private final double SHOOTER_CPR = SHOOTER_CPER * SHOOTER_INT_GEAR * SHOOTER_EXT_GEAR;
    private int encPos = 0;
    private double curSpd = 0;
    protected HardwareMap hwMap;
    public DcMotorEx shooter = null;
    private static final String TAG = "SJH_SHT";
    private double dist = 0;
    private double cps = 0;

    public static void main(String[] args)
    {
        Shooter shtr = new Shooter(null);
        double dist = 70;
        double cps = shtr.calcCps(dist);
        System.out.println(String.format(Locale.US, "Dist %4.2f cps %4.2f rps %4.2f",
                dist, cps, cps/shtr.SHOOTER_CPR));
    }
}