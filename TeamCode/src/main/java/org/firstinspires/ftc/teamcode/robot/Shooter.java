package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

import androidx.annotation.NonNull;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

public class Shooter
{
    public Shooter(HardwareMap map)
    {
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
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(RUN_USING_ENCODER);
            success = true;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initShooter\n" + e.toString());
        }

        shtPid = new PIDFCoefficients(80.0, 0.0, 0.0,14.9);
        setPIDF(shtPid);

        RobotLog.dd(TAG, "RUN_USING_ENC shooter PID. %s", shtPid);

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

    public void stop()
    {
        cps = 0.0;
        if(shooter != null) shooter.setVelocity(cps);
    }

    private static final double g = -9.81 *3.28084 *12;
    private static final double height = 35;
    private static final double heightOfShooter = 10;
    private static final double dia = 4; //diameter of the wheels
    private static final double cir = dia * Math.PI; //cicumference of the wheels
    private static final double theta = Math.toRadians(35);
    private double calcCps(double distance)
    {
        v0 = Math.sqrt((g*Math.pow(distance,2))/
            (2*Math.pow(Math.cos(theta),2)*(height-distance*Math.tan(theta)-heightOfShooter)));
        return 2 * (v0 / cir) * SHOOTER_CPR;
    }

    public void shotSpeed(double distance)
    {
        dist = distance;
        cps = calcCps(dist);
        if(shooter != null) shooter.setVelocity(cps);
    }

    public void shootCps(double cps)
    {
        this.cps = cps;
        if(shooter != null) shooter.setVelocity(cps);
    }

    public double getV0() {return v0;}
    public int getEncPos() {return encPos;}
    public double getCurSpd() {return curSpd;}
    public double getCmdSpd() {return cps;}
    public double getDist() {return dist;}

    public PIDFCoefficients getPidf() {return shtPid;}
    public void setPIDF(PIDFCoefficients pidf)
    {
        shtPid = pidf;
        shooter.setPIDFCoefficients(RUN_USING_ENCODER, shtPid);
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
    private double v0 = 0.0;

    private PIDFCoefficients shtPid = RobotConstants.SH_PID;

    public static void main(String[] args)
    {
        Shooter shtr = new Shooter(null);
        for(double dist = 10; dist < 400; dist+=6)
        {
            double cps = shtr.calcCps(dist);
            System.out.println(String.format(Locale.US, "Dist %4.2f cps %.2f rps %.2f v0: %.2f",
                dist, cps, cps/shtr.SHOOTER_CPR, shtr.getV0()));
        }

    }
}