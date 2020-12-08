package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.Units;

import java.util.List;

public class TilerunnerMecanumBot extends TilerunnerGtoBot
{
    public DcMotorEx lfMotor = null;
    public DcMotorEx lrMotor = null;
    public DcMotorEx rfMotor = null;
    public DcMotorEx rrMotor = null;

    public Lifter liftyBoi = null;
    public Shooter burr = null;
    public Intake intake = null;
    public Loader loader = null;

    private final int[] cnts = {0,0,0,0};
    private final double[] vels = {0,0,0,0};
    private double hdg = 0;

    private static final String TAG = "SJH_MEC";
    public TilerunnerMecanumBot()
    {
        super();

        name= "MEC2";

        DRIVE_GEARS = new double[]{RobotConstants.DT_MOTOR.getGear(), RobotConstants.DT_EXT_GEAR_RATIO};

        WHEEL_DIAMETER_INCHES = RobotConstants.DT_WHEEL_DIAM;

        CAMERA_X_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;
        CAMERA_Y_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;
        CAMERA_Z_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;

        gyroInverted = false;
    }

    @Override
    protected void initDriveMotors()
    {
        RobotLog.dd(TAG, TAG + "Initializing drive motors");
        try
        {
            lfMotor = hwMap.get(DcMotorEx.class, "FL");
            rfMotor = hwMap.get(DcMotorEx.class, "FR");
            lrMotor = hwMap.get(DcMotorEx.class, "BL");
            rrMotor = hwMap.get(DcMotorEx.class, "BR");
            motors.put("FL", lfMotor);
            motors.put("FR", rfMotor);
            motors.put("BL", lrMotor);
            motors.put("BR", rrMotor);

            leftMotors.add(numLmotors++, lfMotor);
            leftMotors.add(numLmotors++, lrMotor);
            rightMotors.add(numRmotors++, rfMotor);
            rightMotors.add(numRmotors++, rrMotor);

            lfMotor.setDirection(RobotConstants.DT_LDIR);
            rfMotor.setDirection(RobotConstants.DT_RDIR);
            lrMotor.setDirection(RobotConstants.DT_LDIR);
            rrMotor.setDirection(RobotConstants.DT_RDIR);

            capMap.put("drivetrain", true);

            List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
            for (LynxModule module : allHubs)
            {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }
        catch (Exception e)
        {
            RobotLog.ee("SJH", "ERROR get hardware map\n" + e.toString());
        }

        int mnum = 0;
        for(DcMotor mot : motors.values())
        {
            if(mot != null)
            {
                mot.setPower(0);
                mot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                if (mot instanceof DcMotorEx)
                {
                    DcMotorEx lex = (DcMotorEx) mot;
                    PIDFCoefficients pid;
                    pid = lex.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
                    RobotLog.dd(TAG, "RUN_TO_POS Motor %d PIDs. P:%.2f I:%.2f D:%.2f",
                            mnum, pid.p, pid.i, pid.d);
                    pid = lex.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                    RobotLog.dd(TAG, "RUN_USING_ENC Motor %d PIDs. P:%.2f I:%.2f D:%.2f",
                            mnum, pid.p, pid.i, pid.d);
                    //pid = lex.getPIDCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //RobotLog.dd(TAG, "RUN_WITHOUT_ENC Motor %d PIDs. P:%.2f I:%.2f D:%.2f",
                    //        mnum, pid.p, pid.i, pid.d);
                }
            }
            mnum++;
        }
    }

    @Override
    protected void initArm()
    {
    }

    @Override
    protected void initCollectorLifter()
    {
        //Lifter is wobblyBoi Arm - has rotational lift and gripper
        liftyBoi = new Lifter(hwMap);
        liftyBoi.init();

        //Collector is combo of intake and loader
        initIntakes();
    }

    @Override
    protected void initShooters()
    {
        burr = new Shooter(hwMap);
        burr.init();
    }

    @Override
    protected void initPushers()
    {
    }

    protected void initIntakes()
    {
        intake = new Intake(hwMap);
        intake.init();

        loader = new Loader(hwMap);
        loader.init();
    }

    public void update()
    {
        int c = 0;
        for (DcMotorEx m : motors.values())
        {
            cnts[c] = m.getCurrentPosition();
            vels[c++] = m.getVelocity();
        }

        hdg = getGyroFhdg();

        if(liftyBoi != null) { liftyBoi.update(); }
        if(burr != null)     { burr.update(); }
        if(intake != null)   { intake.update(); }
        if(loader != null)   { loader.update(); }
    }

    public int[] getCnts()    { return cnts; }
    public double[] getVels()
    {
        return vels;
    }
    public double getHdg()    { return hdg;  }

}
