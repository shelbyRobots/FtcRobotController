
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Units;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Shelbybot
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 */
@SuppressWarnings({"WeakerAccess", "FieldCanBeLocal", "unused"})
public class ShelbyBot
{
    protected LinearOpMode op = null;

    public enum OpModeType {TELE, AUTO, UNKNOWN}

    public static OpModeType curOpModeType = OpModeType.UNKNOWN;

    public static RobotConstants rbc = new RobotConstants(RobotConstants.Chassis.MEC2);

    /* Public OpMode members. */
    public DcMotorEx  leftMotor   = null;
    public DcMotorEx  rightMotor  = null;
    public DcMotorEx  elevMotor   = null;
    public DcMotorEx  sweepMotor  = null;
    public DcMotorEx  shotmotor1  = null;
    public DcMotorEx  shotmotor2  = null;
    public Servo    lpusher     = null;
    public Servo    rpusher     = null;

    public List<DcMotorEx> leftMotors  = new ArrayList<>(2);
    public List<DcMotorEx> rightMotors = new ArrayList<>(2);

    public int numLmotors = 0;
    public int numRmotors = 0;

    public ModernRoboticsI2cGyro        gyro        = null;
    //public ModernRoboticsI2cColorSensor colorSensor = null;
    public ColorSensor colorSensor = null;
    public DeviceInterfaceModule        dim         = null;

    protected CommonUtil cmu = CommonUtil.getInstance();

    public BNO055IMU imu = null;

    public boolean gyroInverted = true;

    //Distance from ctr of rear wheel to tail
    public float REAR_OFFSET;
    public float FRNT_OFFSET;
    protected static float CAMERA_X_IN_BOT;
    protected static float CAMERA_Y_IN_BOT;
    protected static float CAMERA_Z_IN_BOT;

    private final int colorPort = 0;
    private final DriveDir defDriveDir = DriveDir.PUSHER;
    private DriveDir ddir = defDriveDir;
    private boolean invertDrive = false;
    public DriveDir calibrationDriveDir = DriveDir.UNKNOWN;
    protected HardwareMap hwMap = null;

    protected String name = "ShelbyBot";

    boolean colorEnabled = false;

    private Field.Alliance alliance = Field.Alliance.RED;

    private int initHdg = 0;
    public static double autonEndHdg = 0.0;
    public double getAutonEndHdg() {return  autonEndHdg;}
    public void setAutonEndHdg(double hdg) {autonEndHdg = hdg;}
    public static Point2d autonEndPos = new Point2d("AEND", 0.0, 0.0);
    public Point2d getAutonEndPos() {return autonEndPos;}
    public void setAutonEndPos(Point2d endPos) {autonEndPos = endPos;}
    public Field.Alliance getAlliance() {return alliance;}
    public void setAlliance(Field.Alliance alnc) {alliance = alnc;}

    public float BOT_LENGTH;

    protected double COUNTS_PER_MOTOR_REV;
    protected double[] DRIVE_GEARS;

    public double WHEEL_DIAMETER_INCHES;
    protected double TUNE;
    public double CPI;

    public static DcMotor.Direction  LEFT_DIR = DcMotor.Direction.FORWARD;
    public static DcMotor.Direction RIGHT_DIR = DcMotor.Direction.REVERSE;

    public Map<String, DcMotorEx> motors = new HashMap<>();

    public boolean gyroReady = false;
    Map<String, Boolean> capMap = new HashMap<>();

    /* local OpMode members. */
    private final ElapsedTime period  = new ElapsedTime();

    private static final String TAG = "SJH_BOT";

    /* Constructor */
    public ShelbyBot()
    {
        //Neverest classic 20,40,60, and orbital 20 have 7 rising edges of Channel A per revolution
        //with a quadrature encoder (4 total edges - A rise, B rise, A fall, B fall) for a total
        //of 28 counts per pre-gear box motor shaft revolution.
        COUNTS_PER_MOTOR_REV = 28;
        DRIVE_GEARS = new double[]{40.0, 1.0/2.0};

        WHEEL_DIAMETER_INCHES = 4.1875;
        TUNE = 1.00;

        BOT_LENGTH = 18.0f;

        REAR_OFFSET = 9.0f;
        FRNT_OFFSET = BOT_LENGTH - REAR_OFFSET;

        CAMERA_X_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;
        CAMERA_Y_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;
        CAMERA_Z_IN_BOT = 0.0f * (float)Units.MM_PER_INCH;

        capMap.put("drivetrain", false);
        capMap.put("shooter",    false);
        capMap.put("collector",  false);
        capMap.put("pusher",     false);
        capMap.put("sensor",     false);
        capMap.put("arm",        false);
        capMap.put("holder",     false);
        capMap.put("lifter",     false);
    }

    /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode op, boolean initDirSensor)
    {
        RobotLog.dd(TAG, "ShelbyBot init");
        computeCPI();

        initOp(op);
        initDriveMotors();
        initCollectorLifter();
        initPushers();
        initShooters();
        initSensors();
        initCapabilities();
    }

    public void init(LinearOpMode op)
    {
        init(op, true);
    }

    public void init(LinearOpMode op, String name)
    {
        this.name = name;
        init(op);
    }

    public void init(LinearOpMode op, RobotConstants.Chassis chassis)
    {
        init(op, chassis.name());
        rbc = new RobotConstants(chassis);
    }

    protected void initOp(LinearOpMode op)
    {
        RobotLog.dd(TAG, "ShelbyBot initOp");
        this.op = op;
        this.hwMap = op.hardwareMap;
    }

    protected void initDriveMotors()
    {
        RobotLog.dd(TAG, "ShelbyBot initDriveMotors");
        // FORWARD for CCW drive shaft rotation if using AndyMark motors
        // REVERSE for  CW drive shaft rotation if using AndyMark motors
        try  //Drivetrain
        {
            leftMotor  = hwMap.get(DcMotorEx.class,"leftdrive");
            rightMotor = hwMap.get(DcMotorEx.class,"rightdrive");

            leftMotors.add(numLmotors++, leftMotor);
            rightMotors.add(numRmotors++, rightMotor);

             leftMotor.setDirection(LEFT_DIR);
            rightMotor.setDirection(RIGHT_DIR);
            motors.put("FL", leftMotor);
            motors.put("FR", rightMotor);

            DcMotorEx lm0 = leftMotors.get(0);
            PIDFCoefficients pid;
            pid = lm0.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            RobotLog.dd(TAG, "RUN_TO_POS Motor PIDs. P:%.2f I:%.2f D:%.2f F:%.2f",
                    pid.p, pid.i, pid.d, pid.f);
            pid = lm0.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            RobotLog.dd(TAG, "RUN_USING_ENC Motor PIDs. P:%.2f I:%.2f D:%.2f F:%.2f",
                    pid.p, pid.i, pid.d, pid.f);

            for (DcMotorEx m : leftMotors)
            {
                PIDFCoefficients lpid;
                lpid = m.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
                lpid.p = 6.0;
                m.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, lpid);
            }

            for (DcMotorEx m : rightMotors)
            {
                PIDFCoefficients rpid;
                rpid = m.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
                rpid.p = 6.0;
                m.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, rpid);
            }

            capMap.put("drivetrain", true);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initDriveTrain\n" + e.toString());
        }

        for(DcMotor mot : motors.values())
        {
            if(mot != null)
            {
                mot.setPower(0);
                mot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    protected void initCollectorLifter()
    {
        RobotLog.dd(TAG, "ShelbyBot collector/lifter");
        try  //Collector
        {
            elevMotor = hwMap.get(DcMotorEx.class, "elevmotor");
            sweepMotor = hwMap.get(DcMotorEx.class, "sweepmotor");
            capMap.put("collector", true);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initCollector\n" + e.toString());
        }

        if(elevMotor  != null)  elevMotor.setDirection(DcMotor.Direction.REVERSE);
        if(sweepMotor != null) sweepMotor.setDirection(DcMotor.Direction.FORWARD);
        if(elevMotor  != null)  elevMotor.setPower(0);
        if(sweepMotor != null) sweepMotor.setPower(0);
        if(elevMotor  != null)  elevMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(sweepMotor != null) sweepMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void initShooters()
    {
        RobotLog.dd(TAG, "ShelbyBot init shooters");
        try  //Shooters
        {
            shotmotor1 = hwMap.get(DcMotorEx.class, "leftshooter");
            shotmotor2 = hwMap.get(DcMotorEx.class, "rightshooter");
            capMap.put("shooter", true);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map\n" + e.toString());
        }

        if(shotmotor1 != null) shotmotor1.setDirection(DcMotor.Direction.FORWARD);
        if(shotmotor2 != null) shotmotor2.setDirection(DcMotor.Direction.REVERSE);
        if(shotmotor1 != null) shotmotor1.setPower(0);
        if(shotmotor2 != null) shotmotor2.setPower(0);
        if(shotmotor1 != null) shotmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(shotmotor2 != null) shotmotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(shotmotor1 != null) shotmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(shotmotor2 != null) shotmotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    protected void initPushers()
    {
        RobotLog.dd(TAG, "ShelbyBot pushers");
        try  //Pushers
        {
            lpusher = hwMap.servo.get("lpusher");
            rpusher = hwMap.servo.get("rpusher");
            capMap.put("pusher", true);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map\n" + e.toString());
        }
    }

    protected void initSensors(boolean initDirSensor)
    {
        RobotLog.dd(TAG, "ShelbyBot sensors");
        try  //I2C and DAIO
        {
            dim = hwMap.deviceInterfaceModule.get("dim");

            gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");
            //colorSensor = (ModernRoboticsI2cColorSensor) hwMap.colorSensor.get("color");
            colorSensor = hwMap.colorSensor.get("color");

            capMap.put("sensor", true);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map\n" + e.toString());
        }

        if(colorSensor != null)
        {
            RobotLog.ii(TAG, "COLOR_SENSOR");
            RobotLog.ii(TAG, "ConnectionInfo %s", colorSensor.getConnectionInfo());
            RobotLog.ii(TAG, "I2cAddr %s", Integer.toHexString(colorSensor.getI2cAddress().get8Bit()));
            RobotLog.ii(TAG, "I2cAddr %s", Integer.toHexString(colorSensor.getI2cAddress().get7Bit()));

            colorSensor.enableLed(false);
            colorSensor.enableLed(true);

            turnColorOff();
        }

        if(gyro != null)
        {
            RobotLog.ii(TAG, "GYRO_SENSOR");
            RobotLog.ii(TAG, "ConnectionInfo %s", gyro.getConnectionInfo());
            RobotLog.ii(TAG, "I2cAddr %s", Integer.toHexString(gyro.getI2cAddress().get8Bit()));
            RobotLog.ii(TAG, "I2cAddr %s", Integer.toHexString(gyro.getI2cAddress().get7Bit()));
        }
    }

    protected void initSensors()
    {
        initSensors(true);
    }

    protected void initCapabilities()
    {
        for (Map.Entry<String, Boolean> mEnt : capMap.entrySet())
        {
            RobotLog.dd(TAG, mEnt.getKey() + " = " + mEnt.getValue());
        }
    }

    public boolean getCapability(String cap)
    {
        Boolean isCap = capMap.get(cap);
        return isCap != null && isCap;
    }

    private double getTotalGearRatio()
    {
        double gr = 1.0;
        for(double g : DRIVE_GEARS) gr *= g;
        return gr;
    }

    protected void computeCPI()
    {
        CPI = RobotConstants.DT_CPI;
    }

    public void setDriveDir (DriveDir ddir)
    {
        RobotLog.ii(TAG, "setDriveDir to " + ddir + " from " + this.ddir);
        if(ddir == DriveDir.UNKNOWN)
        {
            RobotLog.ee(TAG, "setDriveDir called with UNKNOWN");
            return;
        }

        if(ddir == DriveDir.RIGHT || ddir == DriveDir.LEFT)
        {
            RobotLog.dd(TAG, "setDriveDir called with R/L");
            if(calibrationDriveDir == DriveDir.UNKNOWN) calibrationDriveDir =
                RobotConstants.DT_DIR;
            return;
        }

        if(calibrationDriveDir == DriveDir.UNKNOWN) calibrationDriveDir = ddir;

        if(this.ddir == ddir)
            return;

        RobotLog.ii(TAG, "Setting Drive Direction to " + ddir + " from " + this.ddir);
        RobotLog.ii(TAG, "Calibration Drive Direction: " + calibrationDriveDir);

        this.ddir = ddir;

        invertDrive = ddir != calibrationDriveDir;
    }

    public void setInitHdg(double initHdg)
    {
        this.initHdg = (int) Math.round(initHdg);
    }

    public boolean calibrateGyro()
    {
        if(gyro == null)
        {
            RobotLog.ee(TAG, "NO GYRO FOUND TO CALIBRATE");
            return false;
        }

        if(calibrationDriveDir == DriveDir.UNKNOWN)
        {
            RobotLog.ii(TAG, "calibrateGyro called without having set a drive Direction. " +
                       "Defaulting to  " + RobotConstants.DT_DIR);
            setDriveDir(RobotConstants.DT_DIR);
        }

        RobotLog.ii(TAG, "Starting gyro calibration");
        RobotLog.ii(TAG, "Calibration drive dir = %s", calibrationDriveDir);
        gyro.calibrate();

        double gyroInitTimout = 5.0;
        boolean gyroCalibTimedout = false;
        ElapsedTime gyroTimer = new ElapsedTime();

        while (!op.isStopRequested() &&
               gyro.isCalibrating())
        {
            op.sleep(50);
            if(gyroTimer.seconds() > gyroInitTimout)
            {
                RobotLog.ii(TAG, "GYRO INIT TIMED OUT!!");
                gyroCalibTimedout = true;
                break;
            }
        }
        RobotLog.ii(TAG, "Gyro calibrated in %4.2f seconds", gyroTimer.seconds());

        boolean gyroReady = !gyroCalibTimedout;
        if(gyroReady) gyro.resetZAxisIntegrator();
        this.gyroReady = gyroReady;
        return gyroReady;
    }

    public void resetGyro()
    {
        if(gyro != null && gyroReady)
        {
            gyro.resetZAxisIntegrator();
        }
    }

    public double getGyroHdg()
    {
        return gyro.getIntegratedZValue();
    }

    public double getGyroFhdg()
    {
        if(imu == null && gyro == null) return 0;
        int dirHdgAdj = 0;
        //if(ddir != calibrationDriveDir) dirHdgAdj = 180;

        double rawGyroHdg = getGyroHdg();
        //There have been cases where gyro incorrectly returns 0 for a frame : needs filter
        //Uncomment the following block for a test filter
//        if(rawGyroHdg == 0 &&
//           Math.abs(lastRawGyroHdg) > 30)
//        {
//            rawGyroHdg = lastRawGyroHdg;
//        }
//        else
//        {
//            lastRawGyroHdg = rawGyroHdg;
//        }

        //NOTE:  gyro.getIntegratedZValue is +ve CCW (left turn)
        //WHEN the freaking gyro is mounted upright.
        //Since snowman 2.0 has gyro mounted upside down, we need
        //to negate the value!!
        int gDir = 1;
        if(gyroInverted) gDir = -1;
        double cHdg = gDir * rawGyroHdg + initHdg + dirHdgAdj;

        while (cHdg <= -180) cHdg += 360;
        while (cHdg >   180) cHdg -= 360;

        return cHdg;
    }

    public String getName() {return name;}
    public void setName(String name) {this.name = name;}

    //INTAKE is the intake side and should have configured left motors to left.
    public enum DriveDir
    {
        UNKNOWN,
        INTAKE,
        PUSHER,
        LEFT,
        RIGHT
    }

    @SuppressWarnings("unused")
    private int getColorPort()
    {
        return colorPort;
    }

    public boolean getInvertDrive() { return invertDrive; }

    public void turnColorOn()
    {
        if(colorSensor == null) return;
        RobotLog.ii(TAG, "Turning on colorSensor LED");
        colorEnabled = true;
        //colorSensor.getI2cController().registerForI2cPortReadyCallback(colorSensor,
        //        getColorPort());

        op.sleep(50);
        colorSensor.enableLed(true);
    }

    public void turnColorOff()
    {
        if(colorSensor == null) return;
        colorEnabled = false;
        colorSensor.enableLed(false);
        op.sleep(50);
        //colorSensor.getI2cController().deregisterForPortReadyCallback(getColorPort());
    }

    public DriveDir getDriveDir() { return ddir; }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs)
    {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            op.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    //With phone laid flat in portrait mode with screen up:
    //The phone axis is 0,0,0 at Camera (using front camera)
    //X pts to the right side of phone (ZTE volume button edge)
    //Y pts to top of phone (head phone jack edge)
    //Z pts out of camera - initially toward bot up
    //to mount camera on front of bot looking bot fwd,
    //rotate -90 about z, then -90 about x
    //translate 0 in bot x, half bot length in bot y, and ~11" in bot z
    public static OpenGLMatrix phoneOrientation;
    public static OpenGLMatrix phoneLocationOnRobot;
    static
    {
        phoneOrientation = Orientation.getRotationMatrix(
                AxesReference.EXTRINSIC, AxesOrder.XYZ, //ZXY
                AngleUnit.DEGREES, 0, 0, 0);

        phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_X_IN_BOT, CAMERA_Y_IN_BOT, CAMERA_Z_IN_BOT)
                .multiplied(phoneOrientation);
    }
}
