package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.Units;

@SuppressWarnings({"FieldCanBeLocal", "unused", "WeakerAccess"})
public class SkyBot extends TilerunnerGtoBot {
    private final String TAG = "SJH SkyRobot";

    public DistanceSensor distSens = null;

    public DcMotor _liftyBoi = null;
    //5202 series yellow jacket motor w/ 5.2:1 gear box has 145.6 cpr of output shaft
    //  1150 rpm (19.17 rps) no load speed
    //
    // Replaced with 13.7:1 gear box motor with 383.6 cpr
    // And Replaced with 3.7:1 gear box motor with 103.6 cpr
    //lift is made with 1:1 gear between motor output shaft and lead screw
    //
    //gobilda lead screw has 7.9in range, 2mm pitch, and 4 start
    //lead=2mm pitch * 4 start = 8 mm/lead screw rev
    //19.17rps * 8mm/rev = 153mm/s no load lift speed (6in/s)
    //201mm (7.9in) range in 201/153=1.31 s
    //201mm / 8mm/rev = 25.125rev
    //25.125rev * 145.6 cpr = 3658 counts

    public DcMotor parkMotor = null;

    private final double LIFTER_CPER = 28; //quad encoder cnts/encoder rev
    private final double LIFTER_INT_GEAR = 3.7; //13.7;  // 5.2;
    private final double LIFTER_CPOR = LIFTER_CPER * LIFTER_INT_GEAR; //383.6 //145.6 cnts/outShaftRev
    private final double LIFTER_EXT_GEAR = 1.0;
    private final double LIFTER_PITCH = 2.0; //2.0mm/pitch
    private final int LIFTER_STARTS = 4; //pitch/rev
    private final double LIFTER_LEAD = LIFTER_PITCH * LIFTER_STARTS; //8 mm/rev
    private final double LIFTER_CPMM = LIFTER_EXT_GEAR * LIFTER_CPOR / LIFTER_LEAD;
    public final double LIFTER_CPI = LIFTER_CPMM * 25.4;
    @SuppressWarnings("FieldCanBeLocal")
    private final int LIFTER_THRESH = (int) (0.2 * LIFTER_CPI);
    private final double LIFTER_STOW =  0.0;
    private final double LIFTER_SAFE = -1.0;
    private final double LIFTER_GRAB = -4.0;
    private final double LIFTER_LOW  = -4.0;
    private final double LIFTER_MOVE = -3.75;
    private final double LIFTER_EASE = -3.00;
    private final double LIFTER_REL1 = -1.0;
    private final double LIFTER_REL2 =  1.0;
    private final double LIFTER_MAX  =  2.0;
    public final int LIFT_STOW_CNTS = (int) (LIFTER_CPI * LIFTER_STOW);
    private final int LIFT_SAFE_CNTS = (int) (LIFTER_CPI * LIFTER_SAFE);
    public  final int LIFT_GRAB_CNTS = (int) (LIFTER_CPI * LIFTER_GRAB);
    private final int LIFT_LOW_CNTS  = (int) (LIFTER_CPI * LIFTER_LOW);
    public  final int LIFT_MOVE_CNTS = (int) (LIFTER_CPI * LIFTER_MOVE);
    public  final int LIFT_EASE_CNTS = (int) (LIFTER_CPI * LIFTER_EASE);
    public  final int LIFT_REL1_CNTS = (int) (LIFTER_CPI * LIFTER_REL1);
    public  final int LIFT_REL2_CNTS = (int) (LIFTER_CPI * LIFTER_REL2);
    private final int LIFT_MAX_CNTS  = (int) (LIFTER_CPI * LIFTER_MAX);

    public Servo lplatch = null;
    public Servo rplatch = null;
    @SuppressWarnings("FieldCanBeLocal")
    private final double LPLATCH_STOW = 0.02;
    @SuppressWarnings("FieldCanBeLocal")
    private final double LPLATCH_PRE = 0.32;
    @SuppressWarnings("FieldCanBeLocal")
    private final double LPLATCH_GRAB = 0.78;
    @SuppressWarnings("FieldCanBeLocal")
    private final double RPLATCH_STOW = 0.80;
    @SuppressWarnings("FieldCanBeLocal")
    private final double RPLATCH_PRE = 0.50;
    @SuppressWarnings("FieldCanBeLocal")
    private final double RPLATCH_GRAB = 0.04;

    @SuppressWarnings("FieldCanBeLocal")
    private final double GRIPPER_STOW = 0.40;
    @SuppressWarnings("FieldCanBeLocal")
    private final double GRIPPER_OPEN = 0.99;
    @SuppressWarnings("FieldCanBeLocal")
    private final double GRIPPER_PRE = 0.6;
    @SuppressWarnings("FieldCanBeLocal")
    private final double GRIPPER_GRAB = 0.32;


    private final int EXTND_COUNTS_PER_MOTOR_REV = 4;
    private final double EXTND_GEAR_ONE = 72;
    private final double EXTND_WHEEL_DIAM = 1.57;
    private final double EXTND_CPR = EXTND_COUNTS_PER_MOTOR_REV * EXTND_GEAR_ONE;
    private final double EXTND_SCALE = 1.0;
    public double EXTND_CPI = EXTND_CPR / (Math.PI * EXTND_WHEEL_DIAM) / EXTND_SCALE;
    private final int EXT_THRESH = (int) (0.667 * EXTND_CPI);
    private int MIN_EXTND_CNT    = 0;
    private int ARM_EXT_STOW_POS =  (int)  (0.0 * EXTND_CPI);
    public  int ARM_EXT_SNUG_POS =  (int)  (7.2 * EXTND_CPI);
    public  int ARM_EXT_STAGE_POS = (int)  (9.5 * EXTND_CPI);
    public  int ARM_EXT_PUSH_POS =  (int) (12.0 * EXTND_CPI);
    public  int ARM_EXT_GRAB_POS =  (int) (14.0 * EXTND_CPI);
    public  int ARM_EXT_DROP_POS =  (int) (16.0 * EXTND_CPI);
    public  int ARM_EXT_XTND_POS =  (int) (17.0 * EXTND_CPI);
    private int MAX_EXTND_CNT    =  (int) (18.0 * EXTND_CPI);

    private int ARMROT_COUNTS_PER_MOTOR_REV = 28;
    private double ARMROT_GEAR_ONE = 50.9; //13.7;
    private double ARMROT_GEAR_EXT = 2.0;
    private double ARMROT_CPR = ARMROT_COUNTS_PER_MOTOR_REV * ARMROT_GEAR_ONE * ARMROT_GEAR_EXT;
    public double ARMROT_CPD = ARMROT_CPR / 360.0;
    private final double ROT_THRESH = 3.0*ARMROT_CPD;
    private final double ROT_PID_THRESH = 8.0*ARMROT_CPD;
    public int ARM_ROT_FWD = (int) (  0.0 * ARMROT_CPD);
    public int ARM_ROT_RGT = (int) (-90.0 * ARMROT_CPD);
    public int ARM_ROT_LFT = (int) ( 90.0 * ARMROT_CPD);
    public int ARM_DRP_RGT = (int) (-25.0 * ARMROT_CPD);
    public int ARM_DRP_LFT = (int) ( 25.0 * ARMROT_CPD);
    private int ARM_ROT_MIN = (int) (-135.0 * ARMROT_CPD);
    private int ARM_ROT_MAX = (int) ( 135.0 * ARMROT_CPD);
    public int ARM_GRB_RGT_L = (int) ( -65.0 * ARMROT_CPD);
    public int ARM_GRB_RGT_R = (int) (-117.0 * ARMROT_CPD);
    public int ARM_GRB_LFT_L = (int) ( 117.0 * ARMROT_CPD);
    public int ARM_GRB_LFT_R = (int) (  65.0 * ARMROT_CPD);

    public int ARM_EXT_RGT_R = (int) (-100.0 * ARMROT_CPD);
    public int ARM_EXT_LFT_L = (int) ( 100.0 * ARMROT_CPD);
    public int ARM_EXT_RGT_L = (int) (-80.0 * ARMROT_CPD);
    public int ARM_EXT_LFT_R = (int) ( 80.0 * ARMROT_CPD);

    //following asumes 270 degree servo range
    //public double ARM_ROT_RGT = 0.16;
    //public double ARM_DRP_RGT = 0.32;
    //public double ARM_ROT_FWD = 0.52;
    //public double ARM_DRP_LFT = 0.72;
    //public double ARM_ROT_LFT = 0.88;

//    public static double ARM_GRB_RGT_L = 0.23;
//    public static double ARM_GRB_RGT_R = 0.03;
//    public static double ARM_GRB_LFT_L = 1.00;
//    public static double ARM_GRB_LFT_R = 0.84;

//    public double ARM_ROT_MIN = 0.0;
//    public double ARM_ROT_MAX = 1.0;

    private final int PRKR_COUNTS_PER_MOTOR_REV = 4;
    private final double PRKR_GEAR_ONE = 72;
    private final double PRKR_WHEEL_DIAM = 2.25;
    private final double PRKR_CPR = PRKR_COUNTS_PER_MOTOR_REV * PRKR_GEAR_ONE;
    private final double PRKR_SCALE = 1.0;
    public double PRKR_CPI = PRKR_CPR / (Math.PI * PRKR_WHEEL_DIAM) / PRKR_SCALE;
    private final int PRKR_THRESH = (int) (1.0 * PRKR_CPI);
    private int PRKR_EXTND_CNT    = 0;
    private int PRKR_STOW_POS =  (int)  (0.0 * EXTND_CPI);
    public  int PRKR_SHRT_POS =  (int)  (6.0 * EXTND_CPI);
    public  int PRKR_MID_POS  =  (int)  (12.0 * EXTND_CPI);
    public  int PRKR_LNG_POS  =  (int)  (30.0 * EXTND_CPI);

    private DigitalChannel armIndexSensor = null;

    public  DcMotor  armRotate   = null;
    //public Servo armRotate = null;
    public DcMotor armExtend = null;

    public SkyBot() {
        super();

        CAMERA_X_IN_BOT = 4.0f * (float) Units.MM_PER_INCH;
        CAMERA_Y_IN_BOT = 0.0f * (float) Units.MM_PER_INCH;
        CAMERA_Z_IN_BOT = 3.0f * (float) Units.MM_PER_INCH;
    }

    public SkyBot(String name) {
        this();
        this.name = name;
    }

    @Override
    public void init(LinearOpMode op, boolean initDirSensor) {
        computeCPI();

        initOp(op);
        initDriveMotors();
        initCollectorLifter();
        initSensors(initDirSensor);
        initArm();
        initHolder();
        initParker();
        initCapabilities();
    }

    @Override
    public void initCollectorLifter() //elev and stone gripper
    {
        initGripper();
        initLifter();
    }

    private double curRotSrvPos = 0.5;

    @Override
    public void initArm() //rotate, extend
    {
        RobotLog.dd(TAG, "GTO initArm");
        try {
            armExtend = hwMap.dcMotor.get("armExtend");
            armExtend.setDirection(DcMotorSimple.Direction.FORWARD);
            armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            RobotLog.ee(TAG, "ERROR in initArm\n" + e.toString());
        }
        try {
            armRotate  = hwMap.dcMotor.get("armRotate");
//            armRotate = hwMap.servo.get("armRotate");
//            if (armRotate != null) curRotSrvPos = armRotate.getPosition();
            armRotate.setDirection(DcMotorSimple.Direction.REVERSE);
            armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            RobotLog.ee(TAG, "ERROR in initArm\n" + e.toString());
        }
        if(armRotate != null && armExtend !=null) capMap.put("arm", true);
    }

    @Override
    public void initHolder() //Platform grabber
    {
        boolean success = true;
        try {
            rplatch = hwMap.servo.get("rplatch");
        } catch (Exception e) {
            return;
        }

        try {
            lplatch = hwMap.servo.get("lplatch");
        } catch (Exception e) {
            return;
        }

        capMap.put("holder", true);
    }

    @Override
    public void initSensors()
    {
        super.initSensors();
        try
        {
            distSens = hwMap.get(DistanceSensor.class, "dist1");
        }
        catch (Exception e)
        {
            RobotLog.dd(TAG, "No distance sensor");
        }
    }

    @SuppressWarnings("UnusedReturnValue")
    private boolean initGripper()
    {
        try
        {
            gripper = hwMap.servo.get("gripper");

            //if extended range is wanted (and available) on servo,
            //keep block below
            //if standard range is wanted, comment block below
            if (gripper.getController() instanceof ServoControllerEx)
            {
                // Set the rotation servo for extended PWM range
                ServoControllerEx srvCntrlrEx =
                        (ServoControllerEx) gripper.getController();
                int gPort = gripper.getPortNumber();
                PwmControl.PwmRange range =
                        new PwmControl.PwmRange(500, 2500);
                srvCntrlrEx.setServoPwmRange(gPort, range);
            }
        } catch (Exception e) {
            RobotLog.ee(TAG, "ERROR in initGripper\n" + e.toString());
            return false;
        }

        capMap.put("collector", true);
        return true;
    }

    @SuppressWarnings("UnusedReturnValue")
    private boolean initLifter()
    {
        try
        {
            _liftyBoi = hwMap.dcMotor.get("liftyboi");

            _liftyBoi.setDirection(DcMotorSimple.Direction.FORWARD);
            //_liftyBoi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            _liftyBoi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            _liftyBoi.setPower(0.0f);
            _liftyBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR in initLifter\n" + e.toString());
            return false;
        }

        capMap.put("lifter", true);
        return true;
    }

    @SuppressWarnings("UnusedReturnValue")
    private boolean initParker()
    {
        try
        {
            parkMotor = hwMap.dcMotor.get("parkyboi");

            parkMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            //_liftyBoi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            parkMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            parkMotor.setPower(0.0f);
            parkMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR in parkyboi\n" + e.toString());
            return false;
        }

        return true;
    }

    public boolean isElevTouchPressed() {
        return false;
    }

    public boolean isArmTouchPressed() {
        return false;
    }

    private boolean lastExtndUseCnts = false;

    public void setExtend(double spd, boolean useCnts, boolean override) {
        double aspd = spd;

        if (armExtend == null) {
            RobotLog.ee(TAG, "ERROR in setExtend - armExtend null\n");
            return;
        }

        int curArmCounts = armExtend.getCurrentPosition();
        if (useCnts)
        {
            if (!lastExtndUseCnts)
            {
                RobotLog.dd(TAG, "Moving to arm pos %d %f",
                        curArmCounts, curArmCounts/EXTND_CPI);
                armExtend.setTargetPosition(curArmCounts);
                armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armExtend.setPower(aspd);
            }
        }
        else
        {
            if(armExtend.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
                armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (curArmCounts <= MIN_EXTND_CNT && !override) aspd =
                    Range.clip(aspd, 0, 1.0);
            if (curArmCounts >= MAX_EXTND_CNT && !override) aspd =
                    Range.clip(aspd, -1.0, 0.0);
            armExtend.setPower(aspd);
        }
        lastExtndUseCnts = useCnts;
    }

    private void startExtendMove(int targetPos)
    {
        if (armExtend == null) {
            RobotLog.ee(TAG, "armExtend is null - fix it");
            return;
        }

        int curPos = armExtend.getCurrentPosition();

        RobotLog.dd(TAG, "armExtend to %d %f", targetPos, EXTND_CPI);

        armExtend.setPower(0.0);
        armExtend.setTargetPosition(targetPos);
        double armExtSpd = 0.95;

        if (Math.abs(targetPos - curPos) < EXT_THRESH) armExtSpd = 0.0;
        armExtend.setTargetPosition(targetPos);

        if(targetPos > curPos)
            armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else
        {
            armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armExtSpd = -armExtSpd;
        }

        if (Math.abs(targetPos - curPos) < EXT_THRESH) armExtSpd = 0.0;

        RobotLog.dd(TAG, "armExtend to %d %f pwr: %f", targetPos, EXTND_CPI, armExtSpd);
        armExtend.setPower(armExtSpd);
    }

    public void setExtendPos(int targetPos) //targetPos in counts
    {
        if (armExtend == null)
        {
            RobotLog.ee(TAG, "armExtend is null - fix it");
            return;
        }

        startExtendMove(targetPos);

        ElapsedTime extendTimer = new ElapsedTime();
        double extendTimeout = 4.0;
        int curPos = armExtend.getCurrentPosition();
        while (op.opModeIsActive() &&
               Math.abs(curPos - targetPos) > EXT_THRESH &&
               extendTimer.seconds() < extendTimeout)
        {
            RobotLog.dd(TAG, "In armExtend.  targetpos=" + targetPos + " curPos=" +
                    curPos);

            if(Math.abs(armExtend.getCurrentPosition() - targetPos) < EXT_THRESH)
            {
                armExtend.setPower(0.2);
                armExtend.setTargetPosition(targetPos);
                break;
            }
            curPos = armExtend.getCurrentPosition();
        }
        armExtend.setPower(0.2);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void putExtendAtStow() {
        setExtendPos(ARM_EXT_STOW_POS);
    }

    public void putExtendAtStage() {
        setExtendPos(ARM_EXT_STAGE_POS);
    }

    public void putExtendAtStageIfLow() {
        if(getLiftPos() < LIFT_SAFE_CNTS ) setExtendPos(ARM_EXT_STAGE_POS);
    }

    public void putExtendAtSnug() {
        setExtendPos(ARM_EXT_SNUG_POS);
    }

    public void putExtendAtGrab() {
        setExtendPos(ARM_EXT_GRAB_POS);
    }

    public void putExtendAtDrop()

    {
        setExtendPos(ARM_EXT_DROP_POS);
    }

    public void zeroArmExtend()
    {
        if(armExtend == null) return;
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void startParkerMove(int targetPos)
    {
        if (parkMotor == null) {
            RobotLog.ee(TAG, "parker is null - fix it");
            return;
        }

        int curPos = parkMotor.getCurrentPosition();

        RobotLog.dd(TAG, "parkMotor to %d %f", targetPos, PRKR_CPI);

        parkMotor.setPower(0.0);
        parkMotor.setTargetPosition(targetPos);
        double parkerSpd = 0.95;

        if (Math.abs(targetPos - curPos) < PRKR_THRESH) parkerSpd = 0.0;
        parkMotor.setTargetPosition(targetPos);

        if(targetPos > curPos)
            parkMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else
        {
            parkMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            parkerSpd = -parkerSpd;
        }

        if (Math.abs(targetPos - curPos) < PRKR_THRESH) parkerSpd = 0.0;

        RobotLog.dd(TAG, "parker to %d %f pwr: %f", targetPos, PRKR_CPI, parkerSpd);
        parkMotor.setPower(parkerSpd);
    }

    public void setParkerPos(int targetPos) //targetPos in counts
    {
        if (parkMotor == null)
        {
            RobotLog.ee(TAG, "parker is null - fix it");
            return;
        }

        startParkerMove(targetPos);

        int curPos = parkMotor.getCurrentPosition();
        while (op.opModeIsActive() &&
                Math.abs(curPos - targetPos) > PRKR_THRESH)
        {
            RobotLog.dd(TAG, "In setParkerPos.  targetpos=" + targetPos + " curPos=" +
                    curPos);

            curPos = parkMotor.getCurrentPosition();
            if(Math.abs(curPos - targetPos) < PRKR_THRESH)
            {
                parkMotor.setPower(0.8);
                parkMotor.setTargetPosition(targetPos);
                break;
            }

        }

        parkMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void zeroParker()
    {
        if(parkMotor == null) return;
        parkMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parkMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void putParkHome()   { setParkerPos(PRKR_STOW_POS); }
    public void putParkShrt()   { setParkerPos(PRKR_SHRT_POS); }
    public void putParkMid()    { setParkerPos(PRKR_MID_POS); }
    public void putParkLong()   { setParkerPos(PRKR_LNG_POS); }

    public void threadedParkLong()
    {
        class ParkRunnable implements Runnable
        {
            public void run()
            {
                putParkLong();
            }
        }

        ParkRunnable mr = new ParkRunnable();
        Thread parkThread = new Thread(mr, "parkThread");
        parkThread.start();

    }

    private boolean lastRotUseCnts = false;

    public void setRotate(double spd, boolean useCnts, boolean override)
    {
        double aspd = spd;

        if (armRotate == null)
        {
            RobotLog.ee(TAG, "ERROR in setRotate - armRotate null\n");
            return;
        }

        int curArmCounts = armRotate.getCurrentPosition();
        if(useCnts)
        {
            if(!lastRotUseCnts)
            {
                RobotLog.dd(TAG, "Moving to arm pos %d", curArmCounts);
                armRotate.setTargetPosition(curArmCounts);
                armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRotate.setPower(aspd);
            }
        }
        else
        {
            armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(curArmCounts <= ARM_ROT_MIN && !override) aspd = Range.clip(aspd, 0, 1.0);
            if(curArmCounts >= ARM_ROT_MAX && !override) aspd = Range.clip(aspd, -1.0, 0.0);
            armRotate.setPower(aspd);
        }
        lastRotUseCnts = useCnts;
    }

//    public void setRotate (double scale)
//    {
//        double baseScale = 0.040;
//        curRotSrvPos = armRotate.getPosition();
//        curRotSrvPos = Range.clip(curRotSrvPos + baseScale * scale, -1.0, 1.0);
//        RobotLog.dd(TAG,"Setting armRotate to %f", curRotSrvPos);
//        armRotate.setPosition(curRotSrvPos);
//    }

    private void startRotMove(double rotPos)
    {
        if(armRotate == null)
        {
            RobotLog.ee(TAG, "armRotate is null - fix it");
            return;
        }

        RobotLog.dd(TAG, "armRotate to %d %f", (int)rotPos, rotPos/ARMROT_CPD);
        //armRotate.setPosition(targetPos);

        armRotate.setPower(0.0);
        armRotate.setTargetPosition((int)rotPos);
        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double armRotSpd = 0.30;
        armRotate.setPower(armRotSpd);
    }
    //public void setRotatePos(int targetPos)
    public void setRotatePos(double targetPos)
    {
        if(armRotate == null)
        {
            RobotLog.ee(TAG, "armRotate is null - fix it");
            return;
        }

        startRotMove(targetPos);

        while(op.opModeIsActive())
        {
            RobotLog.dd(TAG, "In armRotate.  targetpos=" + targetPos + " curPos=" +
                    armRotate.getCurrentPosition());
            if(Math.abs(armRotate.getCurrentPosition() - targetPos) < (int)ROT_THRESH)
            {
                break;
            }
        }
        armRotate.setPower(0.0);
        armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void putArmForward()   { setRotatePos(ARM_ROT_FWD); }
    public void putArmLeft()      { setRotatePos(ARM_ROT_LFT); }
    public void putArmRight()     { setRotatePos(ARM_ROT_RGT); }
    public void putArmHalfLeft()  { setRotatePos(ARM_DRP_LFT); }
    public void putArmHalfRight() { setRotatePos(ARM_DRP_RGT); }

    public void zeroArmRotate()
    {
        if(armRotate == null) return;
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void openGripper()
    {
        if(gripper == null) return;
        RobotLog.dd(TAG, "openGripper %.2f", GRIPPER_OPEN);
        gripper.setPosition(GRIPPER_OPEN);
    }

    public void stowGripper()
    {
        if(gripper == null) return;
        RobotLog.dd(TAG, "stowGripper %.2f", GRIPPER_STOW);
        gripper.setPosition(GRIPPER_STOW);
    }

    public void stageGripper()
    {
        if(gripper == null) return;
        RobotLog.dd(TAG, "stageGripper %.2f", GRIPPER_PRE);
        gripper.setPosition(GRIPPER_PRE);
    }

    public void closeGripper()
    {
        if(gripper == null) return;
        RobotLog.dd(TAG, "closeGripper %.2f", GRIPPER_GRAB);
        gripper.setPosition(GRIPPER_GRAB);
    }

    private void startLiftMove(int targetPos)
    {
        if(_liftyBoi == null)
        {
            RobotLog.ee(TAG, "liftyboi is null - fix it");
            return;
        }

        int curPos = _liftyBoi.getCurrentPosition();

        RobotLog.dd(TAG, "liftyboi to %d %f", targetPos, targetPos/LIFTER_CPI);

        _liftyBoi.setPower(0.0);
        _liftyBoi.setTargetPosition(targetPos);
        //_liftyBoi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _liftyBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double liftSpd = 0.95;
        if(targetPos - curPos < 0) liftSpd = -liftSpd;
        else if (Math.abs(targetPos - curPos) < LIFTER_THRESH) liftSpd = 0.0;

        _liftyBoi.setPower(liftSpd);
    }

    public void setLiftPos(int targetPos)
    {
        if(_liftyBoi == null)
        {
            RobotLog.ee(TAG, "liftyboi is null - fix it");
            return;
        }

        startLiftMove(targetPos);

        ElapsedTime liftTimer = new ElapsedTime();
        double liftTimeLimit = 4.0;
        while(op.opModeIsActive() && liftTimer.seconds() < liftTimeLimit)
        {
            RobotLog.dd(TAG, "In lift.  targetpos=" + targetPos + " curPos=" +
                    _liftyBoi.getCurrentPosition());
            if(Math.abs(_liftyBoi.getCurrentPosition() - targetPos) < LIFTER_THRESH)
            {
                _liftyBoi.setTargetPosition(targetPos);
                _liftyBoi.setPower(0.2);
                break;
            }
        }
        _liftyBoi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getLiftPos()
    {
        int pos = -9999;
        if(_liftyBoi != null) pos = _liftyBoi.getCurrentPosition();
        return pos;
    }

    public void threadedLiftStage()
    {
        class LiftRunnable implements Runnable
        {
            public void run()
            {
                putLiftAtGrab();
            }
        }

        LiftRunnable mr = new LiftRunnable();
        Thread liftThread = new Thread(mr, "liftThread");
        liftThread.start();
    }

    public void moveLift(double inches)
    {
        int curPos = _liftyBoi.getCurrentPosition();
        int cnts = (int) (inches*LIFTER_CPI);
        setLiftPos(curPos + cnts);
    }

    public void zeroLift()
    {
        if(_liftyBoi == null) return;
        _liftyBoi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _liftyBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void putLiftAtGrab()
    {
        setLiftPos(LIFT_GRAB_CNTS);
    }

    public void putLiftAtLow()
    {
        setLiftPos(LIFT_LOW_CNTS);
    }

    public void putLiftAtStow()
    {
        setLiftPos(LIFT_STOW_CNTS);
    }

    public void putLiftAtMove()
    {
        setLiftPos(LIFT_MOVE_CNTS);
    }

    public void putLiftAtDrop()
    {
        setLiftPos(LIFT_REL1_CNTS);
    }

    public void putLiftAtLevel2()
    {
        setLiftPos(LIFT_REL2_CNTS);
    }

    public boolean armIsMoving = false;

    public void moveArmToLoc(double liftPos, double rotPos, double extndPos)
    {
        moveArmToLoc(liftPos, rotPos, extndPos, 0.0, 0.0, 0.0);
    }

    public void moveArmToLoc(double liftPos, double rotPos, double extndPos,
                             double liftDly, double rotDly, double extndDly)
    {
        moveArmToLoc(liftPos, rotPos, extndPos, liftDly, rotDly, extndDly, 0, 0.0);
    }

    public void moveArmToLoc(double liftPos, double rotPos, double extndPos,
                             double liftDly, double rotDly, double extndDly,
                             int gripAct,    double gripDly)
    {
        armIsMoving = true;
        RobotLog.dd(TAG, "moveArmTo " + liftPos + " " + rotPos + " " + extndPos);

        ElapsedTime extendTimer = new ElapsedTime();
        ElapsedTime liftTimer = new ElapsedTime();
        ElapsedTime rotTimer = new ElapsedTime();
        ElapsedTime grpTimer = new ElapsedTime();

        double extendTimeout = 2.5;
        double liftTimeLimit = 2.5;

        int curExtPos = armExtend.getCurrentPosition();
        int curLiftPos = _liftyBoi.getCurrentPosition();
        int curRotPos = armRotate.getCurrentPosition();

        boolean liftStarted = false;
        boolean xtndStarted = false;
        boolean arotStarted = false;
        boolean gripStarted = false;
        boolean liftDone = false;
        boolean xtndDone = false;
        boolean arotDone = false;

        double liftCompleteTime;
        double xtndCompleteTime;
        double arotCompleteTime;

        while (op.opModeIsActive())
        {
            if(grpTimer.seconds() > gripDly && !gripStarted)
            {
                gripStarted = true;
                if(gripAct == 1) openGripper();
                else if (gripAct == -1) closeGripper();
            }

            if(liftTimer.seconds() > liftDly && !liftStarted)
            {
                liftStarted = true;
                liftTimer.reset();
                startLiftMove((int)liftPos);
            }
            else if(liftStarted && !liftDone)
            {
                if(Math.abs(curLiftPos - liftPos) < LIFTER_THRESH ||
                        liftTimer.seconds() > liftTimeLimit)
                {
                    _liftyBoi.setTargetPosition((int)liftPos);
                    _liftyBoi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    _liftyBoi.setPower(0.2);
                    liftCompleteTime = liftTimer.seconds();
                    RobotLog.dd(TAG, "Completed lift in " + liftCompleteTime + "s");
                    liftDone = true;
                }
                else
                {
                    RobotLog.dd(TAG, "In lift.  targetpos=" + liftPos +
                            " " + liftPos/LIFTER_CPI + " curPos=" + curLiftPos +
                            " " + curLiftPos/LIFTER_CPI);
                    curLiftPos = _liftyBoi.getCurrentPosition();
                }
            }

            if(extendTimer.seconds() > extndDly && !xtndStarted)
            {
                xtndStarted = true;
                extendTimer.reset();
                startExtendMove((int)extndPos);
            }
            else if(xtndStarted && !xtndDone)
            {
                if(Math.abs(curExtPos - extndPos) < EXT_THRESH ||
                        extendTimer.seconds() > extendTimeout)
                {
                    armExtend.setTargetPosition((int)extndPos);
                    armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armExtend.setPower(0.4);
                    xtndCompleteTime = extendTimer.seconds();
                    RobotLog.dd(TAG, "Completed extend in " + xtndCompleteTime + "s");
                    xtndDone = true;
                }
                else
                {
                    RobotLog.dd(TAG, "In armExtend.  targetpos=" + (int) extndPos +
                            " " + extndPos/EXTND_CPI + " " +
                            " curPos=" + curExtPos + " " + curExtPos/EXTND_CPI);
                    curExtPos = armExtend.getCurrentPosition();
                }
            }

            if(rotTimer.seconds() > rotDly && !arotStarted)
            {
                arotStarted = true;
                rotTimer.reset();
                startRotMove((int)rotPos);
            }
            else if (arotStarted && !arotDone)
            {
                if(Math.abs(curRotPos - rotPos) < ROT_PID_THRESH)
                {
                    //armRotate.setPower(0.1);
                    arotCompleteTime = rotTimer.seconds();
                    RobotLog.dd(TAG, "Completed rotate in " + arotCompleteTime + "s");
                    arotDone = true;
                    armRotate.setPower(0.3);
                    armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                else
                {
                    RobotLog.dd(TAG, "In armRot.  targetpos=" + rotPos +
                            " " + rotPos/ARMROT_CPD + " " + " curPos=" +
                            curRotPos + " " + curRotPos/ARMROT_CPD);
                    curRotPos = armRotate.getCurrentPosition();
                }
            }

            if(arotDone && liftDone && xtndDone)
            {
                armIsMoving = false;
                RobotLog.dd(TAG, "Completed moveArmToLoc");
                break;
            }
        }

        //armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //_liftyBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void putHolderAtStow()
    {
        if(lplatch != null) lplatch.setPosition(LPLATCH_STOW);
        if(rplatch != null) rplatch.setPosition(RPLATCH_STOW);
    }

    public void putHolderAtPre()
    {
        if(lplatch != null) lplatch.setPosition(LPLATCH_PRE);
        if(rplatch != null) rplatch.setPosition(RPLATCH_PRE);
    }

    public void putHolderAtGrab()
    {
        if(lplatch != null) lplatch.setPosition(LPLATCH_GRAB);
        if(rplatch != null) rplatch.setPosition(RPLATCH_GRAB);
    }
}
