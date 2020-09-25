package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.robot.SkyBot;
import org.firstinspires.ftc.teamcode.util.Input_Shaper;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Segment;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@SuppressWarnings("unused")
@TeleOp(name="TeleopDriver", group="Tele")
//@Disabled
public class Teleop_Driver extends InitLinearOpMode
{
    private void initPreStart()
    {
        robot.setName(pmgr.getBotName());
        prevOpModeType = SkyBot.curOpModeType;
        SkyBot.curOpModeType = ShelbyBot.OpModeType.TELE;

        /* Initialize the hardware variables. */
        RobotLog.dd(TAG, "Initialize robot");
        //robot.init(this);
        robot.init(this, false);

        if (robot.numLmotors  > 0 &&
            robot.numRmotors  > 0)
        {
            RobotLog.dd(TAG, "Initialize drivetrain");
            robot.setDriveDir(ShelbyBot.DriveDir.INTAKE);
            dtrn.init(robot);

            dtrn.setRampUp(false);
            dtrn.setRampDown(false);

            RobotLog.dd(TAG, "Start Aend fHdg %.2f", robot.getAutonEndHdg());
            //RobotLog.dd(TAG, "Start Hdg %.2f", robot.get);
            RobotLog.dd(TAG, "Start Pos %s", robot.getAutonEndPos().toString());
            RobotLog.dd(TAG, "Start mode to %s", robot.leftMotor.getMode());

            dtrn.setCurrPt(robot.getAutonEndPos());

            lex = (DcMotorEx)(robot.leftMotors.get(0));
            rex = (DcMotorEx)(robot.rightMotors.get(0));
        }
    }

    private void initPostStart()
    {
        robot.putHolderAtGrab();
    }

    private boolean useExtdCnts = false;
    private boolean useRotCnts  = false;

    class ArmTask implements Runnable
    {
        public void run()
        {
            RobotLog.dd(TAG, "Starting arm control arm");
            processControllerInputs();
        }
    }

    private void controlArm()
    {
        controlArmExtend();
        controlArmRotate();
        controlArmElev();
        controlGripper();

        boolean doSafeHome = gpad2.just_pressed(ManagedGamepad.Button.A) && !gamepad2.start;
        boolean doSafeDply = gpad2.just_pressed(ManagedGamepad.Button.B) && !gamepad2.start;
        boolean doZero     = gpad2.just_pressed(ManagedGamepad.Button.X);
        boolean doSafeRght = gpad2.just_pressed(ManagedGamepad.Button.D_RIGHT);
        boolean doSafeLeft = gpad2.just_pressed(ManagedGamepad.Button.D_LEFT);
        boolean doSafeFrwd = gpad2.just_pressed(ManagedGamepad.Button.D_UP);
        boolean doSafeSnug = gpad2.just_pressed(ManagedGamepad.Button.D_DOWN);
        boolean doSetupElv = gpad2.just_pressed(ManagedGamepad.Button.L_TRIGGER);

        if(doSafeHome)
        {
            robot.closeGripper();
            robot.putLiftAtStow();
            robot.putExtendAtStow();
            robot.putArmForward();
        }

        if(doSafeDply)
        {
            robot.closeGripper();
            robot.putExtendAtStage();
            robot.putArmForward();
            robot.putLiftAtGrab();
        }

        if(doZero)
        {
            robot.zeroArmExtend();
            robot.zeroLift();
            robot.zeroArmRotate();
        }

        if(doSafeRght)
        {
            robot.closeGripper();
            robot.putExtendAtStageIfLow();
            robot.putArmRight();
        }

        if(doSafeLeft)
        {
            robot.closeGripper();
            robot.putExtendAtStageIfLow();
            robot.putArmLeft();
        }

        if(doSafeFrwd)
        {
            robot.closeGripper();
            robot.putExtendAtStageIfLow();
            robot.putArmForward();
        }

        if(doSafeSnug)
        {
            robot.closeGripper();
            robot.putExtendAtStageIfLow();
            robot.putArmForward();
            robot.putLiftAtMove();
            robot.putExtendAtSnug();
        }

        if(doSetupElv)
        {
            robot.setLiftPos((int)(3.9 * robot.LIFTER_CPI));
        }

    }

    private void controlArmExtend()
    {
        if(robot.armExtend == null) return;

        if(robot.isArmTouchPressed() && !lastArmTouchPressed)
        {
            robot.zeroArmExtend();
            return;
        }

        lastArmTouchPressed = robot.isElevTouchPressed();

        double  axtnd        = -gpad2.value(ManagedGamepad.AnalogInput.R_STICK_Y);
        boolean overrideLims =  gpad2.pressed(ManagedGamepad.Button.R_BUMP);

        robot.setExtend(axtnd, false, overrideLims);
    }

    private void controlArmRotate()
    {
        if(robot.armRotate == null) return;

        double  arot         =  -gpad2.value(ManagedGamepad.AnalogInput.R_STICK_X);
        boolean changeMode   =  gpad2.just_pressed(ManagedGamepad.Button.Y);
        boolean overrideLims =  gpad2.pressed(ManagedGamepad.Button.R_BUMP);

        double rscale = 0.25;

        if(changeMode)
        {
            useRotCnts = !useRotCnts;
            arot = 0.3;
        }

        if (!useRotCnts) arot *= rscale;
        robot.setRotate(arot, useExtdCnts, overrideLims);
//        robot.setRotate(arot);
    }

    private void controlArmElev()
    {
        if(robot._liftyBoi == null) return;
        double  aelev       = -gpad2.value(ManagedGamepad.AnalogInput.L_STICK_Y);

        //aelev  = ishaper.shape(aelev);
        if(Math.abs(aelev) > 0.05 && robot._liftyBoi.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
        robot._liftyBoi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot._liftyBoi.setPower(aelev);
    }

    private boolean doLatch = false;

    private void controlLatch()
    {
        if(robot.rplatch == null || robot.lplatch == null) return;

        boolean  platchPos    =  gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);

        if(platchPos)
        {
            doLatch = !doLatch;
            if(doLatch)
            {
                robot.putHolderAtGrab();
            }
            else
            {
                robot.putHolderAtPre();
            }
        }
    }

    private void controlParker()
    {
        if(robot.parkMotor == null || driveType == TELEOP_DRIVE_TYPE.TANK_DRIVE) return;

        double pDrive  = -gpad1.value(ManagedGamepad.AnalogInput.L_STICK_Y);

        robot.parkMotor.setPower(pDrive);
    }

    private boolean gripTog = false;
    private void controlGripper()
    {
        boolean grip =  gpad2.just_pressed(ManagedGamepad.Button.L_BUMP) ||
                        gpad2.just_pressed(ManagedGamepad.Button.R_TRIGGER);

        if(grip) gripTog = !gripTog;

        if (gripTog) robot.closeGripper();
        else robot.openGripper();

    }

    private void controlDrive()
    {
        if(robot.leftMotors.size() == 0 && robot.rightMotors.size() == 0) return;
        // Run wheels in tank mode
        // (note: The joystick goes negative when pushed forwards, so negate it)
        boolean toggle_run_mode   = gpad1.just_pressed(ManagedGamepad.Button.X);
        boolean invert_drive_dir  = gpad1.just_pressed(ManagedGamepad.Button.Y);
        boolean toggle_vel        = gpad1.just_pressed(ManagedGamepad.Button.R_TRIGGER);

        boolean step_driveType    = gpad1.just_pressed(ManagedGamepad.Button.A) && !gamepad1.start;
        boolean toggle_float      = gpad1.just_pressed(ManagedGamepad.Button.B) && !gamepad1.start;

        boolean left_step         = gpad1.pressed(ManagedGamepad.Button.D_LEFT);
        boolean right_step        = gpad1.pressed(ManagedGamepad.Button.D_RIGHT);
        boolean fwd_step          = gpad1.pressed(ManagedGamepad.Button.D_UP);
        boolean back_step         = gpad1.pressed(ManagedGamepad.Button.D_DOWN);
        boolean fst_dtl           = gpad1.pressed(ManagedGamepad.Button.L_BUMP);
        boolean xfst_dtl          = gpad1.pressed(ManagedGamepad.Button.L_TRIGGER);

        raw_left                  = -gpad1.value(ManagedGamepad.AnalogInput.L_STICK_Y);
        raw_right                 = -gpad1.value(ManagedGamepad.AnalogInput.R_STICK_Y);
        raw_turn                  =  gpad1.value(ManagedGamepad.AnalogInput.R_STICK_X);

//        boolean goBox       =  gpad1.just_pressed(ManagedGamepad.Button.L_TRIGGER);
//        boolean goPit       =  gpad1.just_pressed(ManagedGamepad.Button.R_TRIGGER);
//
//        if(goBox)
//        {
//            goToBox();
//            return;
//        }
//
//        if(goPit)
//        {
//            goToPit();
//            return;
//        }

        double shp_left  = ishaper.shape(raw_left,  0.1);
        double shp_right = ishaper.shape(raw_right, 0.1);
        double shp_turn  = ishaper.shape(raw_turn, 0.1);

        double arcadeTurnScale = 0.5;

        if(toggle_vel) useSetVel = !useSetVel;

        double step_dist = 2.0;
        double step_spd  = 0.4;
        double step_ang  = 5.0;
        double drvScale = 1.0;

        double fHdg = robot.getGyroFhdg();
        double detail_speed = 0.18;
        if(fst_dtl) detail_speed = 0.3;
        if(xfst_dtl)
        {
            detail_speed = 0.5;
            drvScale = 1.25;
        }

        double maxIPS = 50.0 * drvScale;
        double maxRPS = maxIPS/(4.0*Math.PI);
        double maxDPS = maxRPS*360.0;

        double left  = 0.0;
        double right = 0.0;
        double turn  = 0.0;

        if(fwd_step)
        {
            left  = detail_speed;
            right = detail_speed;
            //dtrn.driveDistanceLinear(step_dist, step_spd, Drivetrain.Direction.FORWARD);
        }
        else if(back_step)
        {
            left  = -detail_speed;
            right = -detail_speed;
            //dtrn.driveDistanceLinear(step_dist, step_spd, Drivetrain.Direction.REVERSE);
        }
        else if(left_step)
        {
            left  = -detail_speed;
            right =  detail_speed;
            //dtrn.setInitValues();
            //dtrn.ctrTurnToHeading(fHdg + step_ang, step_spd);
        }
        else if(right_step)
        {
            left  =  detail_speed;
            right = -detail_speed;
            //dtrn.setInitValues();
            //dtrn.ctrTurnToHeading(fHdg - step_ang, step_spd);
        }
        else
        {
            if(useSetVel)
            {
                switch (driveType)
                {
                    case TANK_DRIVE:
                        left  = raw_left;
                        right = raw_right;
                        break;

                    case ARCADE_DRIVE:
                        left  = raw_right;
                        right = left;
                        left  += raw_turn*arcadeTurnScale;
                        right -= raw_turn*arcadeTurnScale;
                }
            }
            else
            {
                switch (driveType)
                {
                    case TANK_DRIVE:
                        left  = shp_left;
                        right = shp_right;
                        break;

                    case ARCADE_DRIVE:
                        left = shp_right;
                        right = left;
                        turn = (1 - Math.abs(left)) * shp_turn;
                        if(turn < 0.95) turn *= arcadeTurnScale;
                        left  += turn;
                        right -= turn;
                        break;
                }
            }

            double vmax = Math.max(Math.abs(left), Math.abs(right));
            if(vmax > 1.0)
            {
                left  /= vmax;
                right /= vmax;
            }
        }

        @SuppressWarnings("UnusedAssignment")
        double out_left = left;
        @SuppressWarnings("UnusedAssignment")
        double out_right = right;

        if(useSetVel)
        {
            out_left  = maxDPS*left;
            out_right = maxDPS*right;
            lex = (DcMotorEx)(robot.leftMotors.get(0));
            rex = (DcMotorEx)(robot.rightMotors.get(0));
            lex.setVelocity(out_left,  AngleUnit.DEGREES);
            rex.setVelocity(out_right, AngleUnit.DEGREES);
        }
        else
        {
            double governor = 0.8;
            out_left  = left * governor;
            out_right = right * governor;
            robot.leftMotors.get(0).setPower(out_left);
            robot.rightMotors.get(0).setPower(out_right);
        }

        if(toggle_float)
        {
            if(zeroPwr == DcMotor.ZeroPowerBehavior.BRAKE)
                zeroPwr = DcMotor.ZeroPowerBehavior.FLOAT;
            else
                zeroPwr = DcMotor.ZeroPowerBehavior.BRAKE;
            if(robot.leftMotor  != null) robot.leftMotor.setZeroPowerBehavior(zeroPwr);
            if(robot.rightMotor != null) robot.rightMotor.setZeroPowerBehavior(zeroPwr);
        }

        if(invert_drive_dir)
        {
            robot.invertDriveDir();
        }

        if(toggle_run_mode && robot.leftMotor != null && robot.rightMotor != null)
        {
            DcMotor.RunMode currMode = robot.leftMotor.getMode();
            if(currMode == DcMotor.RunMode.RUN_USING_ENCODER)
            {
                dtrn.setMode(robot.leftMotors, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                dtrn.setMode(robot.rightMotors, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            else
            {
                dtrn.setMode(robot.leftMotors, DcMotor.RunMode.RUN_USING_ENCODER);
                dtrn.setMode(robot.rightMotors, DcMotor.RunMode.RUN_USING_ENCODER);
            }
            RobotLog.dd(TAG, "Change mode to %s", robot.leftMotor.getMode());
        }

        if(step_driveType && robot.leftMotor != null && robot.rightMotor != null)
        {
            switch(driveType)
            {
                case TANK_DRIVE:
                    driveType = TELEOP_DRIVE_TYPE.ARCADE_DRIVE;
                    break;
                case ARCADE_DRIVE:
                    driveType = TELEOP_DRIVE_TYPE.TANK_DRIVE;
                    break;
            }
        }
    }

    //private boolean lastShift = false;

    private void processControllerInputs()
    {
        gpad2.update();
        controlArm();
    }

    private void processDriverInputs()
    {
        gpad1.update();
        controlDrive();
        controlLatch();
        controlParker();
    }

    private void doMove(Segment seg)
    {
        if(!opModeIsActive() || isStopRequested()) return;

        dtrn.setInitValues();
        dtrn.logData(true, seg.getName() + " move");
        dtrn.setDrvTuner(seg.getDrvTuner());

        dtrn.setBusyAnd(true);
        String  snm = seg.getName();
        Point2d spt = seg.getStrtPt();
        Point2d ept = seg.getTgtPt();
        double  fhd = seg.getFieldHeading();
        ShelbyBot.DriveDir dir = seg.getDir();
        double speed = seg.getSpeed();
        double fudge = seg.getDrvTuner();
        Segment.TargetType ttype = seg.getTgtType();

        RobotLog.ii(TAG, "Drive %s %s %s %6.2f %3.2f %s tune: %4.2f %s",
                snm, spt, ept, fhd, speed, dir, fudge, ttype);

        Drivetrain.Direction ddir = Drivetrain.Direction.FORWARD;

        timer.reset();

        double targetHdg = seg.getFieldHeading();
        dtrn.driveToPointLinear(ept, speed, ddir, targetHdg);

        dtrn.setCurrPt(ept);

        RobotLog.ii(TAG, "Completed move %s. Time: %6.3f HDG: %6.3f",
                seg.getName(), timer.time(), robot.getGyroFhdg());
    }

    private void doEncoderTurn(double fHdg, String prefix)
    {
        if(!opModeIsActive() || isStopRequested()) return;
        dtrn.setBusyAnd(true);
        dtrn.setInitValues();
        dtrn.logData(true, prefix);
        double cHdg = dtrn.curHdg;
        double angle = fHdg - cHdg;
        RobotLog.ii(TAG, "doEncoderTurn CHDG %6.3f THDG %6.3f", cHdg, fHdg);

        while (angle <= -180.0) angle += 360.0;
        while (angle >   180.0) angle -= 360.0;
        if(Math.abs(angle) <= 4.0) return;

        RobotLog.ii(TAG, "Turn %5.2f", angle);
        dashboard.displayPrintf(8, "STATE: %s %5.2f", "TURN", angle);
        timer.reset();
        dtrn.ctrTurnLinear(angle, 0.6, Drivetrain.TURN_BUSYTHRESH);
        cHdg = robot.getGyroFhdg();
        RobotLog.ii(TAG, "Completed turn %5.2f. Time: %6.3f CHDG: %6.3f",
                angle, timer.time(), cHdg);
    }

    private void doGyroTurn(double fHdg, String prefix)
    {
        if(!opModeIsActive() || isStopRequested()) return;

        dtrn.setInitValues();
        dtrn.logData(true, prefix);
        double cHdg = dtrn.curHdg;

        RobotLog.ii(TAG, "doGyroTurn CHDG %4.2f THDG %4.2f", cHdg, fHdg);

        if(Math.abs(fHdg-cHdg) < 1.0)
            return;

        timer.reset();
        dtrn.ctrTurnToHeading(fHdg, 0.4);

        cHdg = dtrn.curHdg;
        RobotLog.ii(TAG, "Completed turnGyro %4.2f. Time: %6.3f CHDG: %4.2f",
                fHdg, timer.time(), cHdg);
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, false, false, false);

        initPreStart();

        // Send telemetry message to signify robot waiting;
        dashboard.displayPrintf(0, "Hello Driver - I'm %s - in init", robot.getName());

        // Wait for the game to start (driver presses PLAY)
        while(!isStarted() && !isStopRequested())
        {
            printTelem();
            idle();
        }

        RobotLog.dd(TAG, "Telop_Driver starting");

        initPostStart();

        // run until the end of the match (driver presses STOP)
        ElapsedTime prntTimer = new ElapsedTime();
        prntTimer.reset();
        ElapsedTime teleTimer = new ElapsedTime();
        teleTimer.reset();

        while (opModeIsActive())
        {
            if(estimatePos)
            {
                if (robot.leftMotors.size() > 0 && robot.rightMotors.size() >0)
                    dtrn.setCurValues();
                dtrn.logData();
            }

            if(prntTimer.seconds() > 1.0)
            {
                prntTimer.reset();
                RobotLog.dd(TAG, "EstPos %s %4.3f Teletime=%4.3f",
                        dtrn.getEstPos(), robot.getGyroFhdg(),
                        teleTimer.seconds());
            }

            processDriverInputs();
            es.submit(new ArmTask());

            printTelem();

            // Pause for metronome tick.
            robot.waitForTick(10);
        }

        es.shutdownNow();
    }

    private void printTelem()
    {
        String ldir = robot.leftMotor.getDirection().toString();
        String rdir = robot.rightMotor.getDirection().toString();
        int lc = robot.leftMotor.getCurrentPosition();
        int rc = robot.rightMotor.getCurrentPosition();
        int lcnts = 0;
        if(robot._liftyBoi != null) lcnts = robot._liftyBoi.getCurrentPosition();
        int ecnts = 0;
        if(robot.armExtend != null) ecnts = robot.armExtend.getCurrentPosition();
        int acnts = 0;
        if(robot.armRotate != null) acnts = robot.armRotate.getCurrentPosition();

        dashboard.displayPrintf(0, "DMODE %s DDIR %s",driveType, robot.getDriveDir());
        dashboard.displayPrintf(1, "extcnts  %d %f", ecnts, ecnts/robot.EXTND_CPI);
        dashboard.displayPrintf(2, "elevcnts %d %f", lcnts, lcnts/robot.LIFTER_CPI);
        dashboard.displayPrintf(3, "rotPos   %d", acnts);
        dashboard.displayPrintf(4, "L_IN %4.2f LC %d %s", raw_left,  lc, ldir);
        dashboard.displayPrintf(5, "R_IN %4.2f RC %d %s", raw_right, rc, rdir);
        dashboard.displayPrintf(6, "T_IN %4.2f", raw_turn);
        dashboard.displayPrintf(7, "Z_PWR " + zeroPwr);
        if (robot.distSens != null)
        {
            dashboard.displayPrintf( 8, "ArmHt %f",
                    robot.distSens.getDistance(DistanceUnit.INCH));
        }
        //dashboard.displayPrintf(4, "rotcounts %d", robot.armRotate.getCurrentPosition());
    }

    private enum TELEOP_DRIVE_TYPE
    {
        TANK_DRIVE,
        ARCADE_DRIVE
    }

    private TELEOP_DRIVE_TYPE driveType = TELEOP_DRIVE_TYPE.ARCADE_DRIVE;

    public final double minPwr = 0.10;
    public final double minTrn = 0.10;

    private DcMotorEx lex = null;
    private DcMotorEx rex = null;

    private SkyBot robot = new SkyBot();
    private Drivetrain dtrn = new Drivetrain();
    private Input_Shaper ishaper = new Input_Shaper();

    private DcMotor.ZeroPowerBehavior zeroPwr = DcMotor.ZeroPowerBehavior.BRAKE;
    private boolean useSetVel = true;

    private enum PitchState {PITCH_UP, PITCH_DOWN }
    private PitchState currentPitchState = PitchState.PITCH_UP;

    private enum FlickerState { UP, DOWN }
    private FlickerState currentFlickerState = FlickerState.UP;

    private boolean intakeOpen = false;

    private boolean pActive = false;
    private boolean eActive = false;

    private double raw_left;
    private double raw_right;
    private double raw_turn;

    private boolean lastArmTouchPressed = false;

    @SuppressWarnings("FieldCanBeLocal")
    private boolean estimatePos = true;

    private int prevRpitch = 0;

    private ShelbyBot.OpModeType prevOpModeType = ShelbyBot.OpModeType.UNKNOWN;

    private final ExecutorService es = Executors.newSingleThreadExecutor();

    private ElapsedTime timer = new ElapsedTime();

    private static final String TAG = "SJH_TD";
}

