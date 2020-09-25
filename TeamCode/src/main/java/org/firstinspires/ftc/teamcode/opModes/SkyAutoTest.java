package org.firstinspires.ftc.teamcode.opModes;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.PositionOption;
import org.firstinspires.ftc.teamcode.field.Route;
import org.firstinspires.ftc.teamcode.field.SkyField;
import org.firstinspires.ftc.teamcode.field.SkyRoute;
import org.firstinspires.ftc.teamcode.image.Detector;
import org.firstinspires.ftc.teamcode.image.ImageTracker;
import org.firstinspires.ftc.teamcode.image.StoneDetector;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.robot.SkyBot;
import org.firstinspires.ftc.teamcode.robot.TilerunnerGtoBot;
import org.firstinspires.ftc.teamcode.robot.TilerunnerMecanumBot;
import org.firstinspires.ftc.teamcode.util.AutoTransitioner;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Segment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;

import static org.firstinspires.ftc.teamcode.field.Route.ParkPos.DEFEND_PARK;

//If start at 2nd left tile (centered on block 5):
//  a set gyro/field hdg init bot/pts
//  b move forward to scan pt
//  c can - find SkyStone (A L/C/R)
//  d move bot to turnpt and turn to align side with stones
//  e if moving bot to L/R, move fwd/back - else turn arm
//  f grab stone
//  g deliver/drop stone
//  h if doing 2 (default) cycle back to B L/C/R - repeat e-g
//  i if moving platform, do so
//  j park under bridge


@Autonomous(name="SkyAutoTest", group="Auton")
//@Disabled
public class SkyAutoTest extends InitLinearOpMode implements FtcMenu.MenuButtons
{
    public SkyAutoTest()
    {
        //super();
    }

    private void startMode()
    {
        dashboard.clearDisplay();
        drvTrn.start();
        do_main_loop();
    }

    //@SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() //throws InterruptedException
    {
        RobotLog.dd(TAG, "initCommon");
        initCommon(this, true, true, false, false);

        RobotLog.dd(TAG, "getBotName");
        robotName = pmgr.getBotName();

        RobotLog.dd(TAG, "logPrefs");
        pmgr.logPrefs();

        alliance = Field.Alliance.valueOf(pmgr.getAllianceColor());
        startPos = Route.StartPos.valueOf(pmgr.getStartPosition());
        try
        {
            parkPos  = Route.ParkPos.valueOf(pmgr.getParkPosition());
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ParkPosition %s invalid.", pmgr.getParkPosition());
            parkPos = Route.ParkPos.CENTER_PARK;
        }

        delay    = pmgr.getDelay();

        dashboard.displayPrintf(2, "Pref BOT: %s", robotName);
        dashboard.displayPrintf(3, "Pref Alliance: %s", alliance);
        dashboard.displayPrintf(4, "Pref StartPos: %s %s", startPos, parkPos);
        dashboard.displayPrintf(5, "Pref Delay: %.2f", delay);

        setup();
        int initCycle = 0;
        int initSleep = 10;
        timer.reset();
        while(!isStarted())
        {
            if(initCycle % 10 == 0)
            {
                double shdg = robot.getGyroHdg();
                double fhdg = robot.getGyroFhdg();
                dashboard.displayPrintf(0, "HDG %4.2f FHDG %4.2f", shdg, fhdg);
                dashboard.displayPrintf(10, "GyroReady %s", gyroReady);
                dashboard.displayPrintf(11, "RGyroReady %s", robot.gyroReady);
                if (robot.leftMotor != null)
                    dashboard.displayPrintf(12, "LENC %d", robot.leftMotor.getCurrentPosition());
                if (robot.leftMotor != null)
                    dashboard.displayPrintf(13, "RENC %d", robot.rightMotor.getCurrentPosition());
                if (skyBot._liftyBoi != null)
                    dashboard.displayPrintf(14, "ELEV %d", skyBot._liftyBoi.getCurrentPosition());
                if (skyBot.armExtend != null)
                    dashboard.displayPrintf(15, "XTND %d", skyBot.armExtend.getCurrentPosition());

                if(skyBot.armRotate != null)
                    dashboard.displayPrintf(16, "armRot good");

                if (robot.colorSensor != null)
                {
                    int r = robot.colorSensor.red();
                    int g = robot.colorSensor.green();
                    int b = robot.colorSensor.blue();
                    RobotLog.dd(TAG, "RGB = %d %d %d", r, g, b);
                    dashboard.displayPrintf(17, "RGB %d %d %d", r, g, b);
                }
            }

            initCycle++;

            sleep(initSleep);
        }
        startMode();
        stopMode();
    }

    private void stopMode()
    {
        if(drvTrn != null) drvTrn.cleanup();
        if(tracker != null) {
            tracker.setFrameQueueSize(0);
            tracker.setActive(false);
        }
        if(det != null) det.cleanupCamera();
    }

    private void setup()
    {
        dashboard.displayPrintf(0, "PLEASE WAIT - STARTING - CHECK DEFAULTS");
        logData = true;

        dashboard.displayPrintf(6, "HIT A TO ACCEPT VALUES");
        dashboard.displayPrintf(7, "HIT B FOR MENU");
        RobotLog.ii(TAG, "SETUP");

        ElapsedTime mTimer = new ElapsedTime();
        boolean doMen = false;
        while(mTimer.seconds() < 5.0)
        {
            gpad1.update();
            if(gpad1.just_pressed(ManagedGamepad.Button.A))
            {
                break;
            }
            if(gpad1.just_pressed(ManagedGamepad.Button.B))
            {
                doMen = true;
                break;
            }
        }

        if(doMen) doMenus();

        dashboard.displayPrintf(0, "INITIALIZING");

        String teleopName = "TeleopDriver";

        if(robotName.equals("MEC"))
        {
            robot = new TilerunnerMecanumBot();
            teleopName = "Mecanum";
        }
        else
        {
            robot = new SkyBot(robotName);
            skyBot = (SkyBot)robot;
        }

        dashboard.displayPrintf(1, "Prefs Done");

        //Since we only have 5 seconds between Auton and Teleop, automatically load
        //teleop opmode
        RobotLog.dd(TAG, "Setting up auto tele loader : %s", teleopName);
        AutoTransitioner.transitionOnStop(this, teleopName);

        dashboard.displayPrintf(1, "AutoTrans setup");

        //Add wait for gamepad button press to indicate start of gyro init
        //Drive team aligns bot with field
        //Drive team hits another button to lock in gyro init

        dashboard.displayPrintf(6, "ALIGN TO FIELD THEN HIT X TO START GYRO INIT");
        dashboard.displayPrintf(7, "OR ALIGN TO FIRST SEG THEN HIT Y TO SKIP");
        ElapsedTime gyroTimer = new ElapsedTime();
        boolean gyroSetToField = false;
        boolean gyroSetToSeg   = false;
        while(gyroTimer.seconds() < 10.0)
        {
            gpad1.update();
            if(gpad1.just_pressed(ManagedGamepad.Button.X))
            {
                gyroSetToField = true;
                break;
            }
            if(gpad1.just_pressed(ManagedGamepad.Button.Y))
            {
                gyroSetToSeg = true;
                break;
            }
        }

        if(gyroSetToField)
            RobotLog.ii(TAG,"X pressed: gyroSetToField");
        if(gyroSetToSeg)
            RobotLog.ii(TAG,"Y pressed: gyroSetToSeg");

        dashboard.displayPrintf(0, "GYRO CALIBRATING DO NOT TOUCH OR START");
        SkyBot.curOpModeType = ShelbyBot.OpModeType.AUTO;
        skyBot.init(this, robotName);

        if (robot.imu != null || robot.gyro  != null)
        {
            gyroReady = robot.calibrateGyro();
        }
        dashboard.displayPrintf(0, "GYRO CALIBATED: %s", gyroReady);

        dashboard.displayPrintf(6, gyroSetToField ? "Field" : "1stSeg" +"Init done");
        dashboard.displayPrintf(7, "");

        robot.setAlliance(alliance);

        dashboard.displayPrintf(1, "Robot Inited");

        RobotLog.dd(TAG, "Robot CPI " + robot.CPI);

        drvTrn.init(robot);
        drvTrn.setRampUp(false);
        int colThresh = 450;
        if(robotName.equals("GTO1")) colThresh = 3000;
        drvTrn.setColorThresh(colThresh);

        dashboard.displayPrintf(1, "DrvTrn Inited");

        det = new StoneDetector(robotName);
        RobotLog.dd(TAG, "Setting up vuforia");
        tracker = new ImageTracker(new SkyField());

        setupLogger();

        dl.addField("Start: " + startPos.toString());
        dl.addField("Alliance: " + alliance.toString());
        dl.addField("Park: "     + parkPos.toString());
        RobotLog.ii(TAG, "STARTPOS %s", startPos);
        RobotLog.ii(TAG, "ALLIANCE %s", alliance);
        RobotLog.ii(TAG, "PARKPOS %s", parkPos);
        RobotLog.ii(TAG, "DELAY    %4.2f", delay);
        RobotLog.ii(TAG, "BOT      %s", robotName);

        Route pts = new SkyRoute(startPos, alliance, robotName);

        pathSegs.addAll(Arrays.asList(pts.getSegments()));

        initHdg = pathSegs.get(0).getFieldHeading();

        ShelbyBot.DriveDir startDdir = pathSegs.get(0).getDir();
        robot.setDriveDir(startDdir);

        RobotLog.dd(TAG, "START DRIVEDIR =%s", startDdir);

        RobotLog.ii(TAG, "ROUTE: \n" + pts.toString());

        Point2d currPoint = pathSegs.get(0).getStrtPt();
        drvTrn.setCurrPt(currPoint);

        drvTrn.setStartHdg(gyroSetToField ? 0 : initHdg);
        robot.setInitHdg(gyroSetToField   ? 0 : initHdg);

        RobotLog.ii(TAG, "Start %s.", currPoint);
        dashboard.displayPrintf(8, "PATH: Start at %s", currPoint);

        RobotLog.ii(TAG, "IHDG %4.2f", initHdg);

        skyBot.putHolderAtStow();
        skyBot.stowGripper();
        skyBot.zeroLift();
        skyBot.zeroArmRotate();
        skyBot.zeroArmExtend();

        det.setTelemetry(telemetry);
    }

    private void do_main_loop()
    {
        timer.reset();
        startTimer.reset();
        dl.resetTime();

        RobotLog.ii(TAG, "STARTING AT %4.2f", timer.seconds());
        if(logData)
        {
            Point2d spt = pathSegs.get(0).getStrtPt();
            dl.addField("START");
            dl.addField(initHdg);
            dl.addField(spt.getX());
            dl.addField(spt.getY());
            dl.newLine();
        }

        RobotLog.ii(TAG, "Delaying for %4.2f seconds", delay);
        ElapsedTime delayTimer = new ElapsedTime();
        while (opModeIsActive() && delayTimer.seconds() < delay)
        {
            idle();
        }

        RobotLog.ii(TAG, "Done delay");

        RobotLog.ii(TAG, "START CHDG %6.3f", robot.getGyroHdg());

        //robot.resetGyro();

        //doFindLoc();

        boolean SkipNextSegment = false;

        if(!robotName.equals("MEC"))
        {
            drvTrn.setRampSpdL(0.15);
            drvTrn.setRampSpdM(0.35);
            drvTrn.setRampSpdH(0.60);
            drvTrn.setRampCntH(600);
            drvTrn.setRampCntM(300);
            drvTrn.setRampCntL(100);
        }

        ElapsedTime mTimer = new ElapsedTime();
        while(opModeIsActive())
        {
            gpad1.update();
            gpad2.update();
            if(gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT))
            {
                skyBot.putArmRight();
            }
            if(gpad1.just_pressed(ManagedGamepad.Button.D_LEFT))
            {
                skyBot.putArmLeft();
            }
            if(gpad1.just_pressed(ManagedGamepad.Button.D_UP))
            {
                skyBot.putArmForward();
            }
            if(gpad1.just_pressed(ManagedGamepad.Button.D_DOWN))
            {
                if(skyBot.armRotate == null) return;
                int curPos = skyBot.armRotate.getCurrentPosition();
                if(curPos == skyBot.ARM_ROT_LFT)
                    skyBot.putArmHalfLeft();
                else if(curPos == skyBot.ARM_DRP_LFT)
                    skyBot.putArmForward();
                else if(curPos == skyBot.ARM_ROT_FWD)
                    skyBot.putArmHalfRight();
                else if(curPos == skyBot.ARM_DRP_RGT)
                    skyBot.putArmRight();
                else if(curPos == skyBot.ARM_ROT_RGT)
                    skyBot.putArmLeft();
            }

            if(gpad1.just_pressed(ManagedGamepad.Button.L_BUMP))
                skyBot.putHolderAtPre();
            if(gpad1.just_pressed(ManagedGamepad.Button.L_TRIGGER))
                skyBot.putHolderAtGrab();

            if(gpad1.just_pressed(ManagedGamepad.Button.R_BUMP))
                skyBot.openGripper();
            if(gpad1.just_pressed(ManagedGamepad.Button.R_TRIGGER))
                skyBot.closeGripper();

            if(gpad1.just_pressed(ManagedGamepad.Button.Y))
            {
                skyBot.putHolderAtStow();
                skyBot.stageGripper();
            }

            if(gpad1.just_pressed(ManagedGamepad.Button.A))
                skyBot.putExtendAtStow();
            if(gpad1.just_pressed(ManagedGamepad.Button.B))
                skyBot.putExtendAtStage();
            if(gpad1.just_pressed(ManagedGamepad.Button.X))
                skyBot.putExtendAtGrab();

            double  aelev       = -gpad1.value(ManagedGamepad.AnalogInput.L_STICK_Y);
            double  aelev1      = gpad1.value(ManagedGamepad.AnalogInput.L_STICK_X);
            if(aelev > 0.5) skyBot.putLiftAtStow();
            else if(aelev < -0.5) skyBot.putLiftAtGrab();
            else if(aelev1 > 0.5) skyBot.putLiftAtMove();

            if(gpad2.just_pressed(ManagedGamepad.Button.D_UP))
            {
                skyBot.putLiftAtStow();
            }
            if(gpad2.just_pressed(ManagedGamepad.Button.D_DOWN))
            {
                skyBot.putLiftAtGrab();
            }
            if(gpad2.just_pressed(ManagedGamepad.Button.D_RIGHT))
            {
                skyBot.putLiftAtMove();
            }
            if(gpad2.just_pressed(ManagedGamepad.Button.D_LEFT))
            {
                skyBot.putLiftAtDrop();
            }
            if(gpad2.just_pressed(ManagedGamepad.Button.A))
            {
                skyBot.putLiftAtLevel2();
            }
            idle();
        }

        RobotLog.dd(TAG, "Finished auton segments");
        robot.setAutonEndHdg(robot.getGyroFhdg());
        //robot.setAutonEndPos(drvTrn.getCurrPt());
        robot.setAutonEndPos(drvTrn.getEstPos());

        while(opModeIsActive() && !isStopRequested())
        {
            idle();
        }
    }


    @SuppressWarnings("unused")
    private void doFindLoc()
    {
        //Try to use Vuf localization to find loc
        //Turn to NSEW depending on startpos to sens loc
        tracker.setActive(true);
        Point2d sensedPos = null;
        Double  sensedHdg = null;
        String sensedImg = null;
        ElapsedTime imgTimer = new ElapsedTime();

        RobotLog.dd(TAG, "doFindLoc");

        while(opModeIsActive()         &&
              imgTimer.seconds() < 1.0 &&
              sensedPos == null)
        {
            tracker.updateRobotLocationInfo();
            sensedPos = tracker.getSensedPosition();
            sensedHdg = tracker.getSensedFldHeading();
            sensedImg = tracker.getLastVisName();
        }

        if(sensedPos != null) RobotLog.dd(TAG, "SENSED POS " + sensedImg + " " + sensedPos);
        if(sensedPos != null) RobotLog.dd(TAG, "SENSED HDG " + sensedImg + " " + sensedHdg);

        tracker.setActive(false);
    }

    private void doScan(int segIdx)
    {
        RobotLog.dd(TAG, "doScan");

        if(useLight)
            CameraDevice.getInstance().setFlashTorchMode(true) ;

        stonePos =  getStonePos();
        RobotLog.dd(TAG, "doScan stonePos = %s", stonePos);

        if(useLight)
            CameraDevice.getInstance().setFlashTorchMode(false);

        setStonePoint(segIdx);
    }

    private void doGrab(int segIdx)
    {
        double hdgAdj = 14.0;

        Point2d curPt = pathSegs.get(segIdx).getTgtPt();
        double curAng = pathSegs.get(segIdx).angle();
//        double newHdg = curAng - hdgAdj;
        double newHdg = robot.getGyroFhdg() - hdgAdj;
        //TODO:  Uncomment if we are turning bot to grab stone.
        //       Otherwise, turn arm
//        doGyroTurn(newHdg, "scanAdj");

        //Rotate arm for red/blue

        sleep(2000);
        if(alliance == Field.Alliance.BLUE)
        {
            skyBot.putArmRight();
        }
        else
        {
            skyBot.putArmLeft();
        }
        //Extend arm so fixed gripper is clear of side of bot
        skyBot.putExtendAtStage();
        //Lower arm and open gripper
        skyBot.putLiftAtGrab();
        skyBot.openGripper();
        //Extend arm to grab state
        skyBot.putExtendAtGrab();
        //Close gripper
        skyBot.closeGripper();
        //Raise arm
        skyBot.putLiftAtLevel2();
        //Rotate arm to forward and possibly retract some
        skyBot.putArmForward();
        skyBot.putExtendAtStage();
    }

    private void setStonePoint(int segIdx)
    {
        RobotLog.dd(TAG, "Getting stonePt for %s %s %s seg=%d",
                alliance, startPos, stonePos, segIdx);
        //TODO: If we are going to move bot we need to determine pts for L/R blocks
        //      TgtPt could be stoneGrabPt[center].x +/- 8in.
        //      If we are simply going to rotate grabber arm, no need.
//        Segment sMin = pathSegs.get(segIdx+1);
//        Segment sRev = pathSegs.get(segIdx+2);
//        sMin.setEndPt(tgtPt1);
//        sRev.setStrtPt(tgtPt1);
    }

    @SuppressWarnings("unused")
    private void doDrop(int segIdx)
    {
        RobotLog.dd(TAG, "Dropping stone");
        //rotate arm, lower?, release gripper, return arm
        sleep(1000);
        if(alliance == Field.Alliance.BLUE)
        {
            skyBot.putArmHalfRight();
        }
        else
        {
            skyBot.putArmHalfLeft();

        }
        //skyBot.putLiftAtDrop();
        skyBot.openGripper();
        skyBot.putLiftAtGrab();
        skyBot.putArmForward();
    }

    private void doPlatch()
    {
        sleep(1000);
        RobotLog.dd(TAG, "Platching platform");
        if(skyBot != null)
        {
            skyBot.putHolderAtGrab();
        }
        sleep(1000);
    }

    private void doUnPlatch()
    {
        RobotLog.dd(TAG, "UnPlatching platform");
        if(skyBot != null)
        {
            skyBot.putHolderAtStow();
        }
    }

    private void doPark()
    {
        RobotLog.dd(TAG, "Parking bot");
        if(skyBot != null)
        {
            int hoverCounts = -5000; //-(int)(20 * robot.ARM_CPD);

            RobotLog.dd(TAG, "Moving to arm pos %d from %d", hoverCounts, 0);
            //extend arm if needed
        }
        else
        {
            RobotLog.dd(TAG, "No skyRobot - can't extend arm");
        }
    }

    private void doMove(Segment seg)
    {
        if(!opModeIsActive() || isStopRequested()) return;

        drvTrn.setInitValues();
        RobotLog.ii(TAG, "Setting drive tuner to %4.2f", seg.getDrvTuner());
        drvTrn.logData(true, seg.getName() + " move");
        drvTrn.setDrvTuner(seg.getDrvTuner());

        drvTrn.setBusyAnd(true);
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

        dashboard.displayPrintf(2, "STATE: %s %s %s - %s %6.2f %3.2f %s",
                "DRIVE", snm, spt, ept, fhd, speed, dir);

        Drivetrain.Direction ddir = Drivetrain.Direction.FORWARD;

        timer.reset();

        if(seg.getTgtType() == Segment.TargetType.COLOR)
        {
            RobotLog.dd(TAG, "colorSensor is %s", robot.colorSensor == null ? "null" : "good");
        }

        if(robot.colorSensor != null && seg.getTgtType() == Segment.TargetType.COLOR)
        {
            RobotLog.dd(TAG,"Doing color seg %d", colSegNum);
            colSegNum++;
            int colSensOffset = drvTrn.distanceToCounts(3.0);
            drvTrn.setColSensOffset(colSensOffset);
            drvTrn.setInitValues();

            double fullSegLen = seg.getLength();

            RobotLog.dd(TAG, "SBH calling driveDistanceLinear %4.2f %4.2f %s %4.2f %s",
            fullSegLen, speed, ddir, fhd, "true");
            drvTrn.driveDistanceLinear(fullSegLen, speed, ddir, fhd, true);
            //Possibly do Vuf scan here to get localization
        }
        else
        {
            double targetHdg = seg.getFieldHeading();
            drvTrn.driveToPointLinear(ept, speed, ddir, targetHdg);
        }

        drvTrn.setCurrPt(ept);

        RobotLog.ii(TAG, "Completed move %s. Time: %6.3f HDG: %6.3f",
                seg.getName(), timer.time(), robot.getGyroFhdg());
    }


    private void doEncoderTurn(double fHdg, @SuppressWarnings("SameParameterValue") int thresh, String prefix)
    {
        if(!opModeIsActive() || isStopRequested()) return;
        drvTrn.setBusyAnd(true);
        drvTrn.setInitValues();
        drvTrn.logData(true, prefix);
        double cHdg = drvTrn.curHdg;
        double angle = fHdg - cHdg;
        RobotLog.ii(TAG, "doEncoderTurn CHDG %6.3f THDG %6.3f", cHdg, fHdg);

        while (angle <= -180.0) angle += 360.0;
        while (angle >   180.0) angle -= 360.0;
        if(Math.abs(angle) <= 4.0) return;

        RobotLog.ii(TAG, "Turn %5.2f", angle);
        dashboard.displayPrintf(2, "STATE: %s %5.2f", "TURN", angle);
        timer.reset();
        drvTrn.ctrTurnLinear(angle, DEF_ENCTRN_PWR, thresh);
        cHdg = robot.getGyroFhdg();
        RobotLog.ii(TAG, "Completed turn %5.2f. Time: %6.3f CHDG: %6.3f",
                angle, timer.time(), cHdg);
    }

    private void doEncoderTurn(double fHdg, String prefix)
    {
        doEncoderTurn(fHdg, Drivetrain.TURN_BUSYTHRESH, prefix);
    }

    private void doGyroTurn(double fHdg, String prefix)
    {
        if(!gyroReady) return;
        if(!opModeIsActive() || isStopRequested()) return;

        drvTrn.setInitValues();
        drvTrn.logData(true, prefix);
        double cHdg = drvTrn.curHdg;

        RobotLog.ii(TAG, "doGyroTurn CHDG %4.2f THDG %4.2f", cHdg, fHdg);

        if(Math.abs(fHdg-cHdg) < 1.0)
            return;

        timer.reset();
        drvTrn.ctrTurnToHeading(fHdg, DEF_GYRTRN_PWR);

        cHdg = drvTrn.curHdg;
        RobotLog.ii(TAG, "Completed turnGyro %4.2f. Time: %6.3f CHDG: %4.2f",
                fHdg, timer.time(), cHdg);
    }

    private StoneDetector.Position getStonePos()
    {
        if(!opModeIsActive() || stonePos != StoneDetector.Position.NONE)
            return stonePos;

        tracker.setActive(true);
        stonePos = StoneDetector.Position.NONE;
        RobotLog.dd(TAG, "Set qsize to get frames");
        tracker.setFrameQueueSize(1);
        RobotLog.dd(TAG, "Start LD sensing");
        det.startSensing();

        StoneDetector.Position stonePos = StoneDetector.Position.NONE;

        double stoneTimeout = 0.5;

        ElapsedTime mtimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(opModeIsActive()                                   &&
              stonePos == StoneDetector.Position.NONE &&
              mtimer.seconds() < stoneTimeout)
        {
            tracker.updateImages();
            Bitmap rgbImage = tracker.getLastImage();

            boolean tempTest = false;
            if(rgbImage == null)
            {
                RobotLog.dd(TAG, "getStonePos - image from tracker is null");
                //noinspection ConstantConditions
                if(!tempTest) continue;
            }
            det.setBitmap(rgbImage);
            det.logDebug();
            det.logTelemetry();
            if(det instanceof StoneDetector)
                stonePos = ((StoneDetector) det).getStonePos();

            if(stonePos == StoneDetector.Position.NONE)
                sleep(10);
        }

        det.stopSensing();
        tracker.setFrameQueueSize(0);
        tracker.setActive(false);

        dashboard.displayPrintf(1, "MIN: " + stonePos);

        if (stonePos == StoneDetector.Position.NONE)
        {
            RobotLog.dd(TAG, "No skystone found - defaulting to center");
            stonePos = StoneDetector.Position.CENTER;
        }
        return stonePos;
    }

    @Override
    public boolean isMenuUpButton() { return gamepad1.dpad_up;} //isMenuUpButton

    @Override
    public boolean isMenuAltUpButton()
    {
        return gamepad1.left_bumper;
    }

    @Override
    public boolean isMenuDownButton()
    {
        return gamepad1.dpad_down;
    } //isMenuDownButton

    @Override
    public boolean isMenuAltDownButton()
    {
        return gamepad1.right_bumper;
    }

    @Override
    public boolean isMenuEnterButton()
    {
        return gamepad1.a;
    } //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return gamepad1.dpad_left;
    }  //isMenuBackButton

    private void doMenus()
    {
        FtcChoiceMenu<PositionOption> startPosMenu =
                new FtcChoiceMenu<>("START:", null, this);
        FtcChoiceMenu<Field.Alliance> allianceMenu =
                new FtcChoiceMenu<>("ALLIANCE:", startPosMenu, this);
        FtcChoiceMenu<String> robotNameMenu =
                new FtcChoiceMenu<>("BOT_NAME:", allianceMenu, this);
        FtcChoiceMenu<PositionOption> parkMenu
                = new FtcChoiceMenu<>("Park:",   robotNameMenu, this);
        FtcValueMenu delayMenu
                = new FtcValueMenu("DELAY:", parkMenu, this,
                0.0, 20.0, 1.0, 0.0, "%5.2f");

        startPosMenu.addChoice("Start_1", Route.StartPos.START_1, true, allianceMenu);
        startPosMenu.addChoice("Start_2", Route.StartPos.START_2, false, allianceMenu);

        allianceMenu.addChoice("RED",  Field.Alliance.RED,  true, parkMenu);
        allianceMenu.addChoice("BLUE", Field.Alliance.BLUE, false, parkMenu);

        parkMenu.addChoice("CENTER_PARK", Route.ParkPos.CENTER_PARK, true, robotNameMenu);
        parkMenu.addChoice("DEFEND_PARK", DEFEND_PARK, false, robotNameMenu);

        robotNameMenu.addChoice("GTO1", "GTO1", true, delayMenu);
        robotNameMenu.addChoice("GTO2", "GTO2", false, delayMenu);
        robotNameMenu.addChoice("MEC", "MEC", false, delayMenu);

        FtcMenu.walkMenuTree(startPosMenu, this);

        startPos  = startPosMenu.getCurrentChoiceObject();
        alliance  = allianceMenu.getCurrentChoiceObject();
        robotName = robotNameMenu.getCurrentChoiceObject();
        parkPos   = parkMenu.getCurrentChoiceObject();
        delay     = delayMenu.getCurrentValue();

        int lnum = 2;
        dashboard.displayPrintf(lnum++, "NAME: %s", robotName);
        dashboard.displayPrintf(lnum++, "ALLIANCE: %s", alliance);
        dashboard.displayPrintf(lnum++, "START: %s", startPos);
        //noinspection UnusedAssignment
        dashboard.displayPrintf(lnum++, "Pref Delay: %.2f", delay);
    }

    private void setupLogger()
    {
        if (logData)
        {
            dl.addField("NOTE");
            dl.addField("FRAME");
            dl.addField("Gyro");
            dl.addField("LENC");
            dl.addField("RENC");
            dl.addField("LPWR");
            dl.addField("RPWR");
            dl.addField("RED");
            dl.addField("GRN");
            dl.addField("BLU");
            dl.addField("ESTX");
            dl.addField("ESTY");
            dl.addField("ESTH");
            dl.newLine();
        }
    }

    private final static double DEF_ENCTRN_PWR  = 0.6;
    private final static double DEF_GYRTRN_PWR = 0.4;

    private List<Segment> pathSegs = new ArrayList<>();

    private TilerunnerGtoBot   robot;
    private SkyBot skyBot = null;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime startTimer = new ElapsedTime();
    private Drivetrain drvTrn = new Drivetrain();

    private Detector det;
    private static ImageTracker tracker;
    private StoneDetector.Position stonePos = StoneDetector.Position.NONE;

    private static Point2d curPos;
    private double initHdg = 0.0;
    private boolean gyroReady;
    @SuppressWarnings("FieldCanBeLocal")
    private boolean usePostTurn = true;

    private static PositionOption startPos = Route.StartPos.START_1;
    private static Field.Alliance alliance = Field.Alliance.RED;

    private static PositionOption parkPos = Route.ParkPos.CENTER_PARK;
//
//    private int RED_THRESH = 15;
//    private int GRN_THRESH = 15;
//    private int BLU_THRESH = 15;
//    @SuppressWarnings("FieldCanBeLocal")
//    private int COLOR_THRESH = 20;

    private double delay = 0.0;

    @SuppressWarnings("FieldCanBeLocal")
    private boolean useImageLoc  = false;

    private int colSegNum = 0;

    @SuppressWarnings("FieldCanBeLocal")
    private boolean useLight = false;

    private String robotName = "";
    private static final String TAG = "SJH_RRA";
}