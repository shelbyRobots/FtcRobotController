package org.firstinspires.ftc.teamcode.opModes;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.PositionOption;
import org.firstinspires.ftc.teamcode.field.RoRuField;
import org.firstinspires.ftc.teamcode.field.RoRuRoute;
import org.firstinspires.ftc.teamcode.field.Route;
import org.firstinspires.ftc.teamcode.image.Detector;
import org.firstinspires.ftc.teamcode.image.ImageTracker;
import org.firstinspires.ftc.teamcode.image.MineralDetector;
import org.firstinspires.ftc.teamcode.image.VuforiaInitializer;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.RoRuBot;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
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

//After starting on latch, opmode needs to lower/detach
// Then
//  - align (possibly with tape
//  - find gold mineral
//  - move gold mineral
//  - optionally knock partners gold mineral
//  - place marker in depot
//  - park at pit

@SuppressWarnings({"unused", "ForLoopReplaceableByForEach"})
@Autonomous(name="RoRuAutoShelby", group="Auton")
//@Disabled
public class RoRuAutoShelby extends InitLinearOpMode implements FtcMenu.MenuButtons
{
    public RoRuAutoShelby()
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
    public void runOpMode() throws InterruptedException
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
        double scanFreqSec = 3.0;
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
                if (robot.elevMotor != null)
                    dashboard.displayPrintf(14, "ELEV %d", robot.elevMotor.getCurrentPosition());

                if (robot.colorSensor != null)
                {
                    int r = robot.colorSensor.red();
                    int g = robot.colorSensor.green();
                    int b = robot.colorSensor.blue();
                    RobotLog.dd(TAG, "RGB = %d %d %d", r, g, b);
                    dashboard.displayPrintf(15, "RGB %d %d %d", r, g, b);
                }

                dashboard.displayPrintf(7, "lift pos %d", roRuBot.getLiftyPos());
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
            robot = new RoRuBot(robotName);
            roRuBot = (RoRuBot)robot;
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
        boolean botInit = false;
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

        dashboard.displayPrintf(0, "GYRO CALIBRATING DO NOT TOUCH OR START");
        RoRuBot.curOpModeType = ShelbyBot.OpModeType.AUTO;
        robot.init(this, robotName);

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

        det = new MineralDetector(robotName);
        RobotLog.dd(TAG, "Setting up vuforia");
        tracker = new ImageTracker(new RoRuField());

        setupLogger();

        dl.addField("Start: " + startPos.toString());
        dl.addField("Alliance: " + alliance.toString());
        dl.addField("Park: "     + parkPos.toString());
        RobotLog.ii(TAG, "STARTPOS %s", startPos);
        RobotLog.ii(TAG, "ALLIANCE %s", alliance);
        RobotLog.ii(TAG, "PARKPOS %s", parkPos);
        RobotLog.ii(TAG, "DELAY    %4.2f", delay);
        RobotLog.ii(TAG, "BOT      %s", robotName);

        Route pts = new RoRuRoute(startPos, alliance, robotName);
        //noinspection ConstantConditions
        if(pts instanceof RoRuRoute && parkPos == DEFEND_PARK)
        {
            RobotLog.dd(TAG, "Setting up to go for two");
            ((RoRuRoute) pts).setGoForTwo(true);
        }

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

        if(roRuBot != null)
        {
//            ElapsedTime liftAdjTimer = new ElapsedTime();
//            while(liftAdjTimer.seconds() < 10.0)
//            {
//                gpad2.update();
//                if(gpad2.just_pressed(ManagedGamepad.Button.D_UP)) roRuBot.moveHolder(1.0);
//                if(gpad2.just_pressed(ManagedGamepad.Button.D_DOWN)) roRuBot.moveHolder(-1.0);
//                if(gpad2.just_pressed(ManagedGamepad.Button.A)) break;
//                dashboard.displayPrintf(9, "LIFTYBOI ADJUST");
//            }

            roRuBot.zeroHolder();

            RobotLog.dd(TAG, "Stow Liftyboi");
            //roRuBot.putHolderAtStow();
            RobotLog.dd(TAG, "Stow Marker");
            roRuBot.stowMarker();
            roRuBot.stowParker();

            roRuBot.zeroArmPitch();
            roRuBot.zeroArmExtend();
            roRuBot.armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            roRuBot.armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (startPos == Route.StartPos.START_1)
        {
            pitScan = true;
            ctrScan = false;
        }
        else
        {
            pitScan = false;
            ctrScan = true;
        }

        if(det instanceof MineralDetector)
        {
            ((MineralDetector) det).setCenterScan(ctrScan);
            ((MineralDetector) det).setPitScan(pitScan);
        }

        det.setTelemetry(telemetry);
    }

    private void do_main_loop()
    {
        timer.reset();
        startTimer.reset();
        dl.resetTime();

        int dropCycle = 0;

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

        doLower();
        //doFindLoc();

        boolean SkipNextSegment = false;
        boolean breakOut = false;

        if(!robotName.equals("MEC"))
        {
            drvTrn.setRampSpdL(0.15);
            drvTrn.setRampSpdM(0.35);
            drvTrn.setRampSpdH(0.60);
            drvTrn.setRampCntH(600);
            drvTrn.setRampCntM(300);
            drvTrn.setRampCntL(100);
        }


        for (int i = 0; i < pathSegs.size(); ++i)
        {
            if (!opModeIsActive() || isStopRequested()) break;

            Segment prntSeg = pathSegs.get(i);
            String segName = prntSeg.getName();
            RobotLog.ii(TAG, "Starting segment %s at %4.2f", segName,
                    startTimer.seconds());
            RobotLog.ii(TAG, prntSeg.toString());

            //noinspection ConstantConditions
            if (SkipNextSegment)
            {
                SkipNextSegment = false;
                RobotLog.ii(TAG, "Skipping segment %s", pathSegs.get(i).getName());
                if (i < pathSegs.size() - 1)
                {
                    RobotLog.ii(TAG, "Setting segment %s start pt to %s",
                            pathSegs.get(i + 1).getName(),
                            pathSegs.get(i).getStrtPt());
                    pathSegs.get(i + 1).setStrtPt(pathSegs.get(i).getStrtPt());
                }
                continue;
            }

            Segment curSeg;

            if (curPos == null || !useImageLoc)
            {
                curSeg = pathSegs.get(i);
            } else
            {
                drvTrn.setCurrPt(curPos);
                curSeg = new Segment("CURSEG", curPos, pathSegs.get(i).getTgtPt());
            }
            curPos = null;

            robot.setDriveDir(curSeg.getDir());

            drvTrn.setInitValues();
            String segLogStr = String.format(Locale.US, "%s - %s H: %4.1f",
                    curSeg.getStrtPt().toString(),
                    curSeg.getTgtPt().toString(),
                    curSeg.getFieldHeading());
            drvTrn.logData(true, segName + " " + segLogStr);

            if (curSeg.getLength() >= 0.1)
            {
                RobotLog.ii(TAG, "ENCODER TURN %s t=%6.4f", curSeg.getName(),
                        startTimer.seconds());
                doEncoderTurn(curSeg.getFieldHeading(), segName + " encoderTurn"); //quick but rough
                RobotLog.ii(TAG, "GYRO TURN %s t=%6.4f", curSeg.getName(),
                        startTimer.seconds());
                doGyroTurn(curSeg.getFieldHeading(), segName + " gyroTurn");


                RobotLog.ii(TAG, "MOVE %s t=%6.4f", curSeg.getName(),
                        startTimer.seconds());
                doMove(curSeg);
            }


            Double pturn = curSeg.getPostTurn();
            if (usePostTurn && pturn != null)
            {
                RobotLog.ii(TAG, "ENCODER POST TURN %s", curSeg.getName());
                doEncoderTurn(pturn, segName + " postEncoderTurn");

//                RobotLog.ii(TAG, "GRYO POST TURN %s", curSeg.getName());
//                doGyroTurn(pturn, segName + " postGyroTurn");
            }

            if (!opModeIsActive() || isStopRequested())
            {
                drvTrn.stopMotion();
                break;
            }

            RobotLog.ii(TAG, "Planned pos: %s %s",
                    pathSegs.get(i).getTgtPt(),
                    pathSegs.get(i).getFieldHeading());


            Segment.Action act = curSeg.getAction();

            RobotLog.ii(TAG, "ACTION %s %s t=%6.4f", curSeg.getName(), act,
                    startTimer.seconds());

            if (act != Segment.Action.NOTHING)
            {
                drvTrn.setInitValues();
                drvTrn.logData(true, segName + " action " + act.toString());
            }

            switch (act)
            {
                case SET_ALIGN:
                {
                    break;
                }

                case SCAN_IMAGE:
                {
                    doScan(i);
                    break;
                }


                case PUSH:
                {
                    //If we have a grabber, grab block, else just push with chassis
                    break;
                }

                case DROP:
                {
                    doDrop(i);
                    break;
                }

                case PARK:
                {
                    doPark();
                    break;
                }
            }
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

    private void doLower()
    {
        RobotLog.dd(TAG, "Lowering bot");

        if(roRuBot != null)
        {
            roRuBot.putHolderAtRelease();
        }
        else
        {
            RobotLog.dd(TAG, "No RoRuRobot - delaying instead of lowering for now");
            sleep(1); //Remove this - giving 2s for drop for timing
        }
        sleep(500);
    }


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
        if(ctrScan)
        {
            RobotLog.dd(TAG, "doScanForward");
            doScanForward(segIdx);
            return;
        }

        RobotLog.dd(TAG, "doScan");
        double hdgAdj = 14.0;
        Point2d cntMinPt = RoRuField.getMineralPt(alliance, startPos, MineralDetector.Position.CENTER);
        Point2d rgtMinPt = RoRuField.getMineralPt(alliance, startPos, MineralDetector.Position.RIGHT);
        Point2d lftMinPt = RoRuField.getMineralPt(alliance, startPos, MineralDetector.Position.LEFT);
        RobotLog.dd(TAG, "Ctr Orig Offset Min Pt: " + cntMinPt.toString());
        RobotLog.dd(TAG, "Rgt Orig Offset Min Pt: " + rgtMinPt.toString());
        RobotLog.dd(TAG, "Lft Orig Offset Min Pt: " + lftMinPt.toString());
        Point2d cntMinOPt = RoRuField.getMinOffsetPt(alliance, startPos, MineralDetector.Position.CENTER);
        Point2d rgtMinOPt = RoRuField.getMinOffsetPt(alliance, startPos, MineralDetector.Position.RIGHT);
        Point2d lftMinOPt = RoRuField.getMinOffsetPt(alliance, startPos, MineralDetector.Position.LEFT);
        RobotLog.dd(TAG, "Ctr New Offset Min Pt: " + cntMinOPt.toString());
        RobotLog.dd(TAG, "Rgt New Offset Min Pt: " + rgtMinOPt.toString());
        RobotLog.dd(TAG, "Lft New Offset Min Pt: " + lftMinOPt.toString());
        Point2d cntMinAPt = RoRuField.getMinActualPt(alliance, startPos, MineralDetector.Position.CENTER);
        Point2d rgtMinAPt = RoRuField.getMinActualPt(alliance, startPos, MineralDetector.Position.RIGHT);
        Point2d lftMinAPt = RoRuField.getMinActualPt(alliance, startPos, MineralDetector.Position.LEFT);
        RobotLog.dd(TAG, "Ctr New Actual Min Pt: " + cntMinAPt.toString());
        RobotLog.dd(TAG, "Rgt New Actual Min Pt: " + rgtMinAPt.toString());
        RobotLog.dd(TAG, "Lft New Actual Min Pt: " + lftMinAPt.toString());
        Point2d curPt = pathSegs.get(segIdx).getTgtPt();
        //Finish bisector

        double curAng = pathSegs.get(segIdx).angle();
//        double newHdg = curAng - hdgAdj;
        double newHdg = robot.getGyroFhdg() - hdgAdj;
        doGyroTurn(newHdg, "scanAdj");

        if(useLight)
            CameraDevice.getInstance().setFlashTorchMode(true) ;

        mineralPos =  getMineralPos();
        //mineralPos = MineralDetector.Position.CENTER;
        RobotLog.dd(TAG, "doScan mineralPos = %s", mineralPos);

        if(useLight)
            CameraDevice.getInstance().setFlashTorchMode(false);

        setMineralPoint(segIdx);
    }

    private void doScanForward(int segIdx)
    {
        double curHdg = robot.getGyroFhdg();

        double[] dhdgs = {0.0, -27.7, 27.7};
        MineralDetector.Position[] pos =
                {
                        MineralDetector.Position.CENTER,
                        MineralDetector.Position.RIGHT,
                        MineralDetector.Position.LEFT
                };

        if(useLight)
            CameraDevice.getInstance().setFlashTorchMode(true) ;

        for(int i=0; i < 3; ++i)
        {
            dhdgs[i]+=curHdg;
            RobotLog.dd(TAG, "doScanForward " + pos[i]);
            if (i != 0)
            {
                doEncoderTurn(dhdgs[i], "scanAdj");
                doGyroTurn(dhdgs[i], "scanAdj");
            }

            if(getMineralPos() == MineralDetector.Position.GOLDAHEAD)
            {
                mineralPos = pos[i];
                RobotLog.dd(TAG, "doScanForward mineralPos = %s", mineralPos);
                break;
            }
        }

        if(useLight)
            CameraDevice.getInstance().setFlashTorchMode(false);

        setMineralPoint(segIdx);
    }

    private void setMineralPoint(int segIdx)
    {
        RobotLog.dd(TAG, "Getting mineralPt for %s %s %s", alliance, startPos, mineralPos);
        PositionOption otherPos = Route.StartPos.START_2;
        if(startPos == Route.StartPos.START_2) otherPos = Route.StartPos.START_1;
        tgtMinPt1 = RoRuField.getMineralPt(alliance, startPos, mineralPos);
        tgtMinPt2 = RoRuField.getMineralPt(alliance, otherPos, mineralPos);
        RobotLog.dd(TAG, "MineralPt = %s", tgtMinPt1);

        Segment sMin = pathSegs.get(segIdx+1);
        Segment sRev = pathSegs.get(segIdx+2);
        sMin.setEndPt(tgtMinPt1);
        sRev.setStrtPt(tgtMinPt1);
    }

    private void doDrop(int segIdx)
    {
        roRuBot.threadedHolderStow();
        RobotLog.dd(TAG, "Dropping marker");

        if(roRuBot != null)
        {
            roRuBot.dropMarker();
        }
        else
        {
            RobotLog.dd(TAG, "No RoRuRobot - can't drop");
        }

        boolean goForTwo = false;
        if(parkPos == DEFEND_PARK) goForTwo = true;

        if(startPos == Route.StartPos.START_1 & goForTwo)
        {
            RobotLog.dd(TAG, "Adjusting points for go for Two");
            Segment sMin = pathSegs.get(segIdx + 1);
            Segment sRev = pathSegs.get(segIdx + 2);
            sMin.setEndPt(tgtMinPt2);
            sRev.setStrtPt(tgtMinPt2);
        }
    }

    private void doPark()
    {
        RobotLog.dd(TAG, "Parking bot");
        if(roRuBot != null)
        {
            int stowCounts  = 0;
            int dropCounts  = -2400; //-(int)(10 * robot.ARM_CPD);
            int hoverCounts = -5000; //-(int)(20 * robot.ARM_CPD);
            roRuBot.parkMarker();

            RobotLog.dd(TAG, "Moving to arm pos %d from %d", hoverCounts, 0);
            roRuBot.armPitch.setTargetPosition(hoverCounts);
            roRuBot.armPitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roRuBot.armPitch.setPower(0.5);
        }
        else
        {
            RobotLog.dd(TAG, "No RoRuRobot - can't extend marker");
        }

        if(roRuBot != null)
        {
            roRuBot.deployParker();
        }
        else
        {
            RobotLog.dd(TAG, "No RoRuRobot - can't extend parker");
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
            RobotLog.dd(TAG,"Doing color seg");
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


    private void doEncoderTurn(double fHdg, int thresh, String prefix)
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

    private MineralDetector.Position getMineralPos()
    {
        if(!opModeIsActive() || mineralPos != MineralDetector.Position.NONE)
            return mineralPos;

        tracker.setActive(true);
        mineralPos = MineralDetector.Position.NONE;
        RobotLog.dd(TAG, "Set qsize to get frames");
        tracker.setFrameQueueSize(1);
        RobotLog.dd(TAG, "Start LD sensing");
        det.startSensing();

        MineralDetector.Position minPos = MineralDetector.Position.NONE;

        double mineralTimeout = 0.5;
        ElapsedTime mtimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(opModeIsActive()                                   &&
              minPos == MineralDetector.Position.NONE &&
              mtimer.seconds() < mineralTimeout)
        {
            tracker.updateImages();
            Bitmap rgbImage = tracker.getLastImage();

            boolean tempTest = false;
            if(rgbImage == null)
            {
                RobotLog.dd(TAG, "getMineralPos - image from tracker is null");
                //noinspection ConstantConditions
                if(!tempTest) continue;
            }
            det.setBitmap(rgbImage);
            det.logDebug();
            det.logTelemetry();
            if(det instanceof MineralDetector)
                minPos = ((MineralDetector) det).getMineralPos();

            if(minPos == MineralDetector.Position.NONE)
                sleep(10);
        }

        det.stopSensing();
        tracker.setFrameQueueSize(0);
        tracker.setActive(false);

        dashboard.displayPrintf(1, "MIN: " + minPos);

        return minPos;
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

        FtcMenu.walkMenuTree(startPosMenu);

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
    private RoRuBot roRuBot = null;

    @SuppressWarnings("FieldCanBeLocal")
    private Point2d tgtMinPt1;
    private Point2d tgtMinPt2;

    private boolean ctrScan = false;
    @SuppressWarnings("FieldCanBeLocal")
    private boolean pitScan = false;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime startTimer = new ElapsedTime();
    private Drivetrain drvTrn = new Drivetrain();

    private Detector det;
    private static ImageTracker tracker;
    private RelicRecoveryVuMark key = RelicRecoveryVuMark.UNKNOWN;
    private MineralDetector.Position mineralPos = MineralDetector.Position.NONE;

    private static Point2d curPos;
    private static double  curHdg;
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
    private boolean firstInState = true;

    private int postSleep = 150;

    private int colSegNum = 0;

    @SuppressWarnings("FieldCanBeLocal")
    private boolean useLight = false;

    @SuppressWarnings("FieldCanBeLocal")
    private boolean doMore = true;

    private double ppitXShift = 0.0;

    @SuppressWarnings("FieldCanBeLocal")
    private boolean doScanInInit = false;
    @SuppressWarnings("FieldCanBeLocal")
    private boolean useKeyPtAdjust = true;

    private String robotName = "";
    private static final String TAG = "SJH_RRA";
}