package org.firstinspires.ftc.teamcode.opModes;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.PositionOption;
import org.firstinspires.ftc.teamcode.field.Route;
import org.firstinspires.ftc.teamcode.field.UgField;
import org.firstinspires.ftc.teamcode.field.UgRoute;
import org.firstinspires.ftc.teamcode.image.Detector;
import org.firstinspires.ftc.teamcode.image.ImageTracker;
import org.firstinspires.ftc.teamcode.image.StoneDetector;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
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
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;

import static org.firstinspires.ftc.teamcode.field.Route.ParkPos.DEFEND_PARK;

//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="UgAutoShelby", group="Auton")
//@Disabled
public class UgAutoShelby extends InitLinearOpMode implements FtcMenu.MenuButtons
{
    public UgAutoShelby()
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
        while(!isStopRequested() && !isStarted())
        {
            if(initCycle % 10 == 0)
            {
                double shdg = robot.getGyroHdg();
                double fhdg = robot.getGyroFhdg();
                dashboard.displayPrintf(0, "HDG %4.2f FHDG %4.2f", shdg, fhdg);
                dashboard.displayPrintf(10, "GyroReady %s RGyroReady %s",
                        gyroReady, robot.gyroReady);
                if (robot.leftMotor != null && robot.rightMotor != null)
                    dashboard.displayPrintf(11, "LENC %d RENC %d",
                            robot.leftMotor.getCurrentPosition(), robot.rightMotor.getCurrentPosition());

                if (robot.colorSensor != null)
                {
                    int r = robot.colorSensor.red();
                    int g = robot.colorSensor.green();
                    int b = robot.colorSensor.blue();
                    RobotLog.dd(TAG, "RGB = %d %d %d", r, g, b);
                    dashboard.displayPrintf(15, "RGB %d %d %d", r, g, b);
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
        es.shutdownNow();
        if(drvTrn != null) drvTrn.cleanup();
        if(tracker != null) {
            tracker.setFrameQueueSize(0);
            tracker.setActive(false);
        }
        if(det != null) det.cleanupCamera();
    }

    private int grabNum = 0;

    private void setup()
    {
        dashboard.displayPrintf(0, "PLEASE WAIT - STARTING - CHECK DEFAULTS");
        logData = true;

        dashboard.displayPrintf(6, "HIT A TO ACCEPT VALUES");
        dashboard.displayPrintf(7, "HIT B FOR MENU");
        RobotLog.ii(TAG, "SETUP");

        ElapsedTime mTimer = new ElapsedTime();
        boolean doMen = false;
        while(!isStopRequested() && mTimer.seconds() < 5.0)
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
            //robot = new SkyBot(robotName);
            //skyBot = (SkyBot)robot;
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

//        dashboard.displayPrintf(6, "ALIGN TO FIELD THEN HIT X TO START GYRO INIT");
//        dashboard.displayPrintf(7, "OR ALIGN TO FIRST SEG THEN HIT Y TO SKIP");
//        ElapsedTime gyroTimer = new ElapsedTime();
        boolean gyroSetToField = false;
//        boolean gyroSetToSeg   = false;
//        while(gyroTimer.seconds() < 10.0)
//        {
//            gpad1.update();
//            if(gpad1.just_pressed(ManagedGamepad.Button.X))
//            {
//                gyroSetToField = true;
//                break;
//            }
//            if(gpad1.just_pressed(ManagedGamepad.Button.Y))
//            {
//                gyroSetToSeg = true;
//                break;
//            }
//        }

//        if(gyroSetToField)
//            RobotLog.ii(TAG,"X pressed: gyroSetToField");
//        if(gyroSetToSeg)
//            RobotLog.ii(TAG,"Y pressed: gyroSetToSeg");

        dashboard.displayPrintf(0, "GYRO CALIBRATING DO NOT TOUCH OR START");
        ShelbyBot.curOpModeType = ShelbyBot.OpModeType.AUTO;
        robot.init(this, robotName);

        if (robot.imu != null || robot.gyro  != null)
        {
            gyroReady = robot.calibrateGyro();
        }
        dashboard.displayPrintf(0, "GYRO CALIBATED: %s", gyroReady);

//        dashboard.displayPrintf(6, gyroSetToField ? "Field" : "1stSeg" +"Init done");
        dashboard.displayPrintf(6, "");
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
        tracker = new ImageTracker(new UgField());

        setupLogger();

        dl.addField("Start: " + startPos.toString());
        dl.addField("Alliance: " + alliance.toString());
        dl.addField("Park: "     + parkPos.toString());
        RobotLog.ii(TAG, "STARTPOS %s", startPos);
        RobotLog.ii(TAG, "ALLIANCE %s", alliance);
        RobotLog.ii(TAG, "PARKPOS %s", parkPos);
        RobotLog.ii(TAG, "DELAY    %4.2f", delay);
        RobotLog.ii(TAG, "BOT      %s", robotName);

        //TODO: CHANGE to UgRoute
        Route pts = new UgRoute(startPos, alliance, robotName);

        pathSegs.addAll(Arrays.asList(pts.getSegments()));

        initHdg = pathSegs.get(0).getFieldHeading();

        ShelbyBot.DriveDir startDdir = pathSegs.get(0).getDir();
        robot.setDriveDir(startDdir);

        RobotLog.dd(TAG, "START DRIVEDIR =%s", startDdir);

        RobotLog.ii(TAG, "ROUTE: \n" + pts.toString());

        Point2d currPoint = pathSegs.get(0).getStrtPt();
        drvTrn.setCurrPt(currPoint);

        //noinspection ConstantConditions
        drvTrn.setStartHdg(gyroSetToField ? 0 : initHdg);
        //noinspection ConstantConditions
        robot.setInitHdg(gyroSetToField   ? 0 : initHdg);

        RobotLog.ii(TAG, "Start %s.", currPoint);
        dashboard.displayPrintf(8, "PATH: Start at %s", currPoint);

        RobotLog.ii(TAG, "IHDG %4.2f", initHdg);

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

            if (curSeg.getLength() >= 0.01)
            {
                RobotLog.ii(TAG, "ENCODER TURN %s t=%6.4f", curSeg.getName(),
                        startTimer.seconds());
                //doEncoderTurn(curSeg.getFieldHeading(), segName + " encoderTurn"); //quick but rough
                RobotLog.ii(TAG, "GYRO TURN %s t=%6.4f", curSeg.getName(),
                        startTimer.seconds());
                doGyroTurn(curSeg.getFieldHeading(), segName + " gyroTurn");
            }

            if (curSeg.getLength() >= 0.1)
            {
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
                    RobotLog.ii(TAG, "Action SET_ALIGN");
                    doAlign(i);
                    break;
                }

                case SCAN_IMAGE:
                {
                    RobotLog.ii(TAG, "Action SCAN_IMAGE");
                    doScan(i);
                    break;
                }

                case GRAB:
                {
                    RobotLog.ii(TAG, "Action GRAB");
                    doGrab(i);
                    break;
                }


                case PUSH:
                {
                    //If we have a grabber, grab platform
                    RobotLog.ii(TAG, "Action PUSH");
                    doPlatch();
                    break;
                }

                case DROP:
                {
                    RobotLog.ii(TAG, "Action DROP");
                    doDrop(i);
                    break;
                }

                case RETRACT:
                {
                    RobotLog.ii(TAG, "Action RETRACT");
                    doUnPlatch();
                    break;
                }

                case PARK:
                {
                    RobotLog.ii(TAG, "Action PARK");
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

        //TODO: Ring scan
        //stonePos =  getStonePos();
        //RobotLog.dd(TAG, "doScan stonePos = %s", stonePos);

        if(useLight)
            CameraDevice.getInstance().setFlashTorchMode(false);

        //setStonePoint(segIdx);

        //TODO: set wobbly point based on scan result

//        MoveArmTask tsk = new MoveArmTask();
//        tsk.setElevPos(skyBot.LIFT_STOW_CNTS);
//        tsk.setXtndPos(skyBot.ARM_EXT_SNUG_POS);
//        tsk.setArotPos(getArmGrabRot());
//        tsk.setElevDly(0.01);
//        tsk.setXtndDly(0.01);
//        tsk.setArotDly(0.01);
//        tsk.setAutoCross(false);
//        RobotLog.dd(TAG, "doScan submitting non-delay task");
//        es.submit(tsk);
    }

    private void doGrab(int segIdx)
    {

    }

    private void setStonePoint(int segIdx)
    {
        RobotLog.dd(TAG, "Getting stonePt for %s %s %s seg=%d",
                alliance, startPos, stonePos, segIdx);
//        Segment sMin = pathSegs.get(segIdx+1);
//        Segment sRev = pathSegs.get(segIdx+2);
//        sMin.setEndPt(tgtPt1);
//        sRev.setStrtPt(tgtPt1);
    }

    class MoveArmTask implements Runnable
    {
//        private double elevPos; // = skyBot._liftyBoi.getCurrentPosition();
//        private double xtndPos; // = skyBot.armExtend.getCurrentPosition();
//        private double arotPos; // = skyBot.armRotate.getCurrentPosition();
//        private double elevDly = 0;
//        private double xtndDly = 0;
//        private double arotDly = 0;
//        private boolean autoCross = false;
//        private double autoX = 0.0;
//
//        void setElevPos(double pos) {this.elevPos = pos;}
//        void setXtndPos(double pos) {this.xtndPos = pos;}
//        void setArotPos(double pos) {this.arotPos = pos;}
//        void setElevDly(double dly) {this.elevDly = dly;}
//        void setXtndDly(double dly) {this.xtndDly = dly;}
//        void setArotDly(double dly) {this.arotDly = dly;}
//        void setAutoCross(boolean autoCross) {this.autoCross = autoCross;}
//        void setAutoX  (double x)   {this.autoX = x;}
//        volatile boolean getOut = false;
//
        public void run()
        {
//            double startX = drvTrn.getEstPos().getX();
//            double curX = startX;
//
//            RobotLog.dd(TAG, "MoveArmTask autoCross=" + autoCross + " startx=" + startX);
//
//            if(autoCross)
//            {
//                RobotLog.dd(TAG, "Delaying threaded arm move. startx=" + startX);
//                while(opModeIsActive() && !isStopRequested() &&
//                      !getOut &&
//                      (startX < 0.0 && curX < autoX) || (startX > 0.0 && curX > autoX))
//                {
//                    RobotLog.dd(TAG, "Delaying threaded arm move. curx=" + curX);
//                    try
//                    {
//                        Thread.sleep(20);
//                    }
//                    catch (InterruptedException ie)
//                    {
//                        getOut = true;
//                    }
//                    curX = drvTrn.getEstPos().getX();
//                }
//            }
//
//            if(!getOut)
//            {
//                RobotLog.dd(TAG, "Starting Threaded Arm Move");
//                //if(grabNum == 2) skyBot.threadedParkLong();
//                //skyBot.moveArmToLoc(elevPos, arotPos, xtndPos, elevDly, arotDly, xtndDly);
//                RobotLog.dd(TAG, "Completed Threaded Arm Move");
//            }
        }
    }

    private void doAlign(int segIdx)
    {
        RobotLog.dd(TAG, "doGrab lift to MOVE, rot to FWD, extend to STAGE at " +
                startTimer.seconds());

        Segment nxtSeg = pathSegs.get(segIdx + 1);
        Segment.Action nxtAct = nxtSeg.getAction();
//        if (nxtAct == Segment.Action.DROP) {
//            MoveArmTask crossTsk = new MoveArmTask();
//            crossTsk.setElevPos(skyBot.LIFT_STOW_CNTS);
//            crossTsk.setXtndPos(skyBot.ARM_EXT_STAGE_POS);
//            crossTsk.setArotPos(getArmDropRot());
//            crossTsk.setAutoCross(true);
//            crossTsk.setAutoX(6.0);
//            RobotLog.dd(TAG, "doGrab submitting position delay task");
//            es.submit(crossTsk);
//        }
    }

    private void doDrop(int segIdx)
    {
        RobotLog.dd(TAG, "Dropping stone %d on seg %d at %f", grabNum, segIdx, startTimer.seconds());
        //rotate arm, lower?, release gripper, return arm
    }

    private void doPlatch()
    {
        sleep(1000);
        RobotLog.dd(TAG, "Platching platform");

        sleep(1000);
    }

    private void doUnPlatch()
    {
        RobotLog.dd(TAG, "UnPlatching platform");
    }

    private void doPark()
    {
        RobotLog.dd(TAG, "Parking bot");
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

    //TODO:  Make ring detector
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

        dashboard.displayPrintf(1, "POS: " + stonePos);

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
        startPosMenu.addChoice("Start_3", Route.StartPos.START_3, false, allianceMenu);
        startPosMenu.addChoice("Start_4", Route.StartPos.START_4, false, allianceMenu);
        startPosMenu.addChoice("Start_5", Route.StartPos.START_5, false, allianceMenu);


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

    private final static double DEF_ENCTRN_PWR  = 0.8;
    private final static double DEF_GYRTRN_PWR  = 0.5;

    private List<Segment> pathSegs = new ArrayList<>();

    private TilerunnerGtoBot   robot;

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

    private final ExecutorService es = Executors.newSingleThreadExecutor();

    private String robotName = "MEC";
    private static final String TAG = "SJH_RRA";
}