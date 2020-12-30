package org.firstinspires.ftc.teamcode.opModes;

import android.graphics.Bitmap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;
import com.vuforia.Vec2F;
import com.vuforia.Vec4F;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.PositionOption;
import org.firstinspires.ftc.teamcode.field.Route;
import org.firstinspires.ftc.teamcode.field.UgField;
import org.firstinspires.ftc.teamcode.field.UgRoute;
import org.firstinspires.ftc.teamcode.field.UgRrRoute;
import org.firstinspires.ftc.teamcode.image.Detector;
import org.firstinspires.ftc.teamcode.image.ImageTracker;
import org.firstinspires.ftc.teamcode.image.RingDetector;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.MecanumDriveLRR;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.robot.TilerunnerMecanumBot;
import org.firstinspires.ftc.teamcode.util.AutoTransitioner;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Segment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.firstinspires.ftc.teamcode.util.FtcChoiceMenu;
import org.firstinspires.ftc.teamcode.util.FtcMenu;
import org.firstinspires.ftc.teamcode.util.FtcValueMenu;

import static org.firstinspires.ftc.teamcode.field.Route.StartPos.START_1;

//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//@SuppressWarnings("ConstantConditions")
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
        //SBH- drvTrn.start();
        do_main_loop();
    }

    //@SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() //throws InterruptedException
    {
        RobotLog.dd(TAG, "initCommon");
        initCommon(this, true, true, false, false);

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
                StringBuilder motStr = new StringBuilder("ENCs:");
                for (Map.Entry<String, DcMotorEx> e : robot.motors.entrySet())
                {
                    motStr.append(" ");
                    motStr.append(e.getKey());
                    motStr.append(":");
                    motStr.append(e.getValue().getCurrentPosition());
                }
                dashboard.displayText(11, motStr.toString());
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

            robot.waitForTick(initSleep);
        }

        if(!isStopRequested()) startMode();
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

    private void getPrefs()
    {
        RobotLog.dd(TAG, "robotname before pmgr: " + robotName);
        pmgr.logPrefs();

        robotName = pmgr.getBotName();
        alliance = Field.Alliance.valueOf(pmgr.getAllianceColor());
        delay    = pmgr.getDelay();
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

        dashboard.displayPrintf(2, "Pref BOT: %s", robotName);
        dashboard.displayPrintf(3, "Pref Alliance: %s", alliance);
        dashboard.displayPrintf(4, "Pref StartPos: %s %s", startPos, parkPos);
        dashboard.displayPrintf(5, "Pref Delay: %.2f", delay);
    }

    private void setup()
    {
        getPrefs();
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

        if(isStopRequested()) return;

        if(doMen) doMenus();

        dashboard.displayPrintf(0, "INITIALIZING");
        dashboard.displayPrintf(1, "Prefs/Menu Done");
        dashboard.displayPrintf(6, "");
        dashboard.displayPrintf(7, "");

        final String teleopName = "Mecanum";
        robot = new TilerunnerMecanumBot();

        //Since we only have 5 seconds between Auton and Teleop, automatically load
        //teleop opmode
        RobotLog.dd(TAG, "Setting up auto tele loader : %s", teleopName);
        AutoTransitioner.transitionOnStop(this, teleopName);
        dashboard.displayPrintf(1, "AutoTrans setup");

        ShelbyBot.curOpModeType = ShelbyBot.OpModeType.AUTO;

        RobotConstants.Chassis chas = RobotConstants.Chassis.MEC2;
        try
        {
            chas = RobotConstants.Chassis.valueOf(robotName);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "Robotname %s invalid. Defaulting to %s", robotName, chas);
            chas = RobotConstants.Chassis.MEC2;
        }

        robot.init(this, chas, true);

        robot.setBcm(LynxModule.BulkCachingMode.MANUAL);
/*SBH
        pts = new UgRoute(startPos, alliance);
        pathSegs.addAll(Arrays.asList(pts.getSegments()));

        ShelbyBot.DriveDir startDdir = pathSegs.get(0).getDir();
        robot.setDriveDir(startDdir);

        initHdg = pathSegs.get(0).getFieldHeading();
        robot.setInitHdg(initHdg);
SBH*/
        //SBH adds
        ugrr = new UgRrRoute((MecanumDriveLRR)(robot.drive), startPos, alliance);
        ShelbyBot.DriveDir startDdir = ShelbyBot.DriveDir.PUSHER;
        robot.setDriveDir(startDdir);
        initHdg =UgRrRoute.srtPose.getHeading();
        robot.setInitHdg(initHdg);
        robot.setAlliance(alliance);
        //SBH end adds

        dashboard.displayPrintf(0, "GYRO CALIBRATING DO NOT TOUCH OR START");
        if (robot.imu != null)
        {
            gyroReady = robot.calibrateGyro();
        }
        dashboard.displayPrintf(0, "GYRO CALIBATED: %s", gyroReady);
        dashboard.displayPrintf(1, "Robot Inited");

        /* SBH -
        drvTrn.init(robot);
        drvTrn.setStartHdg(initHdg);
        drvTrn.setRampUp(true);
        int colThresh = 450;
        drvTrn.setColorThresh(colThresh);
        Point2d currPoint = pathSegs.get(0).getStrtPt();
        drvTrn.setCurrPt(currPoint);
        drvTrn.setEstPose(currPoint, initHdg);
        robot.drive.setPoseEstimate(new Pose2d(currPoint.getX(), currPoint.getY(), initHdg));
        SBH*/
        //SBH Add
        robot.drive.setPoseEstimate(UgRrRoute.srtPose);
        //SBH end adds

        dashboard.displayPrintf(1, "Robot & DrvTrn Inited");

        det = new RingDetector(robotName);
        RobotLog.dd(TAG, "Setting up vuforia");
        tracker = new ImageTracker(new UgField());
        det.setTelemetry(telemetry);

        setupLogger();
        dl.addField("Start: " + startPos.toString());
        dl.addField("Alliance: " + alliance.toString());
        dl.addField("Park: "     + parkPos.toString());
        RobotLog.ii(TAG, "STARTPOS %s", startPos);
        RobotLog.ii(TAG, "ALLIANCE %s", alliance);
        RobotLog.ii(TAG, "PARKPOS %s", parkPos);
        RobotLog.ii(TAG, "DELAY    %4.2f", delay);
        RobotLog.ii(TAG, "BOT      %s", robotName);
        RobotLog.dd(TAG, "Robot CPI " + RobotConstants.DT_CPI);
        RobotLog.dd(TAG, "BOTDIR=%s START_DDIR =%s",
            RobotConstants.DT_DIR, startDdir);

        /* SBH -
        RobotLog.ii(TAG, "ROUTE: \n" + pts.toString());
        RobotLog.ii(TAG, "Start %s IHDG %4.2f", currPoint, initHdg);
        dashboard.displayPrintf(8, "PATH: Start at %s", currPoint);
        SBH*/
        //SBH Add
        RobotLog.ii(TAG, "Start %s", UgRrRoute.srtPose);
        dashboard.displayPrintf(8, "PATH: Start at %s", UgRrRoute.srtPose);
        //SBH end adds

        com.vuforia.CameraCalibration camCal = com.vuforia.CameraDevice.getInstance().getCameraCalibration();
        Vec4F distParam = camCal.getDistortionParameters();
        Vec2F camFov    = camCal.getFieldOfViewRads();
        Vec2F camFlen   = camCal.getFocalLength();
        Vec2F camPpt    = camCal.getPrincipalPoint();
        Vec2F camSize   = camCal.getSize();

        RobotLog.dd(TAG, "DistortionParams %f %f %f %f",
            distParam.getData()[0],
            distParam.getData()[1],
            distParam.getData()[2],
            distParam.getData()[3]);
        RobotLog.dd(TAG, "CamFOV %f %f", camFov.getData()[0], camFov.getData()[1]);
        RobotLog.dd(TAG, "CamFlen %f %f", camFlen.getData()[0], camFlen.getData()[1]);
        RobotLog.dd(TAG, "CamPpt %f %f", camPpt.getData()[0], camPpt.getData()[1]);
        RobotLog.dd(TAG, "CamSize %f %f", camSize.getData()[0], camSize.getData()[1]);
    }

    private void do_main_loop()
    {
        timer.reset();
        startTimer.reset();
        dl.resetTime();

        RobotLog.ii(TAG, "STARTING AT %4.2f", timer.seconds());
        if(logData)
        {
            //SBH- Point2d spt = pathSegs.get(0).getStrtPt();
            //SBH+
            Pose2d spt = UgRrRoute.srtPose;
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

        /*SBH
        boolean SkipNextSegment = false;

        if(robotName.equals("GTO1"))
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

            double tgtHdg = curSeg.getFieldHeading();
            if(robot.getDriveDir() != RobotConstants.DT_DIR)
            {
                tgtHdg = angNormalize(tgtHdg + 180);
            }

            String segLogStr = String.format(Locale.US, "%s - %s H: %4.1f",
                    curSeg.getStrtPt().toString(),
                    curSeg.getTgtPt().toString(),
                    curSeg.getFieldHeading());
            drvTrn.logData(true, segName + " " + segLogStr);

            if (curSeg.getLength() >= 0.01 &&
                (curSeg.getDir() != ShelbyBot.DriveDir.LEFT &&
                 curSeg.getDir() != ShelbyBot.DriveDir.RIGHT))
            {
//                RobotLog.ii(TAG, "ENCODER TURN %s t=%6.4f", curSeg.getName(),
//                        startTimer.seconds());
//                doEncoderTurn(tgtHdg, segName + " encoderTurn"); //quick but rough
                RobotLog.ii(TAG, "GYRO TURN %s t=%6.4f", curSeg.getName(),
                        startTimer.seconds());
                doGyroTurn(tgtHdg, segName + " gyroTurn");
            }

            if (curSeg.getLength() >= 0.1)
            {
                RobotLog.ii(TAG, "MOVE %s t=%6.4f", curSeg.getName(),
                        startTimer.seconds());
                doMove(curSeg);
            }

            double pturn = curSeg.getPostTurn();
            if (usePostTurn && pturn != curSeg.getFieldHeading())
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
                    pathSegs.get(i).getTgtPt(), tgtHdg);

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

                case SHOOT:
                {
                    RobotLog.ii(TAG, "Action SHOOT");
                    doShoot(i);
                    break;
                }
            }

            Pose2d ePose = drvTrn.getEstPose();
            robot.setAutonEndPos(new Point2d(ePose.getX(), ePose.getY()));
            robot.setAutonEndHdg(ePose.getHeading());

            RobotLog.dd(TAG, "Finished seg %d at X:%.2f Y:%.2f H:%.2f",
                i, ePose.getX(), ePose.getY(), Math.toDegrees(ePose.getHeading()));
        }
        SBH*/

        //SBH add
        RobotLog.ii(TAG, "Action SCAN_IMAGE");
        doScan(0);

        for(Map.Entry<UgRrRoute.State, Trajectory> entry : ugrr.stateTrajMap.entrySet())
        {
            UgRrRoute.State state = entry.getKey();
            Trajectory traj = entry.getValue();
            if(state == UgRrRoute.State.IDLE) break;

            RobotLog.ii(TAG, "Driving trajectory %s", state);;
            ((MecanumDriveLRR)(robot.drive)).followTrajectoryAsync(traj);
            while(opModeIsActive() && !isStopRequested() &&
                ((MecanumDriveLRR)(robot.drive)).isBusy())
            {
                robot.update();
                Pose2d ePose = robot.drive.getPoseEstimate();
                robot.setAutonEndPos(new Point2d(ePose.getX(), ePose.getY()));
                robot.setAutonEndHdg(ePose.getHeading());
                RobotLog.dd(TAG, "Drive %s at %s", state, ePose);
            }
        }

        //end SBH add

        Pose2d ePose = robot.drive.getPoseEstimate();
        robot.setAutonEndPos(new Point2d(ePose.getX(), ePose.getY()));
        robot.setAutonEndHdg(ePose.getHeading());
        RobotLog.dd(TAG, "Finished auton segments at X:%.2f Y:%.2f H:%.2f",
            ePose.getX(), ePose.getY(), Math.toDegrees(ePose.getHeading()));

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

    private void update()
    {
        robot.update();
    }

    private void doScan(int segIdx)
    {
        RobotLog.dd(TAG, "doScan" + " segIdx:" + segIdx);

        if(useLight && usePhone)
            CameraDevice.getInstance().setFlashTorchMode(true) ;

        ringPos =  getRingPos();
        RobotLog.dd(TAG, "doScan RingPos = %s", ringPos);

        if(useLight && usePhone)
            CameraDevice.getInstance().setFlashTorchMode(false);

        setRingPoint(segIdx);
    }

    private void doGrab(int segIdx)
    {
        RobotLog.dd(TAG, "doGrab seg %d at %f", segIdx, startTimer.seconds());
    }

    private void setRingPoint(int segIdx)
    {
        RobotLog.dd(TAG, "Getting ringPt for %s %s %s seg=%d",
                alliance, startPos, ringPos, segIdx);

        int allnc = alliance == Field.Alliance.RED     ? 0 : 1;
        int start = startPos == START_1 ? 0 : 1;
        int rPos  = ringPos  == RingDetector.Position.NONE   ? 0 :
                    ringPos  == RingDetector.Position.LEFT   ? 0 :
                    ringPos  == RingDetector.Position.CENTER ? 1 : 2;

        //SBH add
        if(alliance == Field.Alliance.RED)
        {
            if(startPos == START_1)
            {
                if(ringPos == RingDetector.Position.CENTER)
                {
                    ugrr.stateTrajMap.put(UgRrRoute.State.DROP1,  ugrr.drop1b);
                    ugrr.stateTrajMap.put(UgRrRoute.State.CLEAR1, ugrr.clr1b);
                    ugrr.stateTrajMap.put(UgRrRoute.State.DROP2,  ugrr.drop2b);
                    ugrr.stateTrajMap.put(UgRrRoute.State.PARK,   ugrr.parkb);
                }
                else if(ringPos == RingDetector.Position.RIGHT)
                {
                    ugrr.stateTrajMap.put(UgRrRoute.State.DROP1,  ugrr.drop1c);
                    ugrr.stateTrajMap.put(UgRrRoute.State.CLEAR1, ugrr.clr1c);
                    ugrr.stateTrajMap.put(UgRrRoute.State.DROP2,  ugrr.drop2c);
                    ugrr.stateTrajMap.put(UgRrRoute.State.PARK,   ugrr.parkc);
                }
            }
            else
            {
                //TODO start2
            }
        }
        else
        {
            //TODO BLUE
        }

        /*SBH-

        //3dim array [allnc][start][rPos]
        Point2d[][][] wPts =
           {{{UgField.ROWA, UgField.ROWB, UgField.ROWC},
             {UgField.RIWA, UgField.RIWB, UgField.RIWC}},
            {{pts.convertRtoB(UgField.ROWA), pts.convertRtoB(UgField.ROWB), pts.convertRtoB(UgField.ROWC)},
             {pts.convertRtoB(UgField.RIWA), pts.convertRtoB(UgField.RIWB), pts.convertRtoB(UgField.RIWC)}}};

        Point2d wPt = wPts[allnc][start][rPos];

        RobotLog.dd(TAG, "Setting wobbly drop to %s", wPt.getName());

        for (Segment s : pathSegs) {
            String sPt = s.getStrtPt().getName();
            if (sPt.equals("ROWA") || sPt.equals("RIWA") ||sPt.equals("BOWA") || sPt.equals("BIWA"))
            {
                s.setStrtPt(wPt);
            }

            String ePt = s.getTgtPt().getName();
            if (ePt.equals("ROWA") || ePt.equals("RIWA") || ePt.equals("BOWA") || ePt.equals("BIWA"))
            {
                s.setEndPt(wPt);
            }
        }
        SBH*/
    }

    private void doAlign(int segIdx)
    {
        RobotLog.dd(TAG, "doAlign seg %d at %f", segIdx, startTimer.seconds());
    }

    private void doDrop(int segIdx)
    {
        RobotLog.dd(TAG, "Dropping wobblyBOI on seg %d at %f", segIdx, startTimer.seconds());
        if(robot.burr != null) robot.burr.shotSpeed(DEF_SHT_DST);
    }

    private void doPlatch()
    {
        RobotLog.dd(TAG, "Platching platform");
    }

    private void doUnPlatch()
    {
        RobotLog.dd(TAG, "UnPlatching platform");
    }

    private void doPark()
    {
        RobotLog.dd(TAG, "Parking bot");
    }

    private void doShoot(int segIdx)
    {
        Segment shootSeg = pathSegs .get(segIdx);
        Point2d shootPoint = shootSeg.getTgtPt();
        double shotdist = shootPoint.distance(UgField.RRG1);
        RobotLog.dd(TAG, "Shooting");
        if(robot.burr != null) robot.burr.shotSpeed(shotdist);
        sleep(500);
        // TODO add control of loader fire
        sleep(1000);
        if(robot.burr != null) robot.burr.stop();
    }

    private double angNormalize(double ang)
    {
        double ret = ang;
        while (ret >   180) ret -= 360;
        while (ret <= -180) ret += 360;
        return ret;
    }

    private void doMove(Segment seg)
    {
        if(!opModeIsActive() || isStopRequested() || robot.motors.size() < 1) return;

        drvTrn.setInitValues();

        drvTrn.setBusyAnd(true);
        String  snm = seg.getName();
        Point2d spt = seg.getStrtPt();
        Point2d ept = seg.getTgtPt();
        double  fhd = seg.getFieldHeading();
        ShelbyBot.DriveDir dir = seg.getDir();
        double speed = seg.getSpeed();
        double fudge = seg.getDrvTuner();
        Segment.TargetType ttype = seg.getTgtType();

        Drivetrain.Direction ddir = Drivetrain.Direction.FORWARD;
        switch (dir)
        {
            case LEFT:   ddir = Drivetrain.Direction.LEFT;    break;
            case RIGHT:  ddir = Drivetrain.Direction.RIGHT;   break;
            case INTAKE: ddir = Drivetrain.Direction.REVERSE; break;
            case PUSHER: ddir = Drivetrain.Direction.FORWARD; break;
        }

        RobotLog.ii(TAG, "Drive %s %s %s %6.2f %3.2f %s:%s tune: %4.2f %s",
                snm, spt, ept, fhd, speed, dir, ddir, fudge, ttype);

        dashboard.displayPrintf(2, "STATE: %s %s %s - %s %6.2f %3.2f %s",
                "DRIVE", snm, spt, ept, fhd, speed, dir);
        Pose2d curPose = drvTrn.getEstPose();
        dashboard.displayPrintf(3, "X:%.2f Y:%.2f H:%.2f",
            curPose.getX(), curPose.getY(), Math.toDegrees(curPose.getHeading()));

        drvTrn.logData(true, snm + " move");
        drvTrn.setDrvTuner(fudge);

        timer.reset();

        if(ttype == Segment.TargetType.COLOR)
        {
            RobotLog.dd(TAG, "colorSensor is %s", robot.colorSensor == null ? "null" : "good");
        }

        if(robot.colorSensor != null && ttype == Segment.TargetType.COLOR)
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
            double targetHdg = fhd;
            if(dir != RobotConstants.DT_DIR)
            {
                targetHdg = angNormalize(targetHdg+ 180.0);
            }

            if(dir == ShelbyBot.DriveDir.RIGHT || dir == ShelbyBot.DriveDir.LEFT)
            {
                drvTrn.strafe(spt.distance(ept), speed, ddir, targetHdg);
            }
            else
            {
                drvTrn.driveToPointLinear(ept, speed, ddir, targetHdg);
            }
        }

        drvTrn.setCurrPt(ept);

        RobotLog.ii(TAG, "Completed move %s. Time: %6.3f HDG: %6.3f",
                seg.getName(), timer.time(), robot.getGyroFhdg());
    }


    private void doEncoderTurn(double fHdg, @SuppressWarnings("SameParameterValue") int thresh, String prefix)
    {
        if(!opModeIsActive() || isStopRequested() || robot.motors.size() < 1) return;
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
        if(!opModeIsActive() || isStopRequested() || robot.motors.size() < 1) return;

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


    private RingDetector.Position getRingPos()
    {
        if(!opModeIsActive() || ringPos != RingDetector.Position.NONE)
            return ringPos;

        tracker.setActive(true);
        ringPos = RingDetector.Position.NONE;
        RobotLog.dd(TAG, "Set qsize to get frames");
        tracker.setFrameQueueSize(1);
        RobotLog.dd(TAG, "Start LD sensing");
        det.startSensing();

        RingDetector.Position ringPos = RingDetector.Position.NONE;

        double ringTimeout = 0.5;

        ElapsedTime mtimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(opModeIsActive()                                   &&
              ringPos == RingDetector.Position.NONE &&
              mtimer.seconds() < ringTimeout)
        {
            RobotLog.dd(TAG, "getringPos - loop calling tracker.updateImages");
            tracker.updateImages();
            RobotLog.dd(TAG, "getringPos - loop calling tracker.getLastImage");
            Bitmap rgbImage = tracker.getLastImage();

            boolean tempTest = false;
            if(rgbImage == null)
            {
                RobotLog.dd(TAG, "getringPos - image from tracker is null");
                if(!tempTest) continue;
            }
            RobotLog.dd(TAG, "getringPos - loop calling det.setBitmap");
            det.setBitmap(rgbImage);
            RobotLog.dd(TAG, "getringPos - loop calling det.logDebug");
            det.logDebug();
            RobotLog.dd(TAG, "getringPos - loop calling det.logTelemetry");
            det.logTelemetry();
            RobotLog.dd(TAG, "getringPos - loop calling det.getRingPos");
            if(det instanceof RingDetector)
                ringPos = ((RingDetector) det).getRingPos();

            if(ringPos == RingDetector.Position.NONE)
                sleep(10);
        }

        det.stopSensing();
        tracker.setFrameQueueSize(0);
        tracker.setActive(false);

        dashboard.displayPrintf(1, "POS: " + ringPos);

        if (ringPos == RingDetector.Position.NONE)
        {
            RobotLog.dd(TAG, "No ring answer found - defaulting to A");
            ringPos = RingDetector.Position.LEFT;
        }
        return ringPos;
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

        startPosMenu.addChoice("Start_1", START_1, true, allianceMenu);
        startPosMenu.addChoice("Start_2", Route.StartPos.START_2, false, allianceMenu);
        startPosMenu.addChoice("Start_3", Route.StartPos.START_3, false, allianceMenu);
        startPosMenu.addChoice("Start_4", Route.StartPos.START_4, false, allianceMenu);
        startPosMenu.addChoice("Start_5", Route.StartPos.START_5, false, allianceMenu);


        allianceMenu.addChoice("RED",  Field.Alliance.RED,  true, parkMenu);
        allianceMenu.addChoice("BLUE", Field.Alliance.BLUE, false, parkMenu);

        parkMenu.addChoice("CENTER_PARK", Route.ParkPos.CENTER_PARK, true, robotNameMenu);
        parkMenu.addChoice("DEFEND_PARK", Route.ParkPos.DEFEND_PARK, false, robotNameMenu);

        robotNameMenu.addChoice("GTO1", "GTO1", true, delayMenu);
        robotNameMenu.addChoice("MEC1", "MEC1", false, delayMenu);
        robotNameMenu.addChoice("MEC2", "MEC2", false, delayMenu);
        robotNameMenu.addChoice("MEC3", "MEC3", false, delayMenu);

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

    private final List<Segment> pathSegs = new ArrayList<>();

    private TilerunnerMecanumBot   robot;

    private Route pts;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime startTimer = new ElapsedTime();
    private final Drivetrain drvTrn = new Drivetrain();

    private Detector det;
    private static ImageTracker tracker;
    private RingDetector.Position ringPos = RingDetector.Position.NONE;

    private static Point2d curPos;
    private double initHdg = 0.0;
    private boolean gyroReady;
    @SuppressWarnings("FieldCanBeLocal")
    private final boolean usePostTurn = true;

    private final double DEF_SHT_DST = UgField.ROSA.distance(UgField.RRG1);

    private static PositionOption startPos = START_1;
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
    private final boolean useImageLoc  = false;

    private int colSegNum = 0;

    private static final boolean useLight = true;
    private static final boolean usePhone = false;

    private final ExecutorService es = Executors.newSingleThreadExecutor();

    private String robotName = "MEC2";
    private static final String TAG = "SJH_RRA";

    private UgRrRoute ugrr;


//    public static void main(String[] args)
//    {
//        for (RingDetector.Position ringPos : RingDetector.Position.values()) {
//            for (Field.Alliance all : Field.Alliance.values()) {
//                for (PositionOption strt : EnumSet.range(START_1,START_2)) {
//                    int allnc = all == Field.Alliance.RED ? 0 : 1;
//                    int start = strt == START_1 ? 0 : 1;
//                    int rPos =
//                            ringPos == RingDetector.Position.NONE ? 0 :
//                                    ringPos == RingDetector.Position.LEFT ? 0 :
//                                            ringPos == RingDetector.Position.CENTER ? 1 : 2;
//
//                    //3dim array [allnc][start][rPos]
//                    Point2d[][][] wPts =
//                            {{{UgField.RRWA, UgField.RRWB, UgField.RRWC},
//                                    {UgField.RLWA, UgField.RLWB, UgField.RLWC}},
//                                    {{UgRoute.convertRtoB(UgField.RRWA), UgRoute.convertRtoB(UgField.RRWB), UgRoute.convertRtoB(UgField.RRWC)},
//                                            {UgRoute.convertRtoB(UgField.RLWA), UgRoute.convertRtoB(UgField.RLWB), UgRoute.convertRtoB(UgField.RLWC)}}};
//
//                    Point2d wPt = wPts[allnc][start][rPos];
//
//                    String s = String.format("Alnc %4s Strt %s Rpos %6s %4s", all, strt, ringPos, wPt.getName());
//                    System.out.println(s);
//                    //RobotLog.dd(TAG, "Setting wobbly drop to " + wPt.getName());
//                }
//            }
//        }
//    }
}