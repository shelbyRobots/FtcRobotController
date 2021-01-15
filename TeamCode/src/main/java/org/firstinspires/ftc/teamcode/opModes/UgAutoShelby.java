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
import org.firstinspires.ftc.teamcode.field.UgRrRoute;
import org.firstinspires.ftc.teamcode.image.Detector;
import org.firstinspires.ftc.teamcode.image.ImageTracker;
import org.firstinspires.ftc.teamcode.image.RingDetector;
import org.firstinspires.ftc.teamcode.robot.Loader;
import org.firstinspires.ftc.teamcode.robot.MecanumDriveLRR;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.robot.TilerunnerMecanumBot;
import org.firstinspires.ftc.teamcode.util.AutoTransitioner;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;
import org.firstinspires.ftc.teamcode.util.Point2d;

import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.firstinspires.ftc.teamcode.util.FtcChoiceMenu;
import org.firstinspires.ftc.teamcode.util.FtcMenu;
import org.firstinspires.ftc.teamcode.util.FtcValueMenu;

import static org.firstinspires.ftc.teamcode.field.Route.StartPos.START_1;

//@SuppressWarnings("ConstantConditions")
@Autonomous(name="UgAutoShelby", group="Auton")
//@Disabled
public class UgAutoShelby extends InitLinearOpMode implements FtcMenu.MenuButtons
{
    public UgAutoShelby()
    {
    }

    private void startMode()
    {
        dashboard.clearDisplay();
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

        ugrr = new UgRrRoute(robot, startPos, alliance);
        ShelbyBot.DriveDir startDdir = ShelbyBot.DriveDir.PUSHER;
        robot.setDriveDir(startDdir);
        initHdg = UgRrRoute.startPose.getHeading();
        robot.setInitHdg(initHdg);
        robot.setAlliance(alliance);

        dashboard.displayPrintf(0, "GYRO CALIBRATING DO NOT TOUCH OR START");
        if (robot.imu != null)
        {
            gyroReady = robot.calibrateGyro();
        }
        dashboard.displayPrintf(0, "GYRO CALIBATED: %s", gyroReady);
        dashboard.displayPrintf(1, "Robot Inited");

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
        startTimer.reset();
        dl.resetTime();

        RobotLog.ii(TAG, "STARTING AT %.2f %.2f", startTimer.seconds(), timer.seconds());
        if(logData)
        {
            //SBH- Point2d spt = pathSegs.get(0).getStrtPt();
            //SBH+
            Pose2d spt = UgRrRoute.startPose;
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

        double shootWait = 2.0;
        ElapsedTime shootTimer = new ElapsedTime();

        RobotLog.ii(TAG, "Action SCAN_IMAGE");
        doScan();
        Pose2d ePose = robot.drive.getPoseEstimate();

        for(Map.Entry<UgRrRoute.State, Trajectory> entry : ugrr.stateTrajMap.entrySet())
        {
            timer.reset();
            UgRrRoute.State state = entry.getKey();
            Trajectory traj = entry.getValue();
            if(state == UgRrRoute.State.IDLE) break;

            RobotLog.ii(TAG, "Driving trajectory %s", state);
            ((MecanumDriveLRR)(robot.drive)).followTrajectoryAsync(traj);
            while(opModeIsActive() && !isStopRequested() &&
                ((MecanumDriveLRR)(robot.drive)).isBusy())
            {
                robot.update();

                ePose = robot.drive.getPoseEstimate();
                robot.setAutonEndPos(new Point2d(ePose.getX(), ePose.getY()));
                robot.setAutonEndHdg(ePose.getHeading());
                RobotLog.dd(TAG, "Drive %s from %s at %.2f",
                    state, ePose, startTimer.seconds());
            }

            RobotLog.ii(TAG, "Finished %s at %s at %.2f in %.2f",
                state, ePose, startTimer.seconds(), timer.seconds());

            if(state == UgRrRoute.State.SHOOT)
            {
                shootTimer.reset();
                while(opModeIsActive() && !isStopRequested() &&
                    shootTimer.seconds() < shootWait)
                {
                    robot.update();
                    robot.waitForTick(10);
                }

                if(robot.burr != null) robot.burr.stop();
                if(robot.loader != null)
                {
                    robot.loader.load(0.0);
                    robot.loader.setGatePos(Loader.gatePos.CLOSE);
                    robot.loader.whlStp();
                }
            }
        }

        ePose = robot.drive.getPoseEstimate();
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

    private void doScan()
    {
        RobotLog.dd(TAG, "doScan");

        if(useLight && usePhone)
            CameraDevice.getInstance().setFlashTorchMode(true) ;

        ringPos =  getRingPos();
        RobotLog.dd(TAG, "doScan RingPos = %s", ringPos);

        if(useLight && usePhone)
            CameraDevice.getInstance().setFlashTorchMode(false);

        setRingPoint();
    }

    private void setRingPoint()
    {
        RobotLog.dd(TAG, "Getting ringPt for %s %s %s",
                alliance, startPos, ringPos);

//        int allnc = alliance == Field.Alliance.RED     ? 0 : 1;
//        int start = startPos == START_1 ? 0 : 1;
//        int rPos  = ringPos  == RingDetector.Position.NONE   ? 0 :
//                    ringPos  == RingDetector.Position.LEFT   ? 0 :
//                    ringPos  == RingDetector.Position.CENTER ? 1 : 2;

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
                RobotLog.dd(TAG, "Setup start2");
            }
        }
        else
        {
            //TODO BLUE
            RobotLog.dd(TAG, "Setup blue");
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

    @SuppressWarnings("unused")
    private double angNormalize(double ang)
    {
        double ret = ang;
        while (ret >   180) ret -= 360;
        while (ret <= -180) ret += 360;
        return ret;
    }

    private RingDetector.Position getRingPos()
    {
        if(!opModeIsActive() || ringPos != RingDetector.Position.NONE)
            return ringPos;

        if(!CommonUtil.getInstance().getVuforiaInitializer().hasCam)
            return RingDetector.Position.LEFT;

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

    private TilerunnerMecanumBot   robot;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime startTimer = new ElapsedTime();

    private Detector det;
    private static ImageTracker tracker;
    private RingDetector.Position ringPos = RingDetector.Position.NONE;

    private double initHdg = 0.0;
    private boolean gyroReady;

    private static PositionOption startPos = START_1;
    private static Field.Alliance alliance = Field.Alliance.RED;

    private static PositionOption parkPos = Route.ParkPos.CENTER_PARK;

    private double delay = 0.0;

    @SuppressWarnings("unused")
    private final boolean useImageLoc  = false;

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