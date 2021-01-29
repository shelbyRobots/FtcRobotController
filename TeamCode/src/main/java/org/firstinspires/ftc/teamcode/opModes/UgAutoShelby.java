package org.firstinspires.ftc.teamcode.opModes;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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
import org.firstinspires.ftc.teamcode.robot.Lifter;
import org.firstinspires.ftc.teamcode.robot.Loader;
import org.firstinspires.ftc.teamcode.robot.MecanumDriveLRR;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.robot.TilerunnerMecanumBot;
import org.firstinspires.ftc.teamcode.util.AutoTransitioner;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;
import org.firstinspires.ftc.teamcode.util.Point2d;

import java.util.Locale;
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
        int initSleep = 20;
        timer.reset();

        drawRoute();
        while(!isStopRequested() && !isStarted())
        {
            robot.setInInit(true);
            robot.update();

            if(initCycle % 10 == 0)
            {
                double fhdg = robot.getHdg();
                StringBuilder motStr = new StringBuilder("ENCs:");
                for (Map.Entry<String, DcMotorEx> e : robot.motors.entrySet())
                {
                    motStr.append(" ");
                    motStr.append(e.getKey());
                    motStr.append(":");
                    motStr.append(e.getValue().getCurrentPosition());
                }
                dashboard.displayText(6, motStr.toString());
                dashboard.displayText(7,
                    String.format(Locale.US, "FHDG %4.2f", fhdg));
                if (robot.colorSensor != null)
                {
                    int r = robot.colorSensor.red();
                    int g = robot.colorSensor.green();
                    int b = robot.colorSensor.blue();
                    RobotLog.dd(TAG, "RGB = %d %d %d", r, g, b);
                    dashboard.displayText(8,
                        String.format(Locale.US,"RGB %d %d %d", r, g, b));
                }
            }

            initCycle++;

            robot.waitForTick(initSleep);
        }
        robot.setInInit(false);

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
        cps      = pmgr.getCps();
        RobotConstants.SH_FAV_CPS = cps;
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
    }

    private void drawRoute()
    {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.setStrokeWidth(2);
        for(Map.Entry<UgRrRoute.State, Trajectory> entry : ugrr.stateTrajMap.entrySet())
        {
            fieldOverlay.setStroke(ugrr.stateColMap.get(entry.getKey()));
            DashboardUtil.drawSampledPath(fieldOverlay, entry.getValue().getPath());
        }
        ftcdbrd.sendTelemetryPacket(packet);
    }

    double getBatteryVoltage()
    {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor)
        {
            double voltage = sensor.getVoltage();
            if (voltage > 0)
            {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    private FtcDashboard ftcdbrd;
    private void setup()
    {
        ftcdbrd = FtcDashboard.getInstance();
        ftcdbrd.setTelemetryTransmissionInterval(25);

        getPrefs();
        dashboard.displayText(0, "PLEASE WAIT - STARTING - CHECK DEFAULTS");
        dashboard.displayText(1, "Pref Values:");
        dashboard.displayText(2, "Pref BOT: " + robotName);
        dashboard.displayText(3, "Pref Alliance: " + alliance);
        dashboard.displayText(4, "Pref StartPos: " + startPos + " ParkPos: " + parkPos);
        dashboard.displayText(5,
            String.format(Locale.US, "Pref Delay: %.2f Cps: %d", delay, cps));
        dashboard.displayText(6, "HIT A TO ACCEPT VALUES");
        dashboard.displayText(7, "HIT B FOR MENU");
        logData = true;
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

        dashboard.displayText(0, "INITIALIZING - Please wait");
        dashboard.displayText(1, "Prefs/Menu Done");
        dashboard.displayText(2, "NAME: " + robotName);
        dashboard.displayText(3, "ALLIANCE: " + alliance);
        dashboard.displayText(4, "StartPos: " + startPos + " ParkPos: " + parkPos);
        dashboard.displayText(5,
            String.format(Locale.US, "Delay: %.2f Cps: %d", delay, cps));
        dashboard.displayText(6, "");
        dashboard.displayText(7, "");

        final String teleopName = "Mecanum";
        robot = new TilerunnerMecanumBot();

        //Since we only have 5 seconds between Auton and Teleop, automatically load
        //teleop opmode
        RobotLog.dd(TAG, "Setting up auto tele loader : %s", teleopName);
        AutoTransitioner.transitionOnStop(this, teleopName);
        dashboard.displayText(1, "AutoTrans setup");

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

        if(robot.liftyBoi != null)
        {
            robot.liftyBoi.setClampPos(Lifter.ClampPos.CLOSED);
            robot.liftyBoi.setGuidePos(Lifter.GuidePos.OPEN);
            robot.liftyBoi.setLiftPos(Lifter.LiftPos.STOW);
        }
        if(robot.loader != null)
        {
            robot.loader.setGatePos(Loader.gatePos.CLOSE);
        }

        double strtV = getBatteryVoltage();
        RobotLog.dd(TAG, "SHTPID: %s", vcmpPID);
        vcmpPID = new PIDFCoefficients(pidf.p, pidf.i, pidf.d,
            pidf.f *12.0/strtV);

        ugrr = new UgRrRoute(robot, startPos, alliance);
        ShelbyBot.DriveDir startDdir = ShelbyBot.DriveDir.PUSHER;
        robot.setDriveDir(startDdir);
        initHdg = UgRrRoute.startPose.getHeading();
        robot.setInitHdg(initHdg);
        robot.setAlliance(alliance);

        Pose2d ePose = robot.drive.getPoseEstimate();
        robot.setAutonEndPos(new Point2d(ePose.getX(), ePose.getY()));
        robot.setAutonEndHdg(ePose.getHeading());

        dashboard.displayText(0, "GYRO CALIBRATING DO NOT TOUCH OR START");
        if (robot.imu != null)
        {
            gyroReady = robot.calibrateGyro();
        }
        dashboard.displayText(0, "GYRO CALIBATED: " + gyroReady);
        dashboard.displayText(1, "Robot Inited");

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
        RobotLog.ii(TAG, "CPS      %s", cps);
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

        dashboard.displayText(0, "READY TO START");
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

        double shootWait = 2.5;
        if(UgRrRoute.shootPS) shootWait = 1.5;
//        double intakeRunWait = 0.6;
//        double intakePauseWait = 0.6;
        ElapsedTime shootTimer = new ElapsedTime();
        ElapsedTime intakeTimer = new ElapsedTime();

        RobotLog.ii(TAG, "Action SCAN_IMAGE");
        doScan();
        Pose2d ePose = robot.drive.getPoseEstimate();

        for(Map.Entry<UgRrRoute.State, Trajectory> entry : ugrr.stateTrajMap.entrySet())
        {
            timer.reset();
            UgRrRoute.State state = entry.getKey();
            Trajectory traj = entry.getValue();
            if(state == UgRrRoute.State.IDLE) break;

            ugrr.setState(state);

            RobotLog.ii(TAG, "Driving trajectory %s", state);

            double mechTimeOut = 0.5;
            if(state == UgRrRoute.State.REVERSE || state == UgRrRoute.State.WOB2)
            {
                mechTimeOut = 1.0;
            }

            mechDrv.setFollowerTimeout(mechTimeOut);

            mechDrv.followTrajectoryAsync(traj);
            while(opModeIsActive() && !isStopRequested() &&
                mechDrv.isBusy())
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

            if((state == UgRrRoute.State.SHOOT && !UgRrRoute.shootPS) &&
               ((state == UgRrRoute.State.SHT1 ||
                 state == UgRrRoute.State.SHT1 ||
                 state == UgRrRoute.State.SHT3)   &&  UgRrRoute.shootPS))
            {
                shootTimer.reset();
                intakeTimer.reset();

                while(opModeIsActive() && !isStopRequested() &&
                    shootTimer.seconds() < shootWait)
                {
                    robot.update();
                    robot.waitForTick(20);
                }

                if(robot.burr != null  &&
                    (state == UgRrRoute.State.SHOOT ||
                        state == UgRrRoute.State.SHT3)) robot.burr.stop();
                if(robot.loader != null)
                {
                    robot.loader.load(0.0);
                    robot.loader.setGatePos(Loader.gatePos.CLOSE);
                    robot.loader.whlStp();
                }
                if(robot.intake != null)
                {
                    robot.intake.suck(0.0);
                }
            }
        }

        ePose = robot.drive.getPoseEstimate();
        robot.setAutonEndPos(new Point2d(ePose.getX(), ePose.getY()));
        robot.setAutonEndHdg(ePose.getHeading());
        RobotLog.dd(TAG, "Finished auton segments at X:%.2f Y:%.2f H:%.2f",
            ePose.getX(), ePose.getY(), Math.toDegrees(ePose.getHeading()));

        if(robot.burr != null) robot.burr.stop();
        if(robot.loader != null)
        {
            robot.loader.load(0.0);
            robot.loader.setGatePos(Loader.gatePos.CLOSE);
            robot.loader.whlStp();
        }
        if(robot.intake != null)
        {
            robot.intake.suck(0.0);
        }

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

        if(startPos == START_1)
        {
            if(ringPos == RingDetector.Position.CENTER)
            {
                ugrr.stateTrajMap.put(UgRrRoute.State.DROP1,  ugrr.tDOB);
                ugrr.stateTrajMap.put(UgRrRoute.State.SHOOT,  ugrr.tSOB);
                if(UgRrRoute.GO_FOR_TWO)
                {
                    ugrr.stateTrajMap.put(UgRrRoute.State.DROP2, ugrr.tDIB);
                    ugrr.stateTrajMap.put(UgRrRoute.State.PARK, ugrr.tPIB);
                }
            }
            else if(ringPos == RingDetector.Position.RIGHT)
            {
                ugrr.stateTrajMap.put(UgRrRoute.State.DROP1,  ugrr.tDOC);
                ugrr.stateTrajMap.put(UgRrRoute.State.SHOOT,  ugrr.tSOC);
                if(UgRrRoute.GO_FOR_TWO)
                {
                    ugrr.stateTrajMap.put(UgRrRoute.State.DROP2, ugrr.tDIC);
                    ugrr.stateTrajMap.put(UgRrRoute.State.PARK, ugrr.tPIC);
                }
            }
        }
        else
        {
            RobotLog.dd(TAG, "Setup start2");
            if(ringPos == RingDetector.Position.CENTER)
            {
                ugrr.stateTrajMap.put(UgRrRoute.State.DROP1,  ugrr.tDIB);
                ugrr.stateTrajMap.put(UgRrRoute.State.SHOOT,  ugrr.tSIB);
                if(UgRrRoute.GO_FOR_TWO)
                {
                    ugrr.stateTrajMap.put(UgRrRoute.State.DROP2, ugrr.tDOB);
                    ugrr.stateTrajMap.put(UgRrRoute.State.PARK, ugrr.tPOB);
                }
            }
            else if(ringPos == RingDetector.Position.RIGHT)
            {
                ugrr.stateTrajMap.put(UgRrRoute.State.DROP1,  ugrr.tDIC);
                ugrr.stateTrajMap.put(UgRrRoute.State.SHOOT,  ugrr.tSIC);
                if(UgRrRoute.GO_FOR_TWO)
                {
                    ugrr.stateTrajMap.put(UgRrRoute.State.DROP2, ugrr.tDOC);
                    ugrr.stateTrajMap.put(UgRrRoute.State.PARK, ugrr.tPOC);
                }
            }
        }
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
                // //noinspection ConstantConditions
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

        dashboard.displayText(1, "POS: " + ringPos);

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
    private final MecanumDriveLRR mechDrv = (MecanumDriveLRR)(robot.drive);

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

    public static PIDFCoefficients pidf = RobotConstants.SH_PID;
    private PIDFCoefficients vcmpPID;

    private int cps = 1820;

    @SuppressWarnings("unused")
    private final boolean useImageLoc  = false;

    private static final boolean useLight = true;
    private static final boolean usePhone = false;

    private final ExecutorService es = Executors.newSingleThreadExecutor();

    private String robotName = "MEC2";
    private static final String TAG = "SJH_RRA";

    private UgRrRoute ugrr;
}