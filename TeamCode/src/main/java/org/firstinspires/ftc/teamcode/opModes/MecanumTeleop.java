package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.field.UgRrRoute;
import org.firstinspires.ftc.teamcode.robot.Lifter;
import org.firstinspires.ftc.teamcode.robot.Loader;
import org.firstinspires.ftc.teamcode.robot.MecanumDriveLRR;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.robot.TilerunnerMecanumBot;
import org.firstinspires.ftc.teamcode.util.Input_Shaper;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;
import org.firstinspires.ftc.teamcode.util.Point2d;

import java.util.Locale;

@Config
@TeleOp(name = "Mecanum")
//@Disabled
public class MecanumTeleop extends InitLinearOpMode
{
    private void initPreStart()
    {
        String robotName = pmgr.getBotName();
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

        ShelbyBot.OpModeType prevOpModeType = ShelbyBot.curOpModeType;
        ShelbyBot.curOpModeType = ShelbyBot.OpModeType.TELE;

        /* Initialize the hardware variables. */
        RobotLog.dd(TAG, "Initialize robot");

        boolean initSensors = prevOpModeType != ShelbyBot.OpModeType.AUTO;
        RobotLog.dd(TAG, "Prev opmode type=%s. initSensors=%s",
            prevOpModeType, initSensors);
        robot.init(this, chas, initSensors);

        robot.setBcm(LynxModule.BulkCachingMode.MANUAL);

        Point2d autonEndPos = robot.getAutonEndPos();
        double autonEndHdg = robot.getAutonEndHdg();
        Pose2d startPose = new Pose2d(autonEndPos.getX(), autonEndPos.getY(), autonEndHdg);
        robot.drive.setPoseEstimate(startPose);
        RobotLog.dd(TAG, "Start Aend fHdg %.2f", Math.toDegrees(autonEndHdg));
        RobotLog.dd(TAG, "Start Pos %s", autonEndPos.toString());

        RobotLog.dd(TAG, "Start mode to %s", robot.leftMotors.get(0).getMode());
        if(robot.drive instanceof MecanumDriveLRR)
            ((MecanumDriveLRR)(robot.drive)).setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dbd = FtcDashboard.getInstance();
        dbd.setTelemetryTransmissionInterval(20);
    }

    private void update()
    {
        robot.update();

        l = 0;

        cnts=robot.getCnts();
        vels=robot.getVels();
        hdg=robot.getHdg();

        poseEstimate = robot.drive.getPoseEstimate();

        if(robot.liftyBoi != null) lStr = robot.liftyBoi.toString();
        if(robot.burr != null)     sStr = robot.burr.toString();
    }

    private static final boolean VERBOSE = true;
    private void printTelem()
    {
        String cntStr = String.format(Locale.US,"CNTS: %d %d %d %d",
                cnts[0], cnts[1], cnts[2], cnts[3]);
        String velStr = String.format(Locale.US,"VELS: %d %d %d %d",
                (int)vels[0], (int)vels[1], (int)vels[2], (int)vels[3]);
        String posStr = String.format(Locale.US, "X:%.2f Y:%.2f H:%.2f",
            poseEstimate.getX(), poseEstimate.getY(), Math.toDegrees(poseEstimate.getHeading()));

        dashboard.displayText(l++, "Dir " + robot.getDriveDir());
        dashboard.displayText(l++, posStr);
        dashboard.displayText(l++, cntStr);
        dashboard.displayText(l++, velStr);
        dashboard.displayText(l++, lStr);
        dashboard.displayText(l++, sStr);
        dashboard.displayText(l++, String.format(Locale.US,"V:%.1f SP:%s", strtV, vcmpPID));
        dashboard.displayText(l++, String.format(Locale.US,"L_IN %4.2f L %4.2f", raw_lr, lr));
        dashboard.displayText(l++, String.format(Locale.US,"R_IN %4.2f R %4.2f", raw_fb, fb));
        dashboard.displayText(l++, String.format(Locale.US,"T_IN %4.2f T %4.2f", raw_turn, turn));
        if(VERBOSE) RobotLog.dd(TAG, sStr);
        if(VERBOSE) RobotLog.dd(TAG, "TEL SHT:%.2f ARM:%.2f INT:%.2f DRV%.2f",
            shtTime, armTime, intTime, drvTime);
    }

    private void doLogging()
    {
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("pos", robot.burr.getEncPos());
        packet.put("spd", robot.burr.getCurSpd());
        packet.put("cmd", cps);
        packet.put("dst", robot.burr.getDist());
        if(!RobotConstants.logDrive) dbd.sendTelemetryPacket(packet);

        String shtStr = robot.burr.toString();
        RobotLog.dd(TAG, shtStr);
    }



    private void controlArmElev()
    {
        if(robot.liftyBoi == null) return;
        double lftPwr = -gpad2.value(ManagedGamepad.AnalogInput.R_STICK_Y);
        boolean stow = gpad2.just_pressed(ManagedGamepad.Button.A);
        boolean grab = gpad2.just_pressed(ManagedGamepad.Button.B);
        boolean hold = gpad2.just_pressed(ManagedGamepad.Button.X);
        boolean drop = gpad2.just_pressed(ManagedGamepad.Button.Y);

        if      (stow) robot.liftyBoi.setLiftPos(Lifter.LiftPos.STOW);
        else if (grab) robot.liftyBoi.setLiftPos(Lifter.LiftPos.GRAB);
        else if (hold) robot.liftyBoi.setLiftPos(Lifter.LiftPos.HOLD);
        else if (drop) robot.liftyBoi.setLiftPos(Lifter.LiftPos.DROP);
        else robot.liftyBoi.setLiftSpd(lftPwr);
    }

    private void controlGripper()
    {
        if(robot.liftyBoi == null) return;
        boolean toggleGrp = gpad2.just_pressed(ManagedGamepad.Button.L_TRIGGER);
        if(toggleGrp) robot.liftyBoi.toggleClampPos();
    }

    boolean lastShoot = false;
    private void controlIntake()
    {
        double intake = -gpad2.value(ManagedGamepad.AnalogInput.L_STICK_Y);
        boolean shoot = gpad2.pressed(ManagedGamepad.Button.R_TRIGGER);

        if(robot.intake != null) robot.intake.suck(intake);
        if(robot.loader != null)
        {
            if(shoot)
            {
                robot.loader.setGatePos(Loader.gatePos.OPEN);
                robot.loader.whlFwd();
                robot.loader.load(1.0);
                if (!lastShoot) RobotLog.dd(TAG, "Starting shoot");
            }
            else
            {
                robot.loader.setGatePos(Loader.gatePos.CLOSE);
                robot.loader.whlStp();
                robot.loader.load(0.0);
                if (lastShoot) RobotLog.dd(TAG, "Ending shoot");
            }
            lastShoot = shoot;
        }
    }

    private void controlShooter()
    {
        if(robot.burr == null) return;
        boolean step_up    = gpad2.just_pressed(ManagedGamepad.Button.D_UP);
        boolean step_down  = gpad2.just_pressed(ManagedGamepad.Button.D_DOWN);
        boolean zeroize    = gpad2.just_pressed(ManagedGamepad.Button.D_LEFT);
        boolean normal     = gpad2.just_pressed(ManagedGamepad.Button.D_RIGHT);

        if(useDist)
        {
            if (step_up && distance < MAX_DIST)
            {
                distance += INCREMENT;
                robot.burr.shotSpeed(distance);
            }
            if (step_down && distance > MIN_DIST)
            {
                distance -= INCREMENT;
                robot.burr.shotSpeed(distance);
            }
            cps = robot.burr.getCmdSpd();
        }
        if (step_up) {cps += CPS_INC; cps = Math.min(cps, MAX_CPS);}
        else if (step_down) {cps -=  CPS_INC; cps = Math.max(cps, MIN_CPS);}
        else if (normal)
        {
            //distance = FAV_DIST;
            //robot.burr.shotSpeed(distance);
            cps = RobotConstants.SH_FAV_CPS;
        }
        if(step_up || step_down || normal) robot.burr.shootCps(cps);
        if (zeroize)
        {
            robot.burr.stop();
            cps = robot.burr.getCmdSpd();
        }

        if (lastKp != pidf.p || lastKd != pidf.d || lastKi != pidf.i || lastKf != pidf.f)
        {
            vcmpPID = new PIDFCoefficients(pidf.p, pidf.i, pidf.d,
                pidf.f *12.0/strtV);
            RobotLog.dd(TAG, "V: %.1f SHTPID: %s", vcmpPID);
            robot.burr.setPIDF(vcmpPID);

            lastKp = pidf.p;
            lastKi = pidf.i;
            lastKd = pidf.d;
            lastKf = pidf.f;
        }

    }

    private void controlArm()
    {
        controlArmElev();
        controlGripper();
    }

    private void controlDrive()
    {
        if (robot.leftMotors.size() == 0 && robot.rightMotors.size() == 0) return;

        raw_lr =  gpad1.value(ManagedGamepad.AnalogInput.R_STICK_X);
        raw_fb = -gpad1.value(ManagedGamepad.AnalogInput.R_STICK_Y);
        raw_turn =  gpad1.value(ManagedGamepad.AnalogInput.L_STICK_X);

        boolean rgt  = gpad1.pressed(ManagedGamepad.Button.D_RIGHT);
        boolean lft  = gpad1.pressed(ManagedGamepad.Button.D_LEFT);
        boolean fwd  = gpad1.pressed(ManagedGamepad.Button.D_UP);
        boolean bak  = gpad1.pressed(ManagedGamepad.Button.D_DOWN);
        boolean incr = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
        boolean decr = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);
        boolean hspd = gpad1.pressed(ManagedGamepad.Button.R_TRIGGER);
        boolean dtrn = gpad1.pressed(ManagedGamepad.Button.L_TRIGGER);
        boolean tglF = gpad1.just_pressed(ManagedGamepad.Button.Y);
        boolean drvS = gpad1.just_pressed(ManagedGamepad.Button.A);
        boolean trnS = gpad1.just_pressed(ManagedGamepad.Button.B);
        boolean canA = gpad1.just_pressed(ManagedGamepad.Button.X);

        if (tglF) useField = !useField;

        if (drvS)
        {
            Trajectory traj = new TrajectoryBuilder(poseEstimate,
                RobotConstants.defVelConstraint, RobotConstants.defAccelConstraint)
                .lineToLinearHeading(shtPose)
                .build();

            ((MecanumDriveLRR)(robot.drive)).followTrajectoryAsync(traj);
            return;
        }

        if (trnS)
        {
            ((MecanumDriveLRR)(robot.drive)).turnAsync(Math.toRadians(180.0));
            return;
        }

        if (canA)
        {
            ((MecanumDriveLRR)(robot.drive)).cancelFollowing();
            return;
        }

        if(((MecanumDriveLRR)(robot.drive)).isBusy())
        {
            return;
        }

        lr = ishaper.shape(raw_lr, 0.1);
        fb = ishaper.shape(raw_fb, 0.1);
        turn = ishaper.shape(raw_turn, 0.1);

        if      (incr) dSpd += dStp;
        else if (decr) dSpd -= dStp;
        dSpd = Range.clip(dSpd, 0.0, 1.0);

        if (lft || rgt || fwd || bak)
        {
            if((lft || rgt) && dtrn)
            {
                turn = lft ? -dSpd : dSpd;
            }
            else
            {
                lr = lft ? -dSpd : rgt ? dSpd : 0.0;
                fb = bak ? -dSpd : fwd ? dSpd : 0.0;
                if ((lft || rgt) && (fwd || bak))
                {
                    lr /= spdScl;
                    fb /= spdScl;
                }
            }
        }

        Vector2d driveInput;
        if(useField)
        {
            // Rotate input vector by the inverse of current bot heading
            driveInput = new Vector2d(-lr, -fb).rotated(-poseEstimate.getHeading());
        }
        else
        {
            driveInput = new Vector2d(fb, -lr);
        }

        double maxCPS = RobotConstants.DT_SAF_CPS;
        if(hspd) maxCPS = RobotConstants.DT_MAX_CPS;
        double spdScl = maxCPS/RobotConstants.DT_MAX_CPS;
        if(robot.drive instanceof MecanumDriveLRR)
            ((MecanumDriveLRR)(robot.drive)).setWeightedDrivePower(
                new Pose2d(
                    driveInput.getX() * spdScl,
                    driveInput.getY() * spdScl,
                    -turn *spdScl
                )
            );
    }

    double strTime;
    double shtTime;
    double armTime;
    double intTime;
    double drvTime;
    private final ElapsedTime opTimer = new ElapsedTime();
    private void processControllerInputs()
    {
        gpad2.update();
        opTimer.reset();
        controlShooter();
        shtTime = opTimer.milliseconds();
        opTimer.reset();
        controlArm();
        armTime = opTimer.milliseconds();
        opTimer.reset();
        controlIntake();
        intTime = opTimer.milliseconds();
        opTimer.reset();
    }

    private void processDriverInputs()
    {
        gpad1.update();
        controlDrive();
        drvTime = opTimer.milliseconds() - intTime;
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

    @SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, false, false, false);

        initPreStart();

        strtV = getBatteryVoltage();
        vcmpPID = new PIDFCoefficients(pidf.p, pidf.i, pidf.d,
            pidf.f * 12.0/strtV);
        RobotLog.dd(TAG, "SHTPID: %s", vcmpPID);
        robot.burr.setPIDF(vcmpPID);

        dashboard.displayText(0, robot.getName() + " is ready");
        doLogging();

        // Wait for the game to start (driver presses PLAY)
        while(!isStarted() && !isStopRequested())
        {
            update();
            printTelem();
            robot.waitForTick(10);
        }

        RobotLog.dd(TAG, "Mecanum_Driver starting");

        opTimer.reset();
        while (opModeIsActive())
        {
            strTime = opTimer.milliseconds();
            update();
            processControllerInputs();
            processDriverInputs();

            printTelem();
            doLogging();
            robot.waitForTick(20);
        }
    }


    double dSpd = 0.0;
    double dStp = 0.1;

    static final double spdScl = Math.sqrt(2.0);
    Input_Shaper ishaper = new Input_Shaper();

    private boolean useField = false;
    private final TilerunnerMecanumBot robot = new TilerunnerMecanumBot();

    Pose2d shtPose = UgRrRoute.shtPose;

    double raw_lr;
    double raw_fb;
    double raw_turn;
    double lr;
    double fb;
    double turn;

    Pose2d poseEstimate;

    int[] cnts = {0,0,0,0};
    double[] vels = {0,0,0,0};
    double hdg = 0;

    private static final boolean useDist = false;
    private static final double MIN_DIST = 60;
    private static final double MAX_DIST = 136;
    private static final double INCREMENT = 6;
    private double cps = 0.0;
    private static final double MIN_CPS = 0.0;
    //6000RPM/60 for RPS * 28 CPR for 1:1 goBilda motor = 2800
    private static final double MAX_CPS = (6000.0/60.0) * 28;
    private static final double CPS_INC = 10.0;

    double lastKp = RobotConstants.SH_PID.p;
    double lastKi = RobotConstants.SH_PID.i;
    double lastKd = RobotConstants.SH_PID.d;
    double lastKf = RobotConstants.SH_PID.f;

    public static PIDFCoefficients pidf = RobotConstants.SH_PID;
    private PIDFCoefficients vcmpPID;

    private FtcDashboard dbd;

    private String lStr = "";
    private String sStr = "";
    private int l = 0;

    private double strtV;

    private static final double FAV_DIST = 75;
    private double distance = FAV_DIST;
    private static final String TAG = "SJH_MTD";
}