package org.firstinspires.ftc.teamcode.opModes;

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
import org.firstinspires.ftc.teamcode.util.RateChangeLimiter;

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

        mechDrv = (MecanumDriveLRR)(robot.drive);

        robot.setBcm(LynxModule.BulkCachingMode.MANUAL);

        Point2d autonEndPos = robot.getAutonEndPos();
        double autonEndHdg = robot.getAutonEndHdg();
        Pose2d startPose = new Pose2d(autonEndPos.getX(), autonEndPos.getY(), autonEndHdg);
        robot.drive.setPoseEstimate(startPose);
        RobotLog.dd(TAG, "Start Aend fHdg %.2f", Math.toDegrees(autonEndHdg));
        RobotLog.dd(TAG, "Start Pos %s", autonEndPos.toString());

        RobotLog.dd(TAG, "Start mode to %s", robot.leftMotors.get(0).getMode());
        mechDrv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        transRcl =
            new RateChangeLimiter(RobotConstants.MAX_ACCEL/RobotConstants.DT_MAX_IPS);
        double maxDltAng =
            RobotConstants.DT_TRACK_WIDTH * RobotConstants.MAX_ANG_ACCEL / (2*Math.PI);
        rotRcl =
            new RateChangeLimiter(maxDltAng/RobotConstants.DT_MAX_IPS);
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
        if(VERBOSE) RobotLog.dd(TAG, "TEL SHT:%.1f ARM:%.1f INT:%.1f DRV:%.1f",
            shtTime, armTime, intTime, drvTime);
        if(VERBOSE) RobotLog.dd(TAG, "TEL U:%.1f C:%.1f D:%.1f P:%.1f L:%.1f F:%.1f W:%.1f",
            u, c, d, p, L, f, w);
    }

    private void doLogging()
    {
        TelemetryPacket packet = cmu.getTelemetryPacket();
        if(packet == null) packet = new TelemetryPacket();

        packet.put("pos", robot.burr.getEncPos());
        packet.put("spd", robot.burr.getCurSpd());
        packet.put("cmd", cps);
        packet.put("dst", robot.burr.getDist());
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
        boolean bkWhl = gpad2.pressed(ManagedGamepad.Button.L_BUMP);
        boolean bkFwd = gpad2.pressed(ManagedGamepad.Button.R_BUMP);
        boolean drop  = gpad2.just_pressed(ManagedGamepad.Button.L_STICK_BUTTON);

        if(robot.intake != null)
        {
            if(!shoot || Math.abs(intake) >0.05) robot.intake.suck(intake);
            if(drop) robot.intake.toggleDropPos();
        }
        if(robot.loader != null)
        {
            if(shoot)
            {
                if (!lastShoot)
                {
                    RobotLog.dd(TAG, "Starting shoot");
                    robot.loader.setGatePos(Loader.gatePos.OPEN);
                    robot.loader.whlFwd();
                    robot.loader.load(RobotConstants.LD_TELE_PWR);
                    if(RobotConstants.bot == RobotConstants.Chassis.MEC3 &&
                        Math.abs(intake) < 0.05)
                    {
                        robot.intake.suck(RobotConstants.IN_TELE_PWR);
                    }
                }
            }
            else
            {
                if (lastShoot)
                {
                    RobotLog.dd(TAG, "Ending shoot");
                    robot.loader.setGatePos(Loader.gatePos.CLOSE);
                    robot.loader.whlStp();
                    robot.loader.load(0.0);
                    if(RobotConstants.bot == RobotConstants.Chassis.MEC3 &&
                        Math.abs(intake) < 0.05)
                    {
                        robot.intake.suck(0.0);
                    }
                }

                if(bkWhl)
                {
                    robot.loader.whlBak();
                    robot.loader.load(-1.0);
                }
                else if(bkFwd)
                {
                    robot.loader.whlFwd();
                    robot.loader.load(1.0);
                }
                else{
                    robot.loader.whlStp();
                    robot.loader.load(0.0);
                }
            }
            lastShoot = shoot;
        }
    }

    private boolean usePwr = false;
    private final ElapsedTime hldTimer = new ElapsedTime();
    private double shtPwr = 0.0;

    private void controlShooter()
    {
        if(robot.burr == null) return;
        boolean step_up    = gpad2.just_pressed(ManagedGamepad.Button.D_UP);
        boolean step_down  = gpad2.just_pressed(ManagedGamepad.Button.D_DOWN);
        boolean zeroize    = gpad2.just_pressed(ManagedGamepad.Button.D_LEFT);
        boolean normal     = gpad2.just_pressed(ManagedGamepad.Button.D_RIGHT);
        boolean holdZero   = gpad2.pressed(ManagedGamepad.Button.D_LEFT);

        if(holdZero)
        {
            if(hldTimer.seconds() > 2.0)
            {
                usePwr = !usePwr;
                hldTimer.reset();
            }
        }
        else
        {
            hldTimer.reset();
        }

        if(useDist)
        {
            if (step_up   && distance < MAX_DIST) { distance += INCREMENT;  }
            if (step_down && distance > MIN_DIST) { distance -= INCREMENT;  }
            if (normal)                           { distance = FAV_DIST; }
        }

        if(usePwr)
        {
            if (step_up   && shtPwr   < 1.0) { shtPwr += 0.1;  }
            if (step_down && shtPwr   > 0.0) { shtPwr -= 0.1;  }
            if (normal)                      { shtPwr  = 0.5; }
        }

        if      (step_up)   {cps += CPS_INC; cps = Math.min(cps, MAX_CPS);}
        else if (step_down) {cps -= CPS_INC; cps = Math.max(cps, MIN_CPS);}
        else if (normal)    {cps  = RobotConstants.SH_FAV_CPS; }

        if(step_up || step_down || normal)
        {
            if(useDist)     robot.burr.shotSpeed(distance);
            else if(usePwr) robot.burr.shootPower(shtPwr);
            else            robot.burr.shootCps(cps);

            cps = robot.burr.getCmdSpd();
        }
        if (zeroize)
        {
            robot.burr.stop();
            cps = robot.burr.getCmdSpd();
        }

        if (lastKp != pidf.p || lastKd != pidf.d || lastKi != pidf.i || lastKf != pidf.f)
        {
            double volt = robot.getBatteryVoltage();
            vcmpPID = new PIDFCoefficients(pidf.p, pidf.i, pidf.d,
                pidf.f *12.0/volt);
            RobotLog.dd(TAG, "V: %.1f SHTPID: %s", volt, vcmpPID);
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

            mechDrv.followTrajectoryAsync(traj);
            return;
        }

        if (trnS)
        {
            mechDrv.turnAsync(Math.toRadians(180.0));
            return;
        }

        if (canA)
        {
            mechDrv.cancelFollowing();
            return;
        }

        if(mechDrv.isBusy())
        {
            return;
        }

        lr = ishaper.shape(raw_lr, 0.02);
        fb = ishaper.shape(raw_fb, 0.02);
        turn = ishaper.shape(raw_turn, 0.02);

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
            driveInput = new Vector2d(lr, fb).rotated(-poseEstimate.getHeading());
        }
        else
        {
            driveInput = new Vector2d(fb, -lr);
        }

        double maxCPS = RobotConstants.DT_SAF_CPS;
        if(hspd) maxCPS = RobotConstants.DT_MAX_CPS;
        double spdScl = maxCPS/RobotConstants.DT_MAX_CPS;

        driveInput = driveInput.times(spdScl);
        turn = turn * spdScl;

//        Pose2d velPose =
//            new Pose2d(transRcl.calculate(driveInput.norm()), -rotRcl.calculate(turn));
        Pose2d velPose = new Pose2d(driveInput, -turn);

        mechDrv.setWeightedDrivePower(velPose);
    }

    double shtTime;
    double armTime;
    double intTime;
    double drvTime;
    double u, c, d, p, L, f, w;
    private final ElapsedTime oTimer = new ElapsedTime();
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
        drvTime = opTimer.milliseconds();
    }

    @SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, false, false, false);

        initPreStart();

        dashboard.displayText(0, robot.getName() + " is ready");

        // Wait for the game to start (driver presses PLAY)
        while(!isStarted() && !isStopRequested())
        {
            update();
            printTelem();
            doLogging();
            robot.finishFrame();
            robot.waitForTick(20);
        }

        strtV = robot.getBatteryVoltage();
        pidf = RobotConstants.SH_PID;
        vcmpPID = new PIDFCoefficients(pidf.p, pidf.i, pidf.d,
            pidf.f * 12.0/strtV);
        RobotLog.dd(TAG, "SHTPID: %s", vcmpPID);
        robot.burr.setPIDF(vcmpPID);

        RobotLog.dd(TAG, "Mecanum_Driver starting");

        opTimer.reset();
        while (opModeIsActive())
        {
            oTimer.reset();
            update();
            u=opTimer.milliseconds();
            oTimer.reset();
            processControllerInputs();
            c=opTimer.milliseconds();
            oTimer.reset();
            processDriverInputs();
            d=opTimer.milliseconds();
            oTimer.reset();
            printTelem();
            p=opTimer.milliseconds();
            oTimer.reset();
            doLogging();
            L=opTimer.milliseconds();
            oTimer.reset();
            robot.finishFrame();
            f=opTimer.milliseconds();
            oTimer.reset();
            robot.waitForTick(20);
            w=opTimer.milliseconds();
            oTimer.reset();
        }
    }


    double dSpd = 0.0;
    double dStp = 0.1;

    static final double spdScl = Math.sqrt(2.0);
    Input_Shaper ishaper = new Input_Shaper();

    private boolean useField = false;
    private final TilerunnerMecanumBot robot = new TilerunnerMecanumBot();
    private MecanumDriveLRR  mechDrv;

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

    private String lStr = "";
    private String sStr = "";
    private int l = 0;

    private double strtV;

    private RateChangeLimiter transRcl;
    private RateChangeLimiter rotRcl;

    private static final double FAV_DIST = 75;
    private double distance = FAV_DIST;
    private static final String TAG = "SJH_MTD";
}