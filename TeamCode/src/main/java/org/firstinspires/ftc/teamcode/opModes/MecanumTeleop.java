package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    private void printTelem()
    {
        String cntStr = String.format(Locale.US,"CNTS: %d %d %d %d",
                cnts[0], cnts[1], cnts[2], cnts[3]);
        String velStr = String.format(Locale.US,"VELS: %d %d %d %d",
                (int)vels[0], (int)vels[1], (int)vels[2], (int)vels[3]);
        String posStr = String.format(Locale.US, "X:%.2f Y:%.2f H:%.2f",
            poseEstimate.getX(), poseEstimate.getY(), Math.toDegrees(poseEstimate.getHeading()));

        dashboard.displayPrintf(l++, "Dir %s", robot.getDriveDir());
        dashboard.displayText  (l++, posStr);
        dashboard.displayText  (l++, cntStr);
        dashboard.displayText  (l++, velStr);
        dashboard.displayText  (l++, lStr);
        dashboard.displayPrintf(l++, sStr);
        dashboard.displayPrintf(l++,"L_IN %4.2f L %4.2f", raw_lr_x, lr_x);
        dashboard.displayPrintf(l++,"R_IN %4.2f R %4.2f", raw_fb_y, fb_y);
        dashboard.displayPrintf(l++,"T_IN %4.2f T %4.2f", raw_turn, turn);
        RobotLog.dd(TAG, sStr);
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
                if (lastShoot != shoot) RobotLog.dd(TAG, "Starting shoot");
            }
            else
            {
                robot.loader.setGatePos(Loader.gatePos.CLOSE);
                robot.loader.whlStp();
                robot.loader.load(0.0);
                if (lastShoot != shoot) RobotLog.dd(TAG, "Ending shoot");
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


//        if (step_up && distance < MAX_DIST) {
//            distance += INCREMENT;
//            robot.burr.shotSpeed(distance);
//        }
//        if (step_down && distance > MIN_DIST) {
//            distance -= INCREMENT;
//            robot.burr.shotSpeed(distance);
//        }
        if (step_up) {cps += CPS_INC; cps = Math.min(cps, MAX_CPS);}
        else if (step_down) {cps -=  CPS_INC; cps = Math.max(cps, MIN_CPS);}
        else if (normal)
        {
            //distance = FAV_DIST;
            //robot.burr.shotSpeed(distance);
            cps = RobotConstants.SH_FAV_CPS;
        }
        if(step_up || step_down || normal) robot.burr.shootCps(cps);
        if (zeroize){
            robot.burr.stop();
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

        raw_lr_x =  gpad1.value(ManagedGamepad.AnalogInput.R_STICK_X);
        raw_fb_y = -gpad1.value(ManagedGamepad.AnalogInput.R_STICK_Y);
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

        lr_x = ishaper.shape(raw_lr_x, 0.1);
        fb_y = ishaper.shape(raw_fb_y, 0.1);
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
                lr_x = lft ? -dSpd : rgt ? dSpd : 0.0;
                fb_y = bak ? -dSpd : fwd ? dSpd : 0.0;
                if ((lft || rgt) && (fwd || bak))
                {
                    lr_x /= spdScl;
                    fb_y /= spdScl;
                }
            }
        }

        Vector2d driveInput;
        if(useField)
        {
            // Rotate input vector by the inverse of current bot heading
            driveInput = new Vector2d(-lr_x, -fb_y).rotated(-poseEstimate.getHeading());
        }
        else
        {
            driveInput = new Vector2d(fb_y, -lr_x);
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

    private void processControllerInputs()
    {
        gpad2.update();
        controlShooter();
        controlArm();
        controlIntake();
    }

    private void processDriverInputs()
    {
        gpad1.update();
        controlDrive();
    }

    @SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, false, false, false);

        initPreStart();

        dashboard.displayPrintf(0, "%s is ready", robot.getName());

        // Wait for the game to start (driver presses PLAY)
        while(!isStarted() && !isStopRequested())
        {
            update();
            printTelem();
            robot.waitForTick(10);
        }

        RobotLog.dd(TAG, "Mecanum_Driver starting");

        while (opModeIsActive())
        {
            update();
            processControllerInputs();
            processDriverInputs();

            printTelem();
            robot.waitForTick(10);
        }
    }


    double dSpd = 0.0;
    double dStp = 0.1;

    static final double spdScl = Math.sqrt(2.0);
    Input_Shaper ishaper = new Input_Shaper();

    private boolean useField = false;
    private final TilerunnerMecanumBot robot = new TilerunnerMecanumBot();

    Pose2d shtPose = UgRrRoute.shtPose;

    double raw_lr_x;
    double raw_fb_y;
    double raw_turn;
    double lr_x;
    double fb_y;
    double turn;

    Pose2d poseEstimate;

    int[] cnts = {0,0,0,0};
    double[] vels = {0,0,0,0};
    double hdg = 0;

    private static final double MIN_DIST = 60;
    private static final double MAX_DIST = 136;
    private static final double INCREMENT = 6;
    private double cps = 0.0;
    private static final double MIN_CPS = 0.0;
    //6000RPM/60 for RPS * 28 CPR for 1:1 goBilda motor = 2800
    private static final double MAX_CPS = (6000.0/60.0) * 28;
    private static final double CPS_INC = 10.0;

    private String lStr = "";
    private String sStr = "";
    private int l = 0;

    private static final double FAV_DIST = 75;
    private double distance = FAV_DIST;
    private static final String TAG = "SJH_MTD";
}