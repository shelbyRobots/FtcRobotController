package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.MecanumDriveLRR;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.robot.TilerunnerMecanumBot;
import org.firstinspires.ftc.teamcode.util.Point2d;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "drive")
public class SbhTest extends InitLinearOpMode
{
    private final TilerunnerMecanumBot robot = new TilerunnerMecanumBot();

    private static final String TAG = "SJH_MTD";

    MecanumDriveLRR drive = null;

    public static enum DROP_POS
    {
        DROPA,
        DROPB,
        DROPC
    }

    public static DROP_POS dpos = DROP_POS.DROPB;

    Pose2d wA1Pose = new Pose2d(4, -59,0);
    Pose2d wA2Pose = new Pose2d(10, -44,-Math.toRadians(90));
    Pose2d wB1Pose = new Pose2d(24, -48, Math.toRadians(45));
    Pose2d wB2Pose = new Pose2d(24, -30,-Math.toRadians(20));
    Pose2d wC1Pose = new Pose2d(48, -59,0);
    Pose2d wC2Pose = new Pose2d(54, -46,-Math.toRadians(45));

    Pose2d srtPose = new Pose2d(-61.5,-44,0);
    Pose2d dogPose = new Pose2d(0,-59,0);
    Pose2d wAcPose = new Pose2d(-4.0,-59,0);
    Pose2d shtPose = new Pose2d(-2.0,-36,0);
    Pose2d md1Pose = new Pose2d(-10.0, -52.0, 0);
    Pose2d md2Pose = new Pose2d(-48.0, -52.0, 0);
    Pose2d walPose = new Pose2d(-61.0,-40,0);
    Pose2d w2gPose = new Pose2d(-61.0,-24,0);
    Pose2d prkPose = new Pose2d(10.0,-36,0);

    private void initPreStart()
    {
        initCommon(this, false, false, false, false);
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

        ShelbyBot.curOpModeType = ShelbyBot.OpModeType.AUTO;

        /* Initialize the hardware variables. */
        RobotLog.dd(TAG, "Initialize robot");

        robot.init(this, chas, true);

        robot.setBcm(LynxModule.BulkCachingMode.AUTO);

        if(robot.drive instanceof MecanumDriveLRR)
            ((MecanumDriveLRR)(robot.drive)).setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive = (MecanumDriveLRR)(robot.drive);
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        initPreStart();
        Pose2d w1dPose;
        Pose2d w2dPose;

        switch (dpos)
        {
            case DROPB:
                w1dPose = wB1Pose;
                w2dPose = wB2Pose;
                break;
            case DROPC:
                w1dPose = wC1Pose;
                w2dPose = wC2Pose;
                break;
            case DROPA:
            default:
                w1dPose = wA1Pose;
                w2dPose = wA2Pose;
        }

        drive.setPoseEstimate(srtPose);

        Trajectory t1 = drive.trajectoryBuilder(srtPose)
            .splineTo(dogPose.vec(), dogPose.getHeading())
            .splineTo(w1dPose.vec(), w1dPose.getHeading()).build();
        Trajectory t2 = drive.trajectoryBuilder(t1.end(), Math.toRadians(180))
            .splineToLinearHeading(wAcPose,180).build();
        Trajectory t3 = drive.trajectoryBuilder(t2.end())
            .strafeTo(shtPose.vec()).build();
        Trajectory t6 = drive.trajectoryBuilder(t3.end(), Math.toRadians(-90))
            .splineToConstantHeading(md1Pose.vec(),Math.toRadians(180))
            .splineToConstantHeading(md2Pose.vec(),Math.toRadians(180))
            .splineToConstantHeading(walPose.vec(),Math.toRadians(90))
            .splineToConstantHeading(w2gPose.vec(),Math.toRadians(90)).build();
//        Trajectory t5 = drive.trajectoryBuilder(t4.end())
//            .strafeTo(walPose.vec()).build();
//        Trajectory t6 = drive.trajectoryBuilder(t5.end())
//            .strafeTo(w2gPose.vec()).build();
        Trajectory t7 = drive.trajectoryBuilder(t6.end())
            .splineTo(w2dPose.vec(), w2dPose.getHeading()).build();
        Trajectory t8 = drive.trajectoryBuilder(t7.end(),true)
            .splineToLinearHeading(prkPose, 0).build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(t1);
        drive.followTrajectory(t2);
        drive.followTrajectory(t3); sleep(2000);
        //drive.followTrajectory(t4);
        //drive.followTrajectory(t5);
        drive.followTrajectory(t6);
        drive.followTrajectory(t7);
        drive.followTrajectory(t8);

        Pose2d ePose = drive.getPoseEstimate();
        robot.setAutonEndPos(new Point2d(ePose.getX(), ePose.getY()));
        robot.setAutonEndHdg(ePose.getHeading());
    }
}
