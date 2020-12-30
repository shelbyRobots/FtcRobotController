package org.firstinspires.ftc.teamcode.field;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.MecanumDriveLRR;

import java.util.EnumMap;

public class UgRrRoute
{
  MecanumDriveLRR drive;
  PositionOption startPos;
  PositionOption parkChoice;
  Field.Alliance alliance;

  private final TrajectoryVelocityConstraint defVelLim = DriveConstants.defVelConstraint;
  private final TrajectoryAccelerationConstraint defAccelLim = DriveConstants.defAccelConstraint;

  public enum State
  {
    DROP1,
    CLEAR1,
    SHOOT,
    RETMID,
    WALL,
    WOB2,
    DROP2,
    PARK,
    IDLE
  }

  public EnumMap<State, Trajectory> stateTrajMap = new EnumMap<>(State.class);

  Pose2d wA1Pose = new Pose2d(4, -59,0);
  Pose2d wA2Pose = new Pose2d(10, -44,-Math.toRadians(90));
  Pose2d wB1Pose = new Pose2d(24, -48, Math.toRadians(45));
  Pose2d wB2Pose = new Pose2d(24, -30,-Math.toRadians(20));
  Pose2d wC1Pose = new Pose2d(48, -59,0);
  Pose2d wC2Pose = new Pose2d(54, -46,-Math.toRadians(45));

  public static final Pose2d srtPose = new Pose2d(-61.5,-44,0);
  Pose2d dogPose = new Pose2d(0,-59,0);
  Pose2d wAcPose = new Pose2d(-4.0,-59,0);
  Pose2d shtPose = new Pose2d(-2.0,-36,0);
  Pose2d md1Pose = new Pose2d(-10.0, -52.0, 0);
  Pose2d walPose = new Pose2d(-60.0,-52,0);
  Pose2d w2gPose = new Pose2d(-61.0,-24,0);
  Pose2d prkPose = new Pose2d(10.0,-36,0);

  public Trajectory drop1a;
  public Trajectory drop1b;
  public Trajectory drop1c;
  public Trajectory clr1a;
  public Trajectory clr1b;
  public Trajectory clr1c;
  public Trajectory shoot;
  public Trajectory midret;
  public Trajectory wall;
  public Trajectory wob2;
  public Trajectory drop2a;
  public Trajectory drop2b;
  public Trajectory drop2c;
  public Trajectory parka;
  public Trajectory parkb;
  public Trajectory parkc;

  private static final String TAG = "SJH_URR";

  public UgRrRoute(MecanumDriveLRR drive,
                   PositionOption startPos,
                   Field.Alliance alliance)
  {
    this.drive        = drive;
    this.startPos     = startPos;
    this.alliance     = alliance;

    drive.setPoseEstimate(srtPose);
    drop1a = new TrajectoryBuilder(srtPose, defVelLim, defAccelLim)
        .splineTo(dogPose.vec(), dogPose.getHeading())
        .splineTo(wA1Pose.vec(), wA1Pose.getHeading()).build();
    drop1b = new TrajectoryBuilder(srtPose, defVelLim, defAccelLim)
        .splineTo(dogPose.vec(), dogPose.getHeading())
        .splineTo(wB1Pose.vec(), wB1Pose.getHeading()).build();
    drop1c = new TrajectoryBuilder(srtPose, defVelLim, defAccelLim)
        .splineTo(dogPose.vec(), dogPose.getHeading())
        .splineTo(wC1Pose.vec(), wC1Pose.getHeading()).build();
    clr1a = new TrajectoryBuilder(drop1a.end(), defVelLim, defAccelLim)
        .splineToLinearHeading(wAcPose,0).build();
    clr1b = new TrajectoryBuilder(drop1b.end(), defVelLim, defAccelLim)
        .splineToLinearHeading(wAcPose,0).build();
    clr1c = new TrajectoryBuilder(drop1c.end(), defVelLim, defAccelLim)
        .splineToLinearHeading(wAcPose,0).build();
    shoot = new TrajectoryBuilder(clr1a.end(), defVelLim, defAccelLim)
        .strafeTo(shtPose.vec()).build();
    midret = new TrajectoryBuilder(shoot.end(), defVelLim, defAccelLim)
        .strafeTo(md1Pose.vec()).build();
    wall = new TrajectoryBuilder(midret.end(), defVelLim, defAccelLim)
        .strafeTo(walPose.vec()).build();
    wob2 = new TrajectoryBuilder(wall.end(), defVelLim, defAccelLim)
        .strafeTo(w2gPose.vec()).build();
    drop2a = new TrajectoryBuilder(wob2.end(), defVelLim, defAccelLim)
        .splineTo(wA2Pose.vec(), wA2Pose.getHeading()).build();
    drop2b = new TrajectoryBuilder(wob2.end(), defVelLim, defAccelLim)
        .splineTo(wB2Pose.vec(), wB2Pose.getHeading()).build();
    drop2c = new TrajectoryBuilder(wob2.end(), defVelLim, defAccelLim)
        .splineTo(wC2Pose.vec(), wC2Pose.getHeading()).build();
    parka = new TrajectoryBuilder(drop2a.end(),true, defVelLim, defAccelLim)
        .splineToLinearHeading(prkPose, 0).build();
    parkb = new TrajectoryBuilder(drop2b.end(),true, defVelLim, defAccelLim)
        .splineToLinearHeading(prkPose, 0).build();
    parkc = new TrajectoryBuilder(drop2c.end(),true, defVelLim, defAccelLim)
        .splineToLinearHeading(prkPose, 0).build();

    stateTrajMap.put(State.DROP1,  drop1a);
    stateTrajMap.put(State.CLEAR1, clr1a);
    stateTrajMap.put(State.SHOOT,  shoot);
    stateTrajMap.put(State.RETMID, midret);
    stateTrajMap.put(State.WALL,   wall);
    stateTrajMap.put(State.WOB2,   wob2);
    stateTrajMap.put(State.DROP2,  drop2a);
    stateTrajMap.put(State.PARK,   parka);
  }
}
