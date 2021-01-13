package org.firstinspires.ftc.teamcode.field;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.Loader;
import org.firstinspires.ftc.teamcode.robot.MecanumDriveLRR;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.TilerunnerMecanumBot;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.Vector;

import static org.firstinspires.ftc.teamcode.field.Route.StartPos.START_2;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.DT_TRACK_WIDTH;

public class UgRrRoute
{
  TilerunnerMecanumBot robot;
  MecanumDriveLRR drive;
  PositionOption startPos;
  //PositionOption parkChoice;
  Field.Alliance alliance;

  public static TrajectoryVelocityConstraint defVelLim = RobotConstants.defVelConstraint;
  public static TrajectoryAccelerationConstraint defAccelLim = RobotConstants.defAccelConstraint;

  public final static TrajectoryVelocityConstraint wobVelLim =
      new MinVelocityConstraint(Arrays.asList(
          new AngularVelocityConstraint(MAX_ANG_VEL),
          new MecanumVelocityConstraint(25, DT_TRACK_WIDTH)
      ));
  public final static TrajectoryAccelerationConstraint wobAccelLim
      = new ProfileAccelerationConstraint(20);

  public enum State
  {
    DROP1,
    CLEAR1,
    SHOOT,
    WOB2,
    DROP2,
    PARK,
    IDLE
  }

  public EnumMap<State, Trajectory> stateTrajMap = new EnumMap<>(State.class);

  final static int    MAX_SEGMENTS = 32;

  private final static boolean GO_FOR_TWO = true;

  Pose2d wA1Pose;
  Pose2d wA2Pose;
  Pose2d wB1Pose;
  Pose2d wB2Pose;
  Pose2d wC1Pose;
  Pose2d wC2Pose;

  public static Pose2d startPose;
  Pose2d st1Pose;
  Pose2d st2Pose;
  Pose2d dg1Pose;
  Pose2d dg2Pose;
  Pose2d wAcPose;
  public static Pose2d shtPose;
  Pose2d md1Pose;
  Pose2d md2Pose;
  Pose2d walPose;
  Pose2d w2gPose;
  Pose2d prkPose;

  private double DEF_SHT_DST; //= shtPose.vec().distTo(goalVec);
  private double DEF_SHT_CPS = RobotConstants.SH_FAV_CPS;

  public Trajectory drop1a;
  public Trajectory drop1b;
  public Trajectory drop1c;
  public Trajectory clr1a;
  public Trajectory clr1b;
  public Trajectory clr1c;
  public Trajectory shoot;
  public Trajectory wob2;
  public Trajectory drop2a;
  public Trajectory drop2b;
  public Trajectory drop2c;
  public Trajectory parks;
  public Trajectory parka;
  public Trajectory parkb;
  public Trajectory parkc;

  private static final String TAG = "SJH_URR";

  public UgRrRoute(TilerunnerMecanumBot robot,
                   PositionOption startPos,
                   Field.Alliance alliance)
  {
    this.robot        = robot;
    this.drive        = (MecanumDriveLRR) robot.drive;
    this.startPos     = startPos;
    this.alliance     = alliance;

    initPoses();
    initTrajectories();

    stateTrajMap.put(State.DROP1,  drop1a);
    stateTrajMap.put(State.CLEAR1, clr1a);
    stateTrajMap.put(State.SHOOT,  shoot);
    if(GO_FOR_TWO)
    {
      stateTrajMap.put(State.WOB2, wob2);
      stateTrajMap.put(State.DROP2, drop2a);
      stateTrajMap.put(State.PARK, parka);
    }
    else
    {
      stateTrajMap.put(State.PARK, parks);
    }
  }

  public void initTrajectories()
  {
    RobotLog.dd(TAG, "Building trajectories");
    drive.setPoseEstimate(startPose);
    if (startPos == START_2) initStart2Traj();
    else                     initStart1Traj();

    RobotLog.dd(TAG, "Done Building trajectories");
  }

  private void initStart1Traj()
  {
    drop1a = new TrajectoryBuilder(startPose, defVelLim, defAccelLim)
        .splineTo(dg1Pose.vec(), dg1Pose.getHeading())
        .splineTo(wA1Pose.vec(), wA1Pose.getHeading())
        .addDisplacementMarker(this::doDrop).build();
    drop1b = new TrajectoryBuilder(startPose, defVelLim, defAccelLim)
        .splineTo(dg1Pose.vec(), dg1Pose.getHeading())
        .splineTo(wB1Pose.vec(), wB1Pose.getHeading())
        .addDisplacementMarker(this::doDrop).build();
    drop1c = new TrajectoryBuilder(startPose, defVelLim, defAccelLim)
        .splineTo(dg1Pose.vec(), dg1Pose.getHeading())
        .splineTo(wC1Pose.vec(), wC1Pose.getHeading())
        .addDisplacementMarker(this::doDrop).build();
    clr1a = new TrajectoryBuilder(drop1a.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(wAcPose).build();
    clr1b = new TrajectoryBuilder(drop1b.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(wAcPose).build();
    clr1c = new TrajectoryBuilder(drop1c.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(wAcPose).build();
    shoot = new TrajectoryBuilder(clr1a.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(shtPose)
        .addDisplacementMarker(this::doShoot).build();
    wob2 = new TrajectoryBuilder(shoot.end(), Math.toRadians(-90), defVelLim, defAccelLim)
        .splineToConstantHeading(md1Pose.vec(), Math.toRadians(180))
        .splineToConstantHeading(md2Pose.vec(), Math.toRadians(180))
        .splineToConstantHeading(walPose.vec(), Math.toRadians(90), wobVelLim, wobAccelLim)
        .splineToConstantHeading(w2gPose.vec(), Math.toRadians(90), wobVelLim, wobAccelLim).build();
    drop2a = new TrajectoryBuilder(wob2.end(), defVelLim, defAccelLim)
        .splineTo(dg2Pose.vec(), dg2Pose.getHeading())
        .splineTo(wA2Pose.vec(), wA2Pose.getHeading()).build();
    drop2b = new TrajectoryBuilder(wob2.end(), defVelLim, defAccelLim)
        .splineTo(dg2Pose.vec(), dg2Pose.getHeading())
        .splineTo(wB2Pose.vec(), wB2Pose.getHeading()).build();
    drop2c = new TrajectoryBuilder(wob2.end(), defVelLim, defAccelLim)
        .splineTo(dg2Pose.vec(), dg2Pose.getHeading())
        .splineTo(wC2Pose.vec(), wC2Pose.getHeading()).build();
    parks = new TrajectoryBuilder(shoot.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(prkPose).build();
    parka = new TrajectoryBuilder(drop2a.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(prkPose).build();
    parkb = new TrajectoryBuilder(drop2b.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(prkPose).build();
    parkc = new TrajectoryBuilder(drop2c.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(prkPose).build();
  }

  private void initStart2Traj()
  {
    drop1a = new TrajectoryBuilder(startPose, defVelLim, defAccelLim)
      .splineTo(dg2Pose.vec(), dg2Pose.getHeading())
      .splineTo(wA2Pose.vec(), wA2Pose.getHeading())
      .addDisplacementMarker(this::doDrop).build();
    drop1b = new TrajectoryBuilder(startPose, defVelLim, defAccelLim)
        .splineTo(dg2Pose.vec(), dg2Pose.getHeading())
        .splineTo(wB2Pose.vec(), wB2Pose.getHeading())
        .addDisplacementMarker(this::doDrop).build();
    drop1c = new TrajectoryBuilder(startPose, defVelLim, defAccelLim)
        .splineTo(dg2Pose.vec(), dg2Pose.getHeading())
        .splineTo(wC2Pose.vec(), wC2Pose.getHeading())
        .addDisplacementMarker(this::doDrop).build();
    //TODO - figure out CLR for start 2, and rest of route
    clr1a = new TrajectoryBuilder(drop1a.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(wAcPose).build();
    clr1b = new TrajectoryBuilder(drop1b.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(wAcPose).build();
    clr1c = new TrajectoryBuilder(drop1c.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(wAcPose).build();
    shoot = new TrajectoryBuilder(clr1a.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(shtPose)
        .addDisplacementMarker(this::doShoot).build();
    wob2 = new TrajectoryBuilder(shoot.end(), Math.toRadians(-90), defVelLim, defAccelLim)
        .splineToConstantHeading(md1Pose.vec(), Math.toRadians(180))
        .splineToConstantHeading(md2Pose.vec(), Math.toRadians(180))
        .splineToConstantHeading(walPose.vec(), Math.toRadians(90), wobVelLim, wobAccelLim)
        .splineToConstantHeading(w2gPose.vec(), Math.toRadians(90), wobVelLim, wobAccelLim).build();
    drop2a = new TrajectoryBuilder(wob2.end(), defVelLim, defAccelLim)
        .splineTo(dg2Pose.vec(), dg2Pose.getHeading())
        .splineTo(wA2Pose.vec(), wA2Pose.getHeading()).build();
    drop2b = new TrajectoryBuilder(wob2.end(), defVelLim, defAccelLim)
        .splineTo(dg2Pose.vec(), dg2Pose.getHeading())
        .splineTo(wB2Pose.vec(), wB2Pose.getHeading()).build();
    drop2c = new TrajectoryBuilder(wob2.end(), defVelLim, defAccelLim)
        .splineTo(dg2Pose.vec(), dg2Pose.getHeading())
        .splineTo(wC2Pose.vec(), wC2Pose.getHeading()).build();
    parks = new TrajectoryBuilder(shoot.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(prkPose).build();
    parka = new TrajectoryBuilder(drop2a.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(prkPose).build();
    parkb = new TrajectoryBuilder(drop2b.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(prkPose).build();
    parkc = new TrajectoryBuilder(drop2c.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(prkPose).build();
  }

  Vector<Pose2d> poses = new Vector<>(MAX_SEGMENTS);
  private void initPoses()
  {
    RobotLog.dd(TAG, "Initializing poses");
    int sx = 1;
    int sy = 1;
    int sh = 1;
    if(alliance == Field.Alliance.BLUE)
    {
      sy = -1;
      sh = -1;
    }
    wA1Pose = new Pose2d(sx*  6.0,sy*-59.0, sh* Math.toRadians(0));  poses.add(wA1Pose);
    wA2Pose = new Pose2d(sx* 14.0,sy*-44.0, sh*-Math.toRadians(90)); poses.add(wA2Pose);
    wB1Pose = new Pose2d(sx* 24.0,sy*-48.0, sh* Math.toRadians(45)); poses.add(wB1Pose);
    wB2Pose = new Pose2d(sx* 24.0,sy*-30.0, sh*-Math.toRadians(20)); poses.add(wB2Pose);
    wC1Pose = new Pose2d(sx* 48.0,sy*-59.0, sh* Math.toRadians(0));  poses.add(wC1Pose);
    wC2Pose = new Pose2d(sx* 44.0,sy*-44.0, sh*-Math.toRadians(45)); poses.add(wC2Pose);

    st1Pose = new Pose2d(sx*-61.5,sy*-44.0, sh* Math.toRadians(0));  poses.add(st1Pose);
    st2Pose = new Pose2d(sx*-61.5,sy*-28.0, sh* Math.toRadians(0));  poses.add(st2Pose);
    dg1Pose = new Pose2d(sx*  0.0,sy*-59.0, sh* Math.toRadians(0));  poses.add(dg1Pose);
    dg2Pose = new Pose2d(sx*  0.0,sy*-24.0, sh* Math.toRadians(0));  poses.add(dg2Pose);
    wAcPose = new Pose2d(sx*  0.0,sy*-59.0, sh* Math.toRadians(0));  poses.add(wAcPose);
    shtPose = new Pose2d(sx*  0.0,sy*-36.0, sh* Math.toRadians(0));  poses.add(shtPose);
    md1Pose = new Pose2d(sx*-16.0,sy*-52.0, sh* Math.toRadians(0));  poses.add(md1Pose);
    md2Pose = new Pose2d(sx*-48.0,sy*-50.0, sh* Math.toRadians(0));  poses.add(md2Pose);
    walPose = new Pose2d(sx*-60.0,sy*-40.0, sh* Math.toRadians(0));  poses.add(walPose);
    w2gPose = new Pose2d(sx*-61.0,sy*-24.0, sh* Math.toRadians(0));  poses.add(w2gPose);
    prkPose = new Pose2d(sx* 10.0,sy*-36.0, sh* Math.toRadians(0));  poses.add(prkPose);

    DEF_SHT_DST = shtPose.vec().distTo(new Vector2d(72.0, sy*-36.0));

    if(startPos == START_2) startPose = st2Pose;
    else startPose = st1Pose;
  }

  private void printPoses()
  {
    for (Pose2d pose : poses)
    {
      RobotLog.dd(TAG, "%s", pose);
    }
  }

  private void doDrop()
  {
    RobotLog.dd(TAG, "Dropping wobblyBOI");
    //if(robot.burr != null) robot.burr.shotSpeed(DEF_SHT_DST);
    if(robot.burr != null) robot.burr.shootCps(DEF_SHT_CPS);
  }

  @SuppressWarnings("unused")
  private void doGrab()
  {
    RobotLog.dd(TAG, "doGrab");
  }

  @SuppressWarnings("unused")
  private void doPlatch()
  {
    RobotLog.dd(TAG, "Platching platform");
  }

  @SuppressWarnings("unused")
  private void doUnPlatch()
  {
    RobotLog.dd(TAG, "UnPlatching platform");
  }

  @SuppressWarnings("unused")
  private void doPark()
  {
    RobotLog.dd(TAG, "Parking bot");
  }

  private void doShoot()
  {
    //double shotdist = DEF_SHT_DST;
    RobotLog.dd(TAG, "Shooting");
    //if(robot.burr != null) robot.burr.shotSpeed(shotdist);
    if(robot.burr != null) robot.burr.shootCps(DEF_SHT_CPS);

    if(robot.loader != null)
    {
      robot.loader.load(1.0);
      robot.loader.setGatePos(Loader.gatePos.OPEN);
      robot.loader.whlFwd();
    }
  }

//  public static void main(String[] args)
//  {
//    System.out.println(String.format(Locale.US, "RED"));
//    UgRrRoute ugrr = new UgRrRoute(new TilerunnerMecanumBot(), START_1, Field.Alliance.RED);
//    ugrr.initPoses();
//    for (Pose2d pose : ugrr.poses)
//    {
//      System.out.println(String.format(Locale.US, "%s", pose));
//    }
//    System.out.println(String.format(Locale.US, "RED strtPos:%s", srtPose));
//
//
//    System.out.println(String.format(Locale.US, "BLUE"));
//    ugrr = new UgRrRoute(new TilerunnerMecanumBot(), START_1, Field.Alliance.BLUE);
//    ugrr.initPoses();
//    for (Pose2d pose : ugrr.poses)
//    {
//      System.out.println(String.format(Locale.US, "%s", pose));
//    }
//    System.out.println(String.format(Locale.US, "BLUE strtPos:%s", srtPose));
//  }
}
