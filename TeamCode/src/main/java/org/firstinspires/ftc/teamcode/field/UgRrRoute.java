package org.firstinspires.ftc.teamcode.field;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.MecanumDriveLRR;
import org.firstinspires.ftc.teamcode.robot.TilerunnerMecanumBot;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.Arrays;
import java.util.EnumMap;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

public class UgRrRoute
{
  TilerunnerMecanumBot robot;
  MecanumDriveLRR drive;
  PositionOption startPos;
  //PositionOption parkChoice;
  Field.Alliance alliance;

//  private FtcDashboard dashboard;

  public static TrajectoryVelocityConstraint defVelLim = DriveConstants.defVelConstraint;
  public static TrajectoryAccelerationConstraint defAccelLim = DriveConstants.defAccelConstraint;

  public final static TrajectoryVelocityConstraint wobVelLim =
      new MinVelocityConstraint(Arrays.asList(
          new AngularVelocityConstraint(MAX_ANG_VEL),
          new MecanumVelocityConstraint(25, TRACK_WIDTH)
      ));
  public final static TrajectoryAccelerationConstraint wobAccelLim
      = new ProfileAccelerationConstraint(20);

  public enum State
  {
    DROP1,
    CLEAR1,
    SHOOT,
    //RETMID,
    //WALL,
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
  Pose2d wC2Pose = new Pose2d(48, -48,-Math.toRadians(45));

  public static final Pose2d srtPose = new Pose2d(-61.5,-44,0);
  Pose2d dogPose = new Pose2d(0,-59,0);
  Pose2d wAcPose = new Pose2d(0.0,-59,0);
  Pose2d shtPose = new Pose2d(0.0,-36,0);
  Pose2d md1Pose = new Pose2d(-16.0, -52.0, 0);
  Pose2d md2Pose = new Pose2d(-48.0, -50.0, 0);
  Pose2d walPose = new Pose2d(-60.0,-40,0);
  Pose2d w2gPose = new Pose2d(-61.0,-24,0);
  Pose2d prkPose = new Pose2d(10.0,-36,0);

  private final Vector2d goalVec = new Vector2d(72.0, -36.0);
  private final double DEF_SHT_DST = shtPose.vec().distTo(goalVec);

  public Trajectory drop1a;
  public Trajectory drop1b;
  public Trajectory drop1c;
  public Trajectory clr1a;
  public Trajectory clr1b;
  public Trajectory clr1c;
  public Trajectory shoot;
  //public Trajectory midret;
  //public Trajectory wall;
  public Trajectory wob2;
  public Trajectory drop2a;
  public Trajectory drop2b;
  public Trajectory drop2c;
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

//    dashboard = FtcDashboard.getInstance();
//    dashboard.setTelemetryTransmissionInterval(25);

    RobotLog.dd(TAG, "Building trajectories");
    drive.setPoseEstimate(srtPose);
    drop1a = new TrajectoryBuilder(srtPose, defVelLim, defAccelLim)
        .splineTo(dogPose.vec(), dogPose.getHeading())
        .splineTo(wA1Pose.vec(), wA1Pose.getHeading())
        .addDisplacementMarker(this::doDrop).build();
    drop1b = new TrajectoryBuilder(srtPose, defVelLim, defAccelLim)
        .splineTo(dogPose.vec(), dogPose.getHeading())
        .splineTo(wB1Pose.vec(), wB1Pose.getHeading())
        .addDisplacementMarker(this::doDrop).build();
    drop1c = new TrajectoryBuilder(srtPose, defVelLim, defAccelLim)
        .splineTo(dogPose.vec(), dogPose.getHeading())
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
        .splineTo(wA2Pose.vec(), wA2Pose.getHeading()).build();
    drop2b = new TrajectoryBuilder(wob2.end(), defVelLim, defAccelLim)
        .splineTo(wB2Pose.vec(), wB2Pose.getHeading()).build();
    drop2c = new TrajectoryBuilder(wob2.end(), defVelLim, defAccelLim)
        .splineTo(wC2Pose.vec(), wC2Pose.getHeading()).build();
    parka = new TrajectoryBuilder(drop2a.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(prkPose).build();
    parkb = new TrajectoryBuilder(drop2b.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(prkPose).build();
    parkc = new TrajectoryBuilder(drop2c.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(prkPose).build();

    RobotLog.dd(TAG, "Done Building trajectories");

    stateTrajMap.put(State.DROP1,  drop1a);
    stateTrajMap.put(State.CLEAR1, clr1a);
    stateTrajMap.put(State.SHOOT,  shoot);
    //stateTrajMap.put(State.RETMID, midret);
    //stateTrajMap.put(State.WALL,   wall);
    stateTrajMap.put(State.WOB2,   wob2);
    stateTrajMap.put(State.DROP2,  drop2a);
    stateTrajMap.put(State.PARK,   parka);

//    TelemetryPacket packet = new TelemetryPacket();
//    Canvas fieldOverlay = packet.fieldOverlay();
//    fieldOverlay.setStrokeWidth(1);
//    fieldOverlay.setStroke("#4CAF50");
//    for(Map.Entry<UgRrRoute.State, Trajectory> entry : stateTrajMap.entrySet())
//    {
//      UgRrRoute.State state = entry.getKey();
//      Trajectory traj = entry.getValue();
//      if (state == UgRrRoute.State.IDLE) break;
//      DashboardUtil.drawSampledPath(fieldOverlay, traj.getPath());
//      dashboard.sendTelemetryPacket(packet);
//    }
  }

  private void doDrop()
  {
    RobotLog.dd(TAG, "Dropping wobblyBOI");
    if(robot.burr != null) robot.burr.shotSpeed(DEF_SHT_DST);
  }

  private void doGrab()
  {
    RobotLog.dd(TAG, "doGrab");
  }

  private void doShoot()
  {
    double shotdist = DEF_SHT_DST;
    RobotLog.dd(TAG, "Shooting");
    if(robot.burr != null) robot.burr.shotSpeed(shotdist);
    // TODO add control of loader fire
    if(robot.burr != null) robot.burr.stop();
  }
}
