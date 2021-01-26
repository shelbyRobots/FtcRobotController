package org.firstinspires.ftc.teamcode.field;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.Lifter;
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
          new MecanumVelocityConstraint(35, DT_TRACK_WIDTH)
      ));
  public final static TrajectoryAccelerationConstraint wobAccelLim
      = new ProfileAccelerationConstraint(30);

  public enum State
  {
    DROP1,
    SHOOT,
    REVERSE,
    WOB2,
    DROP2,
    PARK,
    IDLE
  }

  public EnumMap<State, Trajectory> stateTrajMap = new EnumMap<>(State.class);
  public EnumMap<UgRrRoute.State, String> stateColMap = new EnumMap<>(State.class);

  final static int    MAX_SEGMENTS = 32;

  public final static boolean GO_FOR_TWO = true;

  public static Pose2d startPose;
  public static Pose2d shtPose;

  //char1: p=Pose2d, t=Trajectory
  //char2: W=Wobble, M=Mid, D=Drop, S=Shoot, R=Return, P=Park, B=Begin
  //char3: O=Outside, I=Inside
  //char4: A-C=dest box, D=Drop, R=Return, N=aNy (non specific)
  Pose2d pBIN;
  Pose2d pBON;
  Pose2d pWON;
  Pose2d pMOD;
  Pose2d pDOA;
  Pose2d pDOB;
  Pose2d pDOC;
  Pose2d pSON;
  Pose2d pMOR;
  Pose2d pRON;
  Pose2d pWIN;
  Pose2d pMID;
  Pose2d pDIA;
  Pose2d pDIB;
  Pose2d pDIC;
  Pose2d pPIN;

  Pose2d pSIN;
  Pose2d pMIR;
  Pose2d pRIN;
  Pose2d pPON;

  public Trajectory tDIA;
  public Trajectory tDIB;
  public Trajectory tDIC;
  public Trajectory tDOA;
  public Trajectory tDOB;
  public Trajectory tDOC;

  public Trajectory tSIA;
  public Trajectory tSIB;
  public Trajectory tSIC;
  public Trajectory tSOA;
  public Trajectory tSOB;
  public Trajectory tSOC;

  public Trajectory tRIN;
  public Trajectory tRON;

  public Trajectory tWIN;
  public Trajectory tWON;

  public Trajectory tPIA;
  public Trajectory tPIB;
  public Trajectory tPIC;
  public Trajectory tPOA;
  public Trajectory tPOB;
  public Trajectory tPOC;

  public Trajectory tPIS;
  public Trajectory tPOS;


  public Trajectory WD1;
  public Trajectory SHT;
  public Trajectory REV;
  public Trajectory WPU;
  public Trajectory WD2;
  public Trajectory PRK;

  //private double DEF_SHT_DST; //= shtPose.vec().distTo(goalVec);

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

    stateTrajMap.put(State.DROP1,  WD1);
    stateTrajMap.put(State.SHOOT,  SHT);
    if(GO_FOR_TWO)
    {
      stateTrajMap.put(State.REVERSE, REV);
      stateTrajMap.put(State.WOB2,    WPU);
      stateTrajMap.put(State.DROP2,   WD2);

    }
    stateTrajMap.put(State.PARK,    PRK);

    stateColMap.put(State.DROP1,   "#F50505");
    stateColMap.put(State.SHOOT,   "#F55505");
    stateColMap.put(State.REVERSE, "#F5D505");
    stateColMap.put(State.WOB2,    "#15F505");
    stateColMap.put(State.DROP2,   "#05F5F5");
    stateColMap.put(State.PARK,    "#0505F5");
  }

  public void initTrajectories()
  {
    double grabDist = 1.0;
    double grabDistI = grabDist;
    double grabDistO = grabDist;
    if(startPos == START_2)
    {
      grabDistI = 0.0;
    }
    else
    {
      grabDistO = 0.0;
    }
    RobotLog.dd(TAG, "Building trajectories");
    drive.setPoseEstimate(startPose);

    tDIA = new TrajectoryBuilder(pBIN, wobVelLim, wobAccelLim)
        .addDisplacementMarker(grabDistI, this::doGrab)
        .splineTo(pMID.vec(), pMID.getHeading())
        .splineTo(pDIA.vec(), pDIA.getHeading())
        .addDisplacementMarker(this::doDrop)
        .addDisplacementMarker(this::doStartShoot).build();
    tDIB = new TrajectoryBuilder(pBIN, defVelLim, defAccelLim)
        .addDisplacementMarker(grabDistI, this::doGrab)
        .splineTo(pMID.vec(), pMID.getHeading())
        .splineTo(pDIB.vec(), pDIB.getHeading())
        .addDisplacementMarker(this::doDrop)
        .addDisplacementMarker(this::doStartShoot).build();
    tDIC = new TrajectoryBuilder(pBIN, defVelLim, defAccelLim)
        .addDisplacementMarker(grabDistI, this::doGrab)
        .splineTo(pMID.vec(), pMID.getHeading())
        .splineTo(pDIC.vec(), pDIC.getHeading())
        .addDisplacementMarker(this::doDrop)
        .addDisplacementMarker(this::doStartShoot).build();
    tDOA = new TrajectoryBuilder(pBON, defVelLim, defAccelLim)
        .addDisplacementMarker(grabDistO, this::doGrab)
        .splineTo(pMOD.vec(), pMOD.getHeading())
        .splineTo(pDOA.vec(), pDOA.getHeading())
        .addDisplacementMarker(this::doDrop)
        .addDisplacementMarker(this::doStartShoot).build();
    tDOB = new TrajectoryBuilder(pBON, defVelLim, defAccelLim)
        .addDisplacementMarker(grabDistO, this::doGrab)
        .splineTo(pMOD.vec(), pMOD.getHeading())
        .splineTo(pDOB.vec(), pDOB.getHeading())
        .addDisplacementMarker(this::doDrop)
        .addDisplacementMarker(this::doStartShoot).build();
    tDOC = new TrajectoryBuilder(pBON, defVelLim, defAccelLim)
        .addDisplacementMarker(grabDistO, this::doGrab)
        .splineTo(pMOD.vec(), pMOD.getHeading())
        .splineTo(pDOC.vec(), pDOC.getHeading())
        .addDisplacementMarker(this::doDrop)
        .addDisplacementMarker(this::doStartShoot).build();

    tSIA = new TrajectoryBuilder(tDIA.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(pSIN)
        .addDisplacementMarker(this::doShoot).build();
    tSIB = new TrajectoryBuilder(tDIB.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(pSIN)
        .addDisplacementMarker(this::doShoot).build();
    tSIC = new TrajectoryBuilder(tDIC.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(pSIN)
        .addDisplacementMarker(this::doShoot).build();
    tSOA = new TrajectoryBuilder(tDOA.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(pSON)
        .addDisplacementMarker(this::doShoot).build();
    tSOB = new TrajectoryBuilder(tDOB.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(pSON)
        .addDisplacementMarker(this::doShoot).build();
    tSOC = new TrajectoryBuilder(tDOC.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .lineToLinearHeading(pSON)
        .addDisplacementMarker(this::doShoot).build();

    tRIN = new TrajectoryBuilder(tSIA.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .splineTo(pMIR.vec(), pMIR.getHeading(), defVelLim, defAccelLim)
        .splineTo(pRIN.vec(), pRIN.getHeading(), wobVelLim, wobAccelLim).build();
    tRON = new TrajectoryBuilder(tSOA.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .splineTo(pMOR.vec(), pMOR.getHeading(), defVelLim, defAccelLim)
        .splineTo(pRON.vec(), pRON.getHeading(), wobVelLim, wobAccelLim).build();

    tWIN = new TrajectoryBuilder(tRON.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .splineToConstantHeading(pWIN.vec(), pWIN.getHeading(), wobVelLim, wobAccelLim).build();
    tWON = new TrajectoryBuilder(tRIN.end(), Math.toRadians(180), defVelLim, defAccelLim)
        .splineToConstantHeading(pWON.vec(), pWIN.getHeading(), wobVelLim, wobAccelLim).build();

    if(startPos == START_2)
    {
      WD1 = tDIA;
      SHT = tSIA;
      REV = tRIN;
      WPU = tWON;
      WD2 = tDOA;
      if(GO_FOR_TWO)
      {
        tPOA = new TrajectoryBuilder(tDOA.end(), Math.toRadians(180), defVelLim, defAccelLim)
            .lineToLinearHeading(pPON)
            .addDisplacementMarker(this::doPark).build();
        tPOB = new TrajectoryBuilder(tDOB.end(), Math.toRadians(180), defVelLim, defAccelLim)
            .lineToLinearHeading(pPON)
            .addDisplacementMarker(this::doPark).build();
        tPOC = new TrajectoryBuilder(tDOC.end(), Math.toRadians(180), defVelLim, defAccelLim)
            .lineToLinearHeading(pPON)
            .addDisplacementMarker(this::doPark).build();
        PRK = tPOA;
      }
      else
      {
        tPIS = new TrajectoryBuilder(tSIA.end(), Math.toRadians(180), defVelLim, defAccelLim)
            .lineToLinearHeading(pPIN)
            .addDisplacementMarker(this::doPark).build();
        PRK = tPIS;
      }
    }
    else
    {
      WD1 = tDOA;
      SHT = tSOA;
      REV = tRON;
      WPU = tWIN;
      WD2 = tDIA;
      if(GO_FOR_TWO)
      {
        tPIA = new TrajectoryBuilder(tDIA.end(), Math.toRadians(180), defVelLim, defAccelLim)
            .lineToLinearHeading(pPIN)
            .addDisplacementMarker(this::doPark).build();
        tPIB = new TrajectoryBuilder(tDIB.end(), Math.toRadians(180), defVelLim, defAccelLim)
            .lineToLinearHeading(pPIN)
            .addDisplacementMarker(this::doPark).build();
        tPIC = new TrajectoryBuilder(tDIC.end(), Math.toRadians(180), defVelLim, defAccelLim)
            .lineToLinearHeading(pPIN)
            .addDisplacementMarker(this::doPark).build();
        PRK = tPIA;
      }
      else
      {
        tPOS = new TrajectoryBuilder(tSOA.end(), Math.toRadians(180), defVelLim, defAccelLim)
            .lineToLinearHeading(pPON)
            .addDisplacementMarker(this::doPark).build();
        PRK = tPOS;
      }
    }

    RobotLog.dd(TAG, "Done Building trajectories");
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

    double shtHdgO = sh*15.0;
    double shtHdgI = -shtHdgO;

    double shtP1O = sh*35.0;
    double shtP2O = sh*30.0;
    double shtP3O = sh*25.0;

    pBIN = new Pose2d(sx*-61.5,sy*-24.0, sh*Math.toRadians(0));  poses.add(pWIN);
    pBON = new Pose2d(sx*-61.5,sy*-48.0, sh*Math.toRadians(0));  poses.add(pWON);

    pWIN = new Pose2d(sx*-61.25,sy*-24.0, sh*Math.toRadians(0));  poses.add(pWIN);
    pWON = new Pose2d(sx*-61.25,sy*-48.0, sh*Math.toRadians(0));  poses.add(pWON);

    pMID = new Pose2d(sx*-24.0,sy*-21.0, sh*Math.toRadians(0));  poses.add(pMID);
    pMOD = new Pose2d(sx*-18.0,sy*-53.0, sh*Math.toRadians(0));  poses.add(pMOD);

    pDIA = new Pose2d(sx*  3.0,sy*-47.0, sh*Math.toRadians(-35));  poses.add(pDIA);
    pDIB = new Pose2d(sx* 20.0,sy*-28.0, sh*Math.toRadians(-20));  poses.add(pDIB);
    pDIC = new Pose2d(sx* 46.0,sy*-46.0, sh*Math.toRadians(-45));  poses.add(pDIC);
    pDOA = new Pose2d(sx*  5.0,sy*-56.0, sh*Math.toRadians(-30));  poses.add(pDOA);
    pDOB = new Pose2d(sx* 28.0,sy*-46.0, sh*Math.toRadians(-35));  poses.add(pDOB);
    pDOC = new Pose2d(sx* 52.0,sy*-59.0, sh*Math.toRadians(0));  poses.add(pDOC);

    pSIN = new Pose2d(sx* -6.0,sy*-18.0, sh*Math.toRadians(shtHdgI));  poses.add(pSIN);
    pSON = new Pose2d(sx* -6.0,sy*-54.0, sh*Math.toRadians(shtHdgO));  poses.add(pSON);

    pMIR = new Pose2d(sx*-30.0,sy*-18.0, sh*Math.toRadians(180.0-shtHdgI));  poses.add(pMIR);
    pMOR = new Pose2d(sx*-30.0,sy*-54.0, sh*Math.toRadians(180.0-shtHdgO));  poses.add(pMOR);

    pRIN = new Pose2d(sx*-60.0,sy*-28.0, sh*Math.toRadians(180.0-shtHdgI));  poses.add(pRIN);
    pRON = new Pose2d(sx*-60.0,sy*-44.0, sh*Math.toRadians(180.0-shtHdgO));  poses.add(pRON);

    pPIN = new Pose2d(sx*  4.0,sy*-36.0, sh*Math.toRadians(0));  poses.add(pPIN);
    pPON = new Pose2d(sx*  4.0,sy*-36.0, sh*Math.toRadians(0));  poses.add(pPON);

    startPose = pBON;
    if(startPos == START_2) startPose = pBIN;

    shtPose = pSON;
    if(startPos == START_2) shtPose = pSIN;
  }

  @SuppressWarnings("unused")
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
    if(robot.liftyBoi != null) robot.liftyBoi.setGuidePos(Lifter.GuidePos.OPEN);
  }

  private void doGrab()
  {
    RobotLog.dd(TAG, "doGrab");
    if(robot.liftyBoi != null) robot.liftyBoi.setGuidePos(Lifter.GuidePos.CLOSED);
  }

  private void doPark()
  {
    RobotLog.dd(TAG, "Parking bot");
    if(robot.burr != null) robot.burr.stop();
    if(robot.loader != null)
    {
      robot.loader.load(0.0);
      robot.loader.setGatePos(Loader.gatePos.CLOSE);
      robot.loader.whlStp();
    }
    if(robot.liftyBoi != null)
    {
      robot.liftyBoi.setGuidePos(Lifter.GuidePos.OPEN);
      robot.liftyBoi.setClampPos(Lifter.ClampPos.CLOSED);
    }
  }

  private void doShoot()
  {
    //double shotdist = DEF_SHT_DST;
    RobotLog.dd(TAG, "Shooting");
    //if(robot.burr != null) robot.burr.shotSpeed(shotdist);
    if(robot.burr != null) robot.burr.shootCps(RobotConstants.SH_FAV_CPS);

    if(robot.loader != null)
    {
      robot.loader.load(1.0);
      robot.loader.setGatePos(Loader.gatePos.OPEN);
      robot.loader.whlFwd();
    }

    if(robot.intake != null)
    {
      robot.intake.suck(1.0);
    }
  }

  private boolean firstShoot = true;
  private void doStartShoot()
  {
    if(!firstShoot) return;
    firstShoot = false;
    RobotLog.dd(TAG, "Starting shooter");
    //if(robot.burr != null) robot.burr.shotSpeed(DEF_SHT_DST);
    if(robot.burr != null) robot.burr.shootCps(RobotConstants.SH_FAV_CPS);
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
