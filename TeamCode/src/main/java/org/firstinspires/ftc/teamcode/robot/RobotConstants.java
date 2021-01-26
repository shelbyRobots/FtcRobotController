package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Arrays;

@SuppressWarnings("unused")
public class RobotConstants
{
  //Wobblie Arm Clamp Servo positions
  public static double WA_CLAMP_OPEN = 0.66;
  public static double WA_CLAMP_MID  = 0.56;
  public static double WA_CLAMP_GRAB = 0.46;

  //Wobblie AutonGuide Clamp Servo positions
  public static double WG_CLAMP_OPEN = 0.62;
  public static double WG_CLAMP_MID  = 0.75;
  public static double WG_CLAMP_GRAB = 0.90;

  public static double LD_GATE_OPEN   = 0.36;
  public static double LD_GATE_CLOSED = 0.56;

  public static double SH_FAV_CPS = 1820;
  public static double SH_72_CPS = 1820;
  public static double SH_96_CPS = 2060;
  public static double SH_PS_CPS = 1740;
  public static PIDFCoefficients SH_PID = new PIDFCoefficients(20.0, 0.0, 0.3,15.0);

  public static DcMotorSimple.Direction LD_PUSH_DIR = DcMotorSimple.Direction.REVERSE;

  public static final double MMPERIN = 25.4;
  public static Motors.MotorModel DT_MOTOR = Motors.MotorModel.GOBILDA_5202_19_2;
  public static double DT_CPMR = DT_MOTOR.getCpr(); //counts per motor output shaft rev
  public static double DT_MAX_RPM = DT_MOTOR.getRpm();
  public static double DT_EXT_GEAR_RATIO = 1.0;
  public static double DT_GEAR_RATIO = DT_MOTOR.getGear() * DT_EXT_GEAR_RATIO;
  public static double DT_CPWR = DT_CPMR * DT_EXT_GEAR_RATIO; //counts per whl rev
  public static double DT_WHEEL_DIAM =  96.0 / MMPERIN; //4.0 for tilerunner
  public static double DT_CIRCUM = DT_WHEEL_DIAM * Math.PI;
  public static double DC_ECIRC = DT_CIRCUM * DT_EXT_GEAR_RATIO;
  public static double DT_CPI = DT_CPMR / DC_ECIRC;
  public static double DT_IPC = 1.0/DT_CPI;
  public static double DC_RPM2VEL = DC_ECIRC / 60.0;

  public static double DT_TRACK_WIDTH = 16.34;

  public static final double DT_SAF_IPS = 30.0;
  public static double DT_MAX_IPS;
  public static final double DT_SAF_CPS = DT_SAF_IPS * DT_CPI;
  public static double DT_MAX_CPS;
  public static Chassis bot= Chassis.MEC2;

  public static ShelbyBot.DriveDir  DT_DIR = ShelbyBot.DriveDir.PUSHER;
  public static DcMotorSimple.Direction DT_LDIR = DcMotorSimple.Direction.REVERSE;
  public static DcMotorSimple.Direction DT_RDIR = DcMotorSimple.Direction.FORWARD;

  //Following variables are related to RoadRunner and should be tuned for each bot
  public static double LATERAL_MULTIPLIER = 1.2; //1.12;

  public static PIDFCoefficients MOTOR_VELO_PID =
      new PIDFCoefficients(14, 0, 0.3, 12.8);

  /*
   * These are the feedforward parameters used to model the drive motor behavior. If you are using
   * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
   * motor encoders or have elected not to use them for velocity control, these values should be
   * empirically tuned.
   */
  public static double kV = 1.0 / rpmToVelocity(DT_MAX_RPM);
  public static double kA = 0;
  public static double kStatic = 0;

  /*
   * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
   * Set this flag to false if drive encoders are not present and an alternative localization
   * method is in use (e.g., tracking wheels).
   *
   * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
   * from DriveVelocityPIDTuner.
   */
  public static final boolean RUN_USING_ENCODER = true;

  /*
   * These values are used to generate the trajectories for you robot. To ensure proper operation,
   * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
   * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
   * small and gradually increase them later after everything is working. All distance units are
   * inches.
   */

  public static double MAX_VEL = 59; //RR tune  maxVel 59.96
  public static double MAX_ACCEL = 40;
  public static double MAX_ANG_VEL = Math.toRadians(180);
  public static double MAX_ANG_ACCEL = Math.toRadians(180);

  public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(7.5, 0, 0);
  public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

  public static final boolean logDrive = true;

  public static final String TAG = "SJH_RBC";

  public RobotConstants()
  {
    this(Chassis.MEC2);
  }

  public RobotConstants(Chassis chassis)
  {
    init(chassis);
  }

  private void init(Chassis chas)
  {
    bot = chas;

    switch (bot)
    {
      case MEC1:
        DT_MOTOR = Motors.MotorModel.AM_NEVEREST_ORBITAL_20;
        DT_EXT_GEAR_RATIO = 1.0; //need RR tuning
        DT_WHEEL_DIAM = 4.0;
        DT_TRACK_WIDTH = 14.9; //need RR tuning
        MOTOR_VELO_PID = new PIDFCoefficients(14, 0, 0.3, 12.8); //need RR tuning
        break;

      case MEC2:
        RobotLog.dd(TAG, "Running as MEC2");
        WA_CLAMP_OPEN = 0.66;
        WA_CLAMP_MID  = 0.56;
        WA_CLAMP_GRAB = 0.46;

        WG_CLAMP_OPEN = 0.62;
        WG_CLAMP_MID  = 0.75;
        WG_CLAMP_GRAB = 0.90;

        LD_GATE_OPEN   = 0.36;
        LD_GATE_CLOSED = 0.56;
        LD_PUSH_DIR = DcMotorSimple.Direction.REVERSE;

        SH_FAV_CPS = 1820;

        MAX_VEL = 50;
        LATERAL_MULTIPLIER = 1.21; //1.18;
        DT_MOTOR = Motors.MotorModel.GOBILDA_5202_19_2;
        DT_EXT_GEAR_RATIO = 1.025; //tuned by RR tuning
        DT_WHEEL_DIAM = 96.0/MMPERIN;
        DT_TRACK_WIDTH = 16.4; //tuned by RR tuning
        MOTOR_VELO_PID = new PIDFCoefficients(14.4, 0, 0.4, 13.4); //RR tuning
        TRANSLATIONAL_PID = new PIDCoefficients(5, 0, 0);
        HEADING_PID = new PIDCoefficients(5, 0, 0);
        break;

      case MEC3:
        DT_MOTOR = Motors.MotorModel.GOBILDA_5202_19_2;
        DT_EXT_GEAR_RATIO = 1.025; //tuned by RR tuning
        DT_WHEEL_DIAM = 96.0/MMPERIN;
        DT_TRACK_WIDTH = 16.8; //tuned by RR tuning
        MOTOR_VELO_PID = new PIDFCoefficients(14, 0, 0.3, 12.8); //RR tuning
        break;

      default:
        DT_MOTOR = Motors.MotorModel.GOBILDA_5202_19_2;
        break;
    }

    DT_CPMR = DT_MOTOR.getCpr();
    DT_MAX_RPM = DT_MOTOR.getRpm();

    DT_GEAR_RATIO = DT_MOTOR.getGear() * DT_EXT_GEAR_RATIO;
    //DT_CPWR = DT_CPMR / DT_EXT_GEAR_RATIO;

    DT_CIRCUM = DT_WHEEL_DIAM * Math.PI;
    DC_ECIRC = DT_CIRCUM * DT_EXT_GEAR_RATIO;
    DT_CPI = DT_CPMR / DC_ECIRC;
    DC_RPM2VEL = DC_ECIRC / 60.0;

    DT_MAX_IPS = DT_MAX_RPM/60.0 * DT_CIRCUM;
    DT_MAX_CPS = DT_MAX_IPS * DT_CPI;

    DT_DIR = ShelbyBot.DriveDir.PUSHER;
    DT_LDIR = DcMotorSimple.Direction.REVERSE;
    DT_RDIR = DcMotorSimple.Direction.FORWARD;

    kV = 1.0 / rpmToVelocity(DT_MAX_RPM);
  }

  public enum Chassis
  {
    MEC1,
    MEC2,
    MEC3
  }

  public final static TrajectoryVelocityConstraint defVelConstraint =
      new MinVelocityConstraint(Arrays.asList(
          new AngularVelocityConstraint(MAX_ANG_VEL),
          new MecanumVelocityConstraint(MAX_VEL, DT_TRACK_WIDTH)
      ));
  public final static TrajectoryAccelerationConstraint defAccelConstraint
      = new ProfileAccelerationConstraint(MAX_ACCEL);

  public static double encoderTicksToInches(double ticks) {
//        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    return ticks * DT_IPC;
  }

  public static double rpmToVelocity(double rpm) {
//        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    return rpm * DC_RPM2VEL;
  }

  public static double getMotorVelocityF(double ticksPerSecond) {
    // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
    return 32767 / ticksPerSecond;
  }
}