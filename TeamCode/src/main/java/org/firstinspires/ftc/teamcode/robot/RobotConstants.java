package org.firstinspires.ftc.teamcode.robot;

@SuppressWarnings("unused")
public class RobotConstants
{
  public static final double MM2IN = 1.0/25.4;
  public static Motors.MotorModel DT_MOTOR = Motors.MotorModel.GOBILDA_5202_19_2;
  public static double DT_CPMR = DT_MOTOR.getCpr(); //counts per motor output shaft rev
  public static double DT_MAX_RPM = DT_MOTOR.getRpm();
  public static double DT_EXT_GEAR_RATIO = 1.0;
  public static double DT_GEAR_RATIO = DT_MOTOR.getGear() * DT_EXT_GEAR_RATIO;
  public static double DT_CPWR = DT_CPMR * DT_EXT_GEAR_RATIO; //counts per whl rev
  public static double DT_WHEEL_DIAM = 4.0; //96.0 * MM2IN for gobilda
  public static double DT_CIRCUM = DT_WHEEL_DIAM * Math.PI;
  public static double DT_CPI = DT_CPWR / DT_CIRCUM;

  public static double DT_TRACK_WIDTH = 16.34;

  public static final double DT_SAF_IPS = 30.0;
  public static double DT_MAX_IPS;
  public static final double DT_SAF_CPS = DT_SAF_IPS * DT_CPI;
  public static double DT_MAX_CPS;

  public RobotConstants(Motors.MotorModel motorModel, double whlDiam, double trackWidth, double extGear)
  {
    init(motorModel, whlDiam, trackWidth, extGear);
  }

  public RobotConstants()
  {
    this(Motors.MotorModel.GOBILDA_5202_19_2, 96/MM2IN, 16.34, 1.0);
  }

  public RobotConstants(Chassis chassis)
  {
    switch (chassis)
    {
      case MEC1:
        init(Motors.MotorModel.AM_NEVEREST_ORBITAL_20, 4.0,14.9, 1.0);
        break;
      case MEC2:
      case MEC3:
      default:
        init(Motors.MotorModel.GOBILDA_5202_19_2, 96/MM2IN,16.34, 1.0);
        break;
    }
  }

  private void init(Motors.MotorModel motorModel, double whlDiam, double trackWidth, double extGear)
  {
    DT_MOTOR = motorModel;
    DT_CPMR = DT_MOTOR.getCpr();
    DT_MAX_RPM = DT_MOTOR.getRpm();

    DT_EXT_GEAR_RATIO = extGear;
    DT_GEAR_RATIO = DT_MOTOR.getGear() * DT_EXT_GEAR_RATIO;
    DT_CPWR = DT_CPMR / DT_EXT_GEAR_RATIO;

    DT_WHEEL_DIAM = whlDiam;
    DT_CIRCUM = DT_WHEEL_DIAM * Math.PI;
    DT_CPI = DT_CPWR / DT_CIRCUM;

    DT_MAX_IPS = DT_MAX_RPM/60.0 * DT_CIRCUM;
    DT_MAX_CPS = DT_MAX_IPS * DT_CPI;

    DT_TRACK_WIDTH = trackWidth;
  }

  public enum Chassis
  {
    MEC1,
    MEC2,
    MEC3
  }
}