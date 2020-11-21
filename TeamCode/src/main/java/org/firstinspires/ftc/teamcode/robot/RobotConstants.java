package org.firstinspires.ftc.teamcode.robot;

public class RobotConstants
{
  //TODO - make table of various motors for CPR and max rpm
  //For now, values for for Neverest orbital 20
  public static final double MM2IN = 1.0/25.4;
  public static final Motors.MotorModel DT_MOTOR = Motors.MotorModel.GOBILDA_5202_19_2;
  public static final double DT_CPMR = DT_MOTOR.getCpr(); //counts per motor output shaft rev
  public static final double DT_MAX_RPM = DT_MOTOR.getRpm();
  public static final double DT_EXT_GEAR_RATIO = 1.0;
  public static final double DT_CPWR = DT_CPMR / DT_EXT_GEAR_RATIO; //counts per whl rev
  public static final double DT_WHEEL_DIAM = 4.0; //96.0 * MM2IN for gobilda
  public static final double DT_WHEEL_RAD = DT_WHEEL_DIAM/2.0; //96.0 * MM2IN for gobilda
  public static final double DT_CIRCUM = DT_WHEEL_DIAM * Math.PI;
  public static final double DT_CPI = DT_CPWR / DT_CIRCUM;

  public static final double DT_TRACK_WIDTH = 16.34;
}
