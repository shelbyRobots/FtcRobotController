package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

public class CamMount
{
  public CamMount(HardwareMap map) { this.hwMap = map; }

  public boolean init(){

    boolean success;

    try{
      camServo = hwMap.get(Servo.class, "CamServo");
      success = true;
      setCamPos(RobotConstants.CamStow);
    }catch (Exception e)
    {
      RobotLog.ee(TAG, "ERROR: Cam Servo missing");
      success = false;
    }
    return success;
  }

  public void setCamPos(double pos){
    camServo.setPosition(pos);
  }

  private static final String TAG = "SJH_CAM";
  public Servo camServo;
  protected HardwareMap hwMap;
}
