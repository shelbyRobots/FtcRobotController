package org.firstinspires.ftc.teamcode.util;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.util.RobotLog;

@SuppressWarnings("unused")
public class PreferenceMgr
{
   private CommonUtil com;
   private SharedPreferences sharedPreferences;

   private String clubName       = "shelby";
   private String botName        = "GTO1";
   private String allianceColor  = "RED";
   private String startPosition  = "START_1";
   private String parkPosition   = "TRI_TIP";
   private String delay          = "0";

   private String leftOffset     = "0.0";
   private String cntrOffset     = "0.0";
   private String rghtOffset     = "0.0";

   private String gOffset        = "12.0";

   private static final String TAG = "SJH_PRF";

   public String getClubName() { return clubName; }

   public String getBotName() { return botName; }
   public void setBotName(String botName) { this.botName = botName; }

   public String getAllianceColor() { return allianceColor; }
   public void setAllianceColor(String allianceColor) { this.allianceColor = allianceColor; }

   public String getStartPosition() { return startPosition; }
   public void setStartPosition(String startPosition) { this.startPosition = startPosition; }

   public String getParkPosition() { return parkPosition; }
   public void setParkPosition(String parkPosition) { this.parkPosition = parkPosition; }

   public int getDelay() { return Integer.parseInt(delay); }
   public void setDelay(int delay) { this.delay =  String.valueOf(delay); }

   public void setLeftOffset(double offset)
   {
      leftOffset = String.valueOf(offset);
   }

   public void setCenterOffset(double offset)
   {
      cntrOffset = String.valueOf(offset);
   }

   public void setRightOffset(double offset)
   {
      rghtOffset = String.valueOf(offset);
   }

   public void setGlyphOffset(double offset)
   {
      gOffset = String.valueOf(offset);
   }

   public double getDropOffset(String botName, String alliance, String startPos, String key)
   {
      String strOff = sharedPreferences.getString(clubName + "." + botName  + "." +
                                                  alliance + "." + startPos + "." + key, "0.0");

      return Double.parseDouble(strOff);
   }

   public double getGlyphOffset(String botName)
   {
      String gOff = sharedPreferences.getString(clubName + "." + botName  +
                                                        ".gOffset", gOffset);

      return Double.parseDouble(gOff);
   }

   public PreferenceMgr()
   {
      com = CommonUtil.getInstance();
   }

   public void readPrefs()
   {
      sharedPreferences = PreferenceManager.getDefaultSharedPreferences(com.getContext());

      botName       = sharedPreferences.getString(clubName + ".botName", botName);
      if(botName == null) botName = "GTO1";

      allianceColor = sharedPreferences.getString(clubName + "." + botName +
                                                          ".Alliance", allianceColor);
      startPosition = sharedPreferences.getString(clubName + "." + botName +
                                                          ".StartPosition", startPosition);
      parkPosition  = sharedPreferences.getString(clubName + "." + botName +
                                                          ".ParkPosition", parkPosition);
      delay         = sharedPreferences.getString(clubName + "." + botName +
                                                          ".Delay", delay);

      writePrefs();
   }

   public void writePrefs()
   {
      //write the options to sharedpreferences
      SharedPreferences.Editor editor = sharedPreferences.edit();
      editor.putString(clubName + ".botName",            botName);
      editor.putString(clubName + "." + botName + ".Alliance",      allianceColor);
      editor.putString(clubName + "." + botName + ".StartPosition", startPosition);
      editor.putString(clubName + "." + botName + ".ParkPosition",  parkPosition);
      editor.putString(clubName + "." + botName + ".Delay",         delay);
      editor.apply();
   }

   public void writeOffsets()
   {
      SharedPreferences.Editor editor = sharedPreferences.edit();

      editor.putString(clubName + "." + botName  + ".gOffset", gOffset);

      editor.putString(clubName + "." + botName  + "." +
                               allianceColor + "." + startPosition + "." + "LEFT", leftOffset);
      editor.putString(clubName + "." + botName  + "." +
                               allianceColor + "." + startPosition + "." + "CENTER", cntrOffset);
      editor.putString(clubName + "." + botName  + "." +
                               allianceColor + "." + startPosition + "." + "RIGHT", rghtOffset);

      editor.apply();
   }

   public void logPrefs()
   {
      RobotLog.dd(TAG, "Default Config Values:");
      RobotLog.dd(TAG, "Club:     %s", clubName);
      RobotLog.dd(TAG, "Bot:      %s", botName);
      RobotLog.dd(TAG, "Alliance: %s", allianceColor);
      RobotLog.dd(TAG, "startPos: %s", startPosition);
      RobotLog.dd(TAG, "parkPos:  %s", parkPosition);
      RobotLog.dd(TAG, "delay:    %s", delay);
   }
}