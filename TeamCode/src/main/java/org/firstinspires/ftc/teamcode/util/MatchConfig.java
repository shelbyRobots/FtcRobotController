package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.PositionOption;

public class MatchConfig
{
    public Field.Alliance getAlliance()
    {
        return alliance;
    }

    public void setAlliance(Field.Alliance alliance)
    {
        this.alliance = alliance;
    }

    public PositionOption getStartPos()
    {
        return startPos;
    }

    public void setStartPos(PositionOption startPos)
    {
        this.startPos = startPos;
    }

    public PositionOption getParkPos()
    {
        return parkPos;
    }

    public void setParkPos(PositionOption parkPos)
    {
        this.parkPos = parkPos;
    }

    public String getBotName()
    {
        return botName;
    }

    public void setBotName(String botName)
    {
        this.botName = botName;
    }

    public double getDelay()
    {
        return delay;
    }

    public void setDelay(double delay)
    {
        this.delay = delay;
    }

    private Field.Alliance   alliance;
    private PositionOption   startPos;
    private PositionOption parkPos;
    private String           botName = "";
    private double           delay = 0.0;
}
