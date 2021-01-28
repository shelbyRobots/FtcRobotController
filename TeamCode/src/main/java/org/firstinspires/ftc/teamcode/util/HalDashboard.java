package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * This class is a wrapper for the Telemetry class. In addition to providing a way to send named data to the Driver
 * Station to be displayed, it also simulates an LCD display similar to the NXT Mindstorms. The Mindstorms has only
 * 8 lines but this dashboard can support as many lines as the Driver Station can support. By default, we set the
 * number of lines to 16. By providing the numLines parameter when calling createInstance, you can have as many lines
 * as you want. This dashboard display is very useful for displaying debug information. In particular, the TrcMenu
 * class uses the dashboard to display a choice menu and interact with the user for choosing autonomous strategies
 * and options.
 */
public class HalDashboard
{
    public static final int DEF_NUM_TEXTLINES = 16;

    private static final String displayKeyFormat = "%02d";
    private static HalDashboard instance = null;
    private final Telemetry telemetry;
    private final int numLines;
    private final Telemetry.Item[] display;

    /**
     * This static methods creates an instance of the object if none already exist. If the object exists previously,
     * that instance is returned.
     *
     * @param telemetry specifies the Telemetry object.
     * @param numLines specifies the number of display lines.
     * @return existing instance or newly created instance of the object.
     */
    public static HalDashboard createInstance(Telemetry telemetry, int numLines)
    {
        if (instance == null)
        {
            instance = new HalDashboard(telemetry, numLines);
        }

        return instance;
    }   //createInstance

    /**
     * This static methods creates an instance of the object if none already exist. If the object exists previously,
     * that instance is returned.
     *
     * @param telemetry specifies the Telemetry object.
     * @return existing instance or newly created instance of the object.
     */
    public static HalDashboard createInstance(Telemetry telemetry)
    {
        return createInstance(telemetry, DEF_NUM_TEXTLINES);
    }   //createInstance

    /**
     * This method returns the instance of this object if one already exist, returns null if none existed.
     *
     * @return instance of the object, null if none existed.
     */
    public static HalDashboard getInstance()
    {
        return instance;
    }   //getInstance

    /**
     * Constructor: Creates an instance of the object. There should only be one global instance of this object.
     * Typically, only the FtcOpMode object should construct an instance of this object via getInstance(telemetry)
     * and nobody else.
     *
     * @param telemetry specifies the Telemetry object.
     * @param numLines specifies the number of display lines.
     */
    private HalDashboard(Telemetry telemetry, int numLines)
    {
        this.telemetry = telemetry;
        this.numLines = numLines;
        telemetry.clearAll();
        telemetry.setAutoClear(false);
        display = new Telemetry.Item[numLines];

        for (int i = 0; i < display.length; i++)
        {
            display[i] = telemetry.addData(String.format(Locale.US, displayKeyFormat, i), "");
        }

        telemetry.update();
    }   //HalDashboard

    /**
     * This method returns the number of text lines on the display.
     *
     * @return number of display lines.
     */
    public int getNumTextLines()
    {
        return numLines;
    }   //getNumTextLines

    /**
     * This method displays a text message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param text specifies the text message.
     */
    public void displayText(int lineNum, String text)
    {
        if (lineNum >= 0 && lineNum < numLines)
        {
            display[lineNum].setValue(text);
            telemetry.update();
        }
    }   //displayText

    /**
     * This method clears all the display lines.
     */
    public void clearDisplay()
    {
        for (int i = 0; i < numLines; i++)
        {
            display[i].setValue("");
        }
        telemetry.update();
    }   //clearDisplay

}   //class HalDashboard
