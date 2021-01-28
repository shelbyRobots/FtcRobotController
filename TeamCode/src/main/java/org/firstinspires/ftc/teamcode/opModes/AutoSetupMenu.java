package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.PreferenceMgr;

import org.firstinspires.ftc.teamcode.util.FtcChoiceMenu;
import org.firstinspires.ftc.teamcode.util.FtcMenu;
import org.firstinspires.ftc.teamcode.util.FtcValueMenu;

@Autonomous(name = "Auton Config", group = "0")
public class AutoSetupMenu extends InitLinearOpMode implements FtcMenu.MenuButtons {

    private final String TAG = "Auton Menu";

    //The autonomous menu settings using sharedpreferences
    private final PreferenceMgr prfMgr = new PreferenceMgr();
    private String club;
    private String bot;
    private String allianceColor;
    private String startPosition;
    private String parkPosition;
    private int    delay;
    private static final boolean useCps = true;
    private int cps = 1820;

    private int lnum = 1;

    public AutoSetupMenu()
    {
    }

    @SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, false, false, false);
        dashboard.displayText(0, "Starting Menu System");
        setup();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive())
        {
            idle();
        }
    }


    private void setup()
    {
        prfMgr.readPrefs();
        getPrefs();
        dashboard.displayText(0, "INITIALIZING - Please wait for Menu");
        doMenus();
        dashboard.displayText(0, "COMPLETE - Settings Written");
    }

    private void getPrefs()
    {
        club          = prfMgr.getClubName();
        bot           = prfMgr.getBotName();
        allianceColor = prfMgr.getAllianceColor();
        startPosition = prfMgr.getStartPosition();
        parkPosition  = prfMgr.getParkPosition();
        delay         = prfMgr.getDelay();
        cps           = prfMgr.getCps();

        RobotLog.dd(TAG, "Default Config Values:");
        RobotLog.dd(TAG, "Club:     %s", club);
        RobotLog.dd(TAG, "Bot:      %s", bot);
        RobotLog.dd(TAG, "Alliance: %s", allianceColor);
        RobotLog.dd(TAG, "startPos: %s", startPosition);
        RobotLog.dd(TAG, "parkPos:  %s", parkPosition);
        RobotLog.dd(TAG, "delay:    %d", delay);
        RobotLog.dd(TAG, "cps:      %d use: %s", cps, useCps);
    }

    private final String[] botNames = {"GTO1", "MEC1", "MEC2", "MEC3"};
    private final String[] alliances = {"RED", "BLUE"};
    private final String[] startPositions = {"START_1", "START_2"};
    private final String[] parkPositions = {"CENTER_PARK", "DEFEND_PARK"};

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton()   { return gamepad1.dpad_up;}

    @Override
    public boolean isMenuAltUpButton()
    {
        return gamepad1.left_bumper;
    }

    @Override
    public boolean isMenuDownButton() { return gamepad1.dpad_down; }

    @Override
    public boolean isMenuAltDownButton()
    {
        return gamepad1.right_bumper;
    }

    @Override
    public boolean isMenuEnterButton() { return gamepad1.a; }

    @Override
    public boolean isMenuBackButton() { return gamepad1.dpad_left; }

    private void doMenus()
    {
        FtcChoiceMenu<String> botMenu
                = new FtcChoiceMenu<>("Bot:",      null,         this);
        FtcChoiceMenu<String> allianceMenu
                = new FtcChoiceMenu<>("Alliance:", botMenu,      this);
        FtcChoiceMenu<String> startPosMenu
                = new FtcChoiceMenu<>("Start:",    allianceMenu, this);
        FtcChoiceMenu<String> parkMenu
                = new FtcChoiceMenu<>("Park:",     startPosMenu, this);
        FtcValueMenu  delayMenu
                = new FtcValueMenu("Delay:",       parkMenu,     this,
                                          0.0, 20.0, 1.0, 0.0, "%5.2f");
        FtcValueMenu  cpsMenu
            = new FtcValueMenu("CPS:",       delayMenu,     this,
            0, 3000, 20, 1900, "%f");

        //
        // remember last saved settings and reorder the menu with last run settings as the defaults
        //

        botMenu.addChoice(bot, bot, true, allianceMenu);
        for(String str : botNames)
        {
            if(!bot.equals(str))
                botMenu.addChoice(str, str, false, allianceMenu);
        }

        allianceMenu.addChoice(allianceColor, allianceColor, true, startPosMenu);
        for(String str : alliances)
        {
            if(!allianceColor.equals(str))
                allianceMenu.addChoice(str, str, false, startPosMenu);
        }

        startPosMenu.addChoice(startPosition, startPosition, true, parkMenu);
        for(String str : startPositions)
        {
            if(!startPosition.equals(str))
                startPosMenu.addChoice(str, startPosition, false, parkMenu);
        }

        parkMenu.addChoice(parkPosition, parkPosition, true, delayMenu);
        for(String str : parkPositions)
        {
            if(!parkPosition.equals(str))
                parkMenu.addChoice(str, str, false, delayMenu);
        }

        delayMenu.setChildMenu(cpsMenu);

        //
        // Walk the menu tree starting with the strategy menu as the root
        // menu and get user choices.
        //
        FtcMenu.walkMenuTree(botMenu, this);

        //
        // Set choices variables.
        //
        startPosition = startPosMenu.getCurrentChoiceText();
        parkPosition  = parkMenu.getCurrentChoiceText();
        allianceColor = allianceMenu.getCurrentChoiceText();
        bot           = botMenu.getCurrentChoiceText();
        delay = (int)delayMenu.getCurrentValue();
        cps = (int)cpsMenu.getCurrentValue();

        prfMgr.setBotName(bot);
        prfMgr.setStartPosition(startPosition);
        prfMgr.setParkPosition(parkPosition);
        prfMgr.setAllianceColor(allianceColor);
        prfMgr.setDelay(delay);
        prfMgr.setCps(cps);

        RobotLog.dd(TAG, "Writing Config Values:");
        RobotLog.dd(TAG, "Club:     %s", club);
        RobotLog.dd(TAG, "Bot:      %s", bot);
        RobotLog.dd(TAG, "Alliance: %s", allianceColor);
        RobotLog.dd(TAG, "startPos: %s", startPosition);
        RobotLog.dd(TAG, "parkPos:  %s", parkPosition);
        RobotLog.dd(TAG, "delay:    %d", delay);
        RobotLog.dd(TAG, "cps:      %d", cps);

        //write the options to sharedpreferences
        prfMgr.writePrefs();

        //read them back to ensure they were written
        getPrefs();

        dashboard.displayText(lnum++, "Bot:      " + bot);
        dashboard.displayText(lnum++, "Alliance: " + allianceColor);
        dashboard.displayText(lnum++, "Start:    " + startPosition);
        dashboard.displayText(lnum++, "Park:     " + parkPosition);
        dashboard.displayText(lnum++, "Delay:    " + delay);
        dashboard.displayText(lnum++, "Cps:      " + cps);
    }
}
