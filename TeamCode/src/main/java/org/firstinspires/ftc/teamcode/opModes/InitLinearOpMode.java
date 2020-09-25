package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;
import org.firstinspires.ftc.teamcode.util.PreferenceMgr;

import hallib.HalDashboard;

public abstract class InitLinearOpMode extends LinearOpMode
{
    protected CommonUtil com = CommonUtil.getInstance();
    protected static DataLogger dl;
    protected static boolean logData = false;
    protected HalDashboard dashboard;
    protected ManagedGamepad gpad1;
    protected ManagedGamepad gpad2;

    protected PreferenceMgr pmgr;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hardwareMap.logDevices();
    }

    //public void runOpMode(boolean );

    public void initCommon(LinearOpMode op,
                            boolean useVuf,
                            boolean useOcv,
                            boolean usemanFedOcv,
                            boolean cfgLayout)
    {
        com.init(op, useVuf, useOcv, usemanFedOcv, cfgLayout);
        dl = com.getDataLogger();
        dashboard = com.getDashboard();
        gpad1 = new ManagedGamepad(gamepad1);
        gpad2 = new ManagedGamepad(gamepad2);

        pmgr = new PreferenceMgr();
        pmgr.readPrefs();
        //dtrnType = Drivetrain.DrivetrainType.valueOf(pmgr.getRobotConfig());
    }

    @SuppressWarnings("unused")
    public void cleanup()
    {
        if(dl != null) dl.closeDataLogger();
        if(dashboard != null) dashboard.clearDisplay();
    }
}
