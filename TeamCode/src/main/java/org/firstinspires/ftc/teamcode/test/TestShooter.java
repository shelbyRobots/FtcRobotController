package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

import java.util.List;

@Config
@TeleOp(name = "TestShooter", group = "Test")
//@Disabled
public class TestShooter extends InitLinearOpMode
{
    private static final double INCREMENT = 3.0;     // amount to step motor each CYCLE_MS cycle
    private static final int     CYCLE_MS = 20;       // period of each cycle
    private static final double   MAX_DIST = 136;     // Maximum that we can shoot
    private static final double   MIN_DIST = 0;     // Minimum that we can shoot.
    private static final double   FAV_DIST = 70;
    private double cps = 0.0;
    private static final double MIN_CPS = 0.0;
    //6000RPM/60 for RPS * 28 CPR for 1:1 goBilda motor = 2800
    private static final double MAX_CPS = (6000.0/60.0) * 28;
    private static final double CPS_INC = 20.0;

    double lastKp = RobotConstants.SH_PID.p;
    double lastKi = RobotConstants.SH_PID.i;
    double lastKd = RobotConstants.SH_PID.d;
    double lastKf = RobotConstants.SH_PID.f;
    
    public static PIDFCoefficients pidf = RobotConstants.SH_PID;

    private FtcDashboard dbd;

    private static Shooter shooter;

    private static final String TAG = "SJH_TSH";

    private void doLogging()
    {
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("pos", shooter.getEncPos());
        packet.put("spd", shooter.getCurSpd());
        packet.put("cmd", cps);
        packet.put("dst", shooter.getDist());
        dbd.sendTelemetryPacket(packet);

        String shtStr = shooter.toString();
        RobotLog.dd(TAG, shtStr);
        //dashboard.displayText(0, shtStr);
    }

    @Override
    public void runOpMode()
    {
        initCommon(this, false, false, false, false);

        dbd = FtcDashboard.getInstance();
        dbd.setTelemetryTransmissionInterval(25);

        shooter = new Shooter(hardwareMap);
        shooter.init();

        shooter.setPIDF(pidf);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs)
        {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        double distance = FAV_DIST;

        // Wait for the start button
        while (!isStarted())
        {
            shooter.update();
            doLogging();
            sleep(CYCLE_MS);
        }
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive())
        {
            for (LynxModule module : allHubs)
            {
                module.clearBulkCache();
            }

            if (lastKp != pidf.p || lastKd != pidf.d || lastKi != pidf.i || lastKf != pidf.f)
            {
                shooter.setPIDF(pidf);

                lastKp = pidf.p;
                lastKi = pidf.i;
                lastKd = pidf.d;
                lastKf = pidf.f;
            }


            shooter.update();
            gpad1.update();

            boolean step_up    = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean step_down  = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean zeroize    = gpad1.just_pressed(ManagedGamepad.Button.D_LEFT);
            boolean normal     = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);
            boolean shtInc     = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
            boolean shtDec     = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);

            if (shtInc) {cps += CPS_INC; cps = Math.min(cps, MAX_CPS);}
            else if (shtDec) {cps -=  CPS_INC; cps = Math.max(cps, MIN_CPS);}

            if(shtInc || shtDec) shooter.shootCps(cps);

            if (step_up && distance < MAX_DIST) {
                distance += INCREMENT;
                shooter.shotSpeed(distance);
                cps = shooter.getCmdSpd();
            }
            if (step_down && distance > MIN_DIST) {
                distance -= INCREMENT;
                shooter.shotSpeed(distance);
                cps = shooter.getCmdSpd();
            }
            if (zeroize)
            {
                shooter.stop();
                cps = shooter.getCmdSpd();
            }
            if (normal){
                distance = FAV_DIST;
                shooter.shotSpeed(distance);
                cps = shooter.getCmdSpd();
            }

            doLogging();

            sleep(CYCLE_MS);
        }
        shooter.stop();
    }
}
