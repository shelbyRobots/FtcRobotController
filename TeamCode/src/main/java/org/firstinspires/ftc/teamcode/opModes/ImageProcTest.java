package org.firstinspires.ftc.teamcode.opModes;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.PositionOption;
import org.firstinspires.ftc.teamcode.field.RoRuField;
import org.firstinspires.ftc.teamcode.field.Route;
import org.firstinspires.ftc.teamcode.image.Detector;
import org.firstinspires.ftc.teamcode.image.ImageTracker;
import org.firstinspires.ftc.teamcode.image.MineralDetector;
import org.firstinspires.ftc.teamcode.image.VuforiaInitializer;
import org.firstinspires.ftc.teamcode.util.Point2d;

@SuppressWarnings({"unused", "ForLoopReplaceableByForEach"})
@Autonomous(name="ImgProcTest", group="Auton")
//@Disabled
public class ImageProcTest extends InitLinearOpMode
{
    public ImageProcTest()
    {
        //super();
    }

    private void startMode()
    {
        dashboard.clearDisplay();
        do_main_loop();
    }

    @SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotLog.dd(TAG, "initCommon");
        initCommon(this, true, true, false, false);

        setup();

        timer.reset();

        waitForStart();

        startMode();
        stopMode();
    }

    private void stopMode()
    {
        if (tracker != null)
        {
            tracker.setFrameQueueSize(0);
            tracker.setActive(false);
        }
        if(det != null) det.cleanupCamera();
    }

    private void setup()
    {
        dashboard.displayPrintf(0, "INITIALIZING");

        det = new MineralDetector();
        RobotLog.dd(TAG, "Setting up vuforia");
        tracker = new ImageTracker(new RoRuField());

        det.setTelemetry(telemetry);
    }

    private void do_main_loop()
    {
        timer.reset();
        startTimer.reset();
        dl.resetTime();

        doFindLoc();

        doScan();

        while(opModeIsActive() && !isStopRequested())
        {
            idle();
        }
    }

    private void doFindLoc()
    {
        //Try to use Vuf localization to find loc
        //Turn to NSEW depending on startpos to sens loc
        tracker.setActive(true);
        Point2d sensedPos = null;
        Double  sensedHdg = null;
        String sensedImg = null;
        ElapsedTime imgTimer = new ElapsedTime();

        RobotLog.dd(TAG, "doFindLoc");

        while(opModeIsActive()         &&
                imgTimer.seconds() < 1.0 &&
                (sensedPos == null || sensedHdg == null))
        {
            tracker.updateRobotLocationInfo();
            sensedPos = tracker.getSensedPosition();
            sensedHdg = tracker.getSensedFldHeading();
            sensedImg = tracker.getLastVisName();
        }

        if(sensedPos != null) RobotLog.dd(TAG, "SENSED POS " + sensedImg + " " + sensedPos);
        if(sensedHdg != null) RobotLog.dd(TAG, "SENSED HDG " + sensedImg + " " + sensedHdg);

        tracker.setActive(false);
    }

    private void doScan()
    {
        RobotLog.dd(TAG, "doScan");
        if(useLight)
            CameraDevice.getInstance().setFlashTorchMode(true) ;

        mineralPos =  getMineralPos();
        //mineralPos = MineralDetector.Position.CENTER;
        RobotLog.dd(TAG, "doScan mineralPos = %s", mineralPos);

        if(useLight)
            CameraDevice.getInstance().setFlashTorchMode(false);

        RobotLog.dd(TAG, "Getting mineralPt for %s %s %s", alliance, startPos, mineralPos);
        Point2d tgtMinPt = RoRuField.getMineralPt(alliance, startPos, mineralPos);
        RobotLog.dd(TAG, "MineralPt = %s", tgtMinPt);
    }

    private MineralDetector.Position getMineralPos()
    {
        if(mineralPos != MineralDetector.Position.NONE || !opModeIsActive())
            return mineralPos;

        tracker.setActive(true);
        mineralPos = MineralDetector.Position.NONE;
        RobotLog.dd(TAG, "Set qsize to get frames");
        tracker.setFrameQueueSize(1);
        RobotLog.dd(TAG, "Start LD sensing");
        det.startSensing();

        MineralDetector.Position minPos = MineralDetector.Position.NONE;

        double mineralTimeout = 1.0;
        ElapsedTime mtimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(opModeIsActive()                                   &&
                minPos == MineralDetector.Position.NONE &&
                mtimer.seconds() < mineralTimeout)
        {
            tracker.updateImages();
            Bitmap rgbImage = tracker.getLastImage();

            boolean tempTest = false;
            if(rgbImage == null)
            {
                RobotLog.dd(TAG, "getMineralPos - image from tracker is null");
                //noinspection ConstantConditions
                if(!tempTest) continue;
            }
            det.setBitmap(rgbImage);
            det.logDebug();
            det.logTelemetry();
            if(det instanceof MineralDetector)
                minPos = ((MineralDetector) det).getMineralPos();

            if(minPos == MineralDetector.Position.NONE)
                sleep(100);
        }

        det.stopSensing();
        tracker.setFrameQueueSize(0);
        tracker.setActive(false);

        dashboard.displayPrintf(1, "MIN: " + minPos);

        return minPos;
    }

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime startTimer = new ElapsedTime();

    private Detector det;
    private static ImageTracker tracker;
    private MineralDetector.Position mineralPos = MineralDetector.Position.NONE;

    private static PositionOption startPos = Route.StartPos.START_1;
    private static Field.Alliance alliance = Field.Alliance.RED;

    @SuppressWarnings("FieldCanBeLocal")
    private boolean useLight = true;

    @SuppressWarnings("FieldCanBeLocal")
    private String robotName = "";
    private static final String TAG = "SJH_RRA";
}