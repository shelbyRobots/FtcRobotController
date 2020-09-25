/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.image;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.util.Point2d;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("WeakerAccess")
@Autonomous(name="OldImageNav", group ="Concept")
@Disabled
public class OldImageNav extends LinearOpMode {

    private static final String TAG = "SJH Image Tracker";

    // Vuforia units are mm = units used in XML for the trackables

    private static final float MM_PER_INCH        = 25.4f;

    private VectorF currPos = new VectorF(0.0f, 0.0f, 0.0f);
    private List<VuforiaTrackable> allTrackables = new ArrayList<>();
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackable blueWheels;
    private VuforiaTrackable blueLegos;
    private VuforiaTrackable redTools;
    private VuforiaTrackable redGears;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables ftcImages;
    private String lastVisName = "UNKNOWN";

    private ImageTracker tracker;

    private Bitmap rgbImage = null;
    private Detector detector = new BeaconDetector();
    private BeaconFinder bf = (BeaconFinder)detector;

    private ElapsedTime timer = new ElapsedTime();

    private Point2d curPos = new Point2d(0.0, 0.0);
    private double  curHdg = 0.0;

//    public void setupTrackables()
//    {
//        //To see camera feedback, pass the view id
//        //For competition, we don't want this - so use the no param ctor
//        if(useScreen)
//        {
//            parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
//        }
//        else
//        {
//            parameters = new VuforiaLocalizer.Parameters();
//        }
//
//        //SJH Teams license
//        parameters.vuforiaLicenseKey =
//                "AQgIvJ7/////AAAAGQSociXWO0kDvfP15zd4zOsS+fHJygDMLA" +
//                        "1HhOJQ3FkeiPLGU6YW3ru+jzC6MGxM5tY1ajF4Y0plOpxhQGfS" +
//                        "R4g3zFiP0IQavezWhGbjBCRMmYu8INy8KvoZ03crZe9wxxQJu9" +
//                        "9KiNX3ZrbUevNXODKKzWyA9RqxxQHbJ3gpXoff4z1O9n211VOg" +
//                        "EsJjrNZq8xJnznilyXwc8colJnZD/Adr6UmOzxoUGgaMrdPrlj" +
//                        "McDJZU6uyoIrOjiv1G2r3iNjtd7LzKAANKrK/0IrO90MgRqQDr" +
//                        "CAAJVHqqyyubMy8EqE5onzw/WFEcEwfQ6nolsNwYTEZb/JppU8" +
//                        "9Q6DZmhz4FCT49shA+4PyNOzqsjhRC";
//
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//        Vuforia.setFrameFormat( PIXEL_FORMAT.RGB565, true );
//
//        RobotLog.ii("SJH", "Vuforia LicKey: " + parameters.vuforiaLicenseKey);
//
//        ftcImages = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
//        //Wheels are on blue side closest to blue corner
//        blueWheels = ftcImages.get(0);
//        blueWheels.setName("BlueWheels");
//
//        //Legos are on blud side furthest from blue corner
//        blueLegos = ftcImages.get(2);
//        blueLegos.setName("BlueLegos");
//
//        //Tools are on red side furthest from red corner
//        redTools = ftcImages.get(1);
//        redTools.setName("RedTools");
//
//        //Gears are on red side closest to red corner
//        redGears = ftcImages.get(3);
//        redGears.setName("RedGears");
//
//        allTrackables.addAll(ftcImages);
//
//        redTools.setLocation(VvField.redToolsLocationOnField);
//        RobotLog.ii(TAG, "Red Tools=%s", format(redTools.getLocation()));
//
//        redGears.setLocation(VvField.redToolsLocationOnField);
//        RobotLog.ii(TAG, "Red Gears=%s", format(redGears.getLocation()));
//
//        blueWheels.setLocation(VvField.blueWheelsLocationOnField);
//        RobotLog.ii(TAG, "Blue Wheels=%s", format(blueWheels.getLocation()));
//
//        blueLegos.setLocation(VvField.blueLegosLocationOnField);
//        RobotLog.ii(TAG, "Blue Legos=%s", format(blueLegos.getLocation()));
//    }
//
//    public void setupPhoneOnRobot()
//    {
//        OpenGLMatrix phoneLocationOnRobot = ShelbyBot.phoneLocationOnRobot;
//        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));
//
//        /**
//         * A brief tutorial: here's how all the math is going to work:
//         *
//         * C = phoneLocationOnRobot     maps   phone coords        -> robot coords
//         * P = tracker.getPose()        maps   image target coords -> phone coords
//         * L = redTargetLocationOnField maps   image target coords -> field coords
//         *
//         * So
//         *
//         * C.inverted()                 maps   robot coords -> phone coords
//         * P.inverted()                 maps   phone coords -> imageTarget coords
//         *
//         * Putting that all together,
//         *
//         * L x P.inverted() x C.inverted() maps robot coords to field coords.
//         *
//         * @see VuforiaTrackableDefaultListener#getRobotLocation()
//         */
//
//        ((VuforiaTrackableDefaultListener)redTools.getListener()).
//                                                                         setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//        ((VuforiaTrackableDefaultListener)redGears.getListener()).
//                                                                         setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//        ((VuforiaTrackableDefaultListener)blueLegos.getListener()).
//                                                                          setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//        ((VuforiaTrackableDefaultListener)blueWheels.getListener()).
//                                                                           setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//    }


//    public OpenGLMatrix getRobotLocation()
//    {
//        /**
//         * getUpdatedRobotLocation() will return null if no new information is available
//         * since the last time that call was made, or if the trackable is not currently
//         * visible.
//         * getRobotLocation() will return null if the trackable is not currently visible.
//         */
//        OpenGLMatrix robotLocationTransform = null;
//        for (VuforiaTrackable trackable : allTrackables)
//        {
//            robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener())
//                                             .getUpdatedRobotLocation();
//            if(robotLocationTransform != null)
//            {
//                lastVisName = trackable.getName();
//                break;
//            }
//        }
//        return robotLocationTransform;
//    }

//    public String getLocString(OpenGLMatrix mat)
//    {
//        String locStr = null;
//        if(mat != null)
//        {
//            float xyz[] = mat.getTranslation().getData();
//            Orientation ori = Orientation.getOrientation(lastLocation,
//                    AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
//
//            locStr = String.format(Locale.US,
//                    "%10s POS: %5.2f, %5.2f, %5.2f ROT: %4.1f, %4.1f, %4.1f",
//                    lastVisName, xyz[0] / MM_PER_INCH, xyz[1] / MM_PER_INCH, xyz[2] / MM_PER_INCH,
//                    ori.firstAngle, ori.secondAngle, ori.thirdAngle);
//        }
//        return locStr;
//    }

//    public Bitmap getImage()
//    {
//        VuforiaLocalizer.CloseableFrame frame = null;
//        try
//        {
//            frame = vuforia.getFrameQueue().take();
//        }
//        catch (InterruptedException e)
//        {
//            RobotLog.ii("SJH", "What is going on here");
//        }
//
//        if(frame == null) return null;
//        long numImages = frame.getNumImages();
//
//        Image imgdata = null;
//        for (int i = 0; i < numImages; i++)
//        {
//            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565)
//            {
//                imgdata = frame.getImage(i);
//                break;
//            }
//        }
//
//        //frame.close();
//        if(imgdata == null) return null;
//
//        int imgW = imgdata.getWidth();
//        int imgH = imgdata.getHeight();
//        Bitmap.Config imgT = Bitmap.Config.RGB_565;
//        if(rgbImage == null) rgbImage = Bitmap.createBitmap(imgW, imgH, imgT);
//        rgbImage.copyPixelsFromBuffer(imgdata.getPixels());
//
//        return rgbImage;
//    }

    private boolean findSensedLoc()
    {
        RobotLog.ii("SJH", "findSensedLoc");
        telemetry.addData("2", "STATE: %s", "FIND IMG LOC");
        curPos = null;
        ElapsedTime itimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        tracker.setActive(true);
        Point2d sensedBotPos = null;
        double  sensedFldHdg = 90.0;
        OpenGLMatrix robotLocationTransform;
        int frm = 0;
        while(sensedBotPos == null && itimer.milliseconds() < 1000)
        {
            tracker.updateRobotLocationInfo();
            sensedBotPos = tracker.getSensedPosition();

            if(sensedBotPos != null)
            {
                curPos = sensedBotPos;
                sensedFldHdg = tracker.getSensedFldHeading();
                curHdg = sensedFldHdg;
                tracker.setActive(false);
            }
            else
            {
                sleep(50);
            }
            frm++;
        }

        if ( sensedBotPos != null )
        {
            double t = itimer.seconds();
            RobotLog.ii("SJH", "Senesed Pos: %s %5.2f %2.3f", sensedBotPos, sensedFldHdg, t);
            RobotLog.ii("SJH", "IMG %s frame %d", tracker.getLocString(), frm);
            telemetry.addData("SLOC", "SLOC: %s %4.1f", sensedBotPos, sensedFldHdg);
            telemetry.addData("IMG", "%s  frame %d", tracker.getLocString(), frm);
        }
        else
        {
            telemetry.addData("4", "SENSLOC: %s", "NO VALUE");
        }

        return (curPos != null);
    }

    @Override
    public void runOpMode()
    {
        tracker = new ImageTracker();
//        setupTrackables();
//        setupPhoneOnRobot();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        waitForStart();

        timer.reset();

        telemetry.addData(":", "Visual Cortex activated!");
        RobotLog.ii("SJH", "Visual Cortex activated!");
        telemetry.update();

        Point2d sensedBotPos = null;

        tracker.setActive(true);
        OpenGLMatrix robotLocationTransform;


        while (opModeIsActive())
        {
            boolean locFound = findSensedLoc();
            //do_findBeaconOrder(false);
            sleep(2000);
            idle();
        }
        tracker.setActive(false);

//        while (opModeIsActive())// && timer.seconds() < 100
//        {
//            tracker.updateRobotLocationInfo();
//            sensedBotPos = tracker.getSensedPosition();
//            /** Start tracking the data sets we care about. */
//            robotLocationTransform = tracker.getRobotLocation();
//            if (robotLocationTransform != null)
//            {
//                lastLocation = robotLocationTransform;
//                currPos = lastLocation.getTranslation();
//                String locStr = VuforiaInitializer.getLocString(lastLocation);
//                telemetry.addData("LOC", locStr);
//                RobotLog.ii("SJH", locStr);
//            }
//
//            tracker.setFrameQueueSize(10);
//            Bitmap rgbImage = tracker.getImage();
//
//            if(rgbImage == null) continue;
//            detector.setBitmap(rgbImage);
//            tracker.setFrameQueueSize(0);
//            tracker.setActive(false);
//
//            RobotLog.ii("SJH", "Beacon Color: " + bf.getLightOrder());
//            telemetry.addData("Beacon Color: ", bf.getLightOrder());
//
//            telemetry.update();
//            idle();
//        }
//        tracker.setActive(false);

        detector.cleanupCamera();
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix)
    {
        return transformationMatrix.formatAsTransform();
    }
}
