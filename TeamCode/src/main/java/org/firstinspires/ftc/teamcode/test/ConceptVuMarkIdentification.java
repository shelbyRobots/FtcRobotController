/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.test;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.test.ConceptVuMarkIdentification.JewelColor.NONE;

@SuppressWarnings({"unused", "DanglingJavadoc", "JavaDoc", "WeakerAccess", "ConstantConditions", "FieldCanBeLocal", "RedundantArrayCreation"})
@Autonomous(name="Concept: VuMark Id - OpenCV", group ="Concept")
//@Disabled
public class ConceptVuMarkIdentification extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    public enum JewelColor {RED, BLUE, NONE}
    private JewelColor jewelColor = NONE;
    private JewelColor lastJewel = NONE;
    private static final double THRESHOLD = 0.4;
    private static final int BINS = 8;
    private static final float MIN_VALUE = 0.0f;
    private static final float MAX_VALUE = 255.0f;
    private JewelColor foundColor = NONE;

    private static boolean loaded = false;

    private void initOpenCv()
    {
        RobotLog.dd(TAG, "Initializing OpenCV");
        BaseLoaderCallback mLoaderCallback =
                new BaseLoaderCallback(hardwareMap.appContext)
        {
            @Override
            public void onManagerConnected(int status)
            {
                super.onManagerConnected(status);
            }
        };

        if(!OpenCVLoader.initDebug())
        {
            RobotLog.dd(TAG, "Trying opencv manager");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0,
                    hardwareMap.appContext,
                    mLoaderCallback);
        }
        else
        {
            RobotLog.dd(TAG, "OpenCV loclib connected");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    private void initVuforia()
    {
        RobotLog.dd(TAG, "Initializing Vuforia");
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey =
                "AQgIvJ7/////AAAAGQSociXWO0kDvfP15zd4zOsS+fHJygDMLA" +
                        "1HhOJQ3FkeiPLGU6YW3ru+jzC6MGxM5tY1ajF4Y0plOpxhQGfS" +
                        "R4g3zFiP0IQavezWhGbjBCRMmYu8INy8KvoZ03crZe9wxxQJu9" +
                        "9KiNX3ZrbUevNXODKKzWyA9RqxxQHbJ3gpXoff4z1O9n211VOg" +
                        "EsJjrNZq8xJnznilyXwc8colJnZD/Adr6UmOzxoUGgaMrdPrlj" +
                        "McDJZU6uyoIrOjiv1G2r3iNjtd7LzKAANKrK/0IrO90MgRqQDr" +
                        "CAAJVHqqyyubMy8EqE5onzw/WFEcEwfQ6nolsNwYTEZb/JppU8" +
                        "9Q6DZmhz4FCT49shA+4PyNOzqsjhRC";
        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.GRAYSCALE, false);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,1);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    Bitmap rgbImage = null;

    private Bitmap getImage()
    {
        RobotLog.dd(TAG, "Grabbing frames from vuforia");
        VuforiaLocalizer.CloseableFrame frame;
        try
        {
            frame = vuforia.getFrameQueue().take();
        }
        catch (InterruptedException e)
        {
            RobotLog.ee(TAG, "InterruptedException getting frame");
            return null;
        }

        long numImages = frame.getNumImages();
        RobotLog.dd(TAG, "Grabbed %d images", numImages);

        Image imgData = null;
        for(int i = 0; i < numImages; i++)
        {
            int format = frame.getImage(i).getFormat();
            RobotLog.dd(TAG, "img %d format %d", i, format);
            if(format == PIXEL_FORMAT.RGB565)
            {
                imgData = frame.getImage(i);
                break;
            }
        }

        if(imgData != null)
        {
            int imgW = imgData.getWidth();
            int imgH = imgData.getHeight();

            RobotLog.dd(TAG, "img %d x %d", imgW,imgH);
            Bitmap.Config imgT = Bitmap.Config.RGB_565;
            if(rgbImage == null) rgbImage = Bitmap.createBitmap(imgW, imgH, imgT);
            rgbImage.copyPixelsFromBuffer(imgData.getPixels());
        }
        frame.close();

        return rgbImage;
    }

    Mat showImg;
    Mat cvImg;
    Mat hsvImage;
    Mat histHue;
    Mat histSat;

    MatOfInt chnls;
    Mat mask;
    Mat hist;
    MatOfInt histSize;
    MatOfFloat hRange;
    MatOfFloat sRange;

    private JewelColor getJewelColor()
    {
        RobotLog.dd(TAG, "Looking for jewel");
        vuforia.setFrameQueueCapacity(1);

        double jTimeout = 5.0;
        ElapsedTime jtimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (opModeIsActive() && jewelColor == NONE && jtimer.seconds() < jTimeout)
        {
            Bitmap ballImg = getImage();
            if(ballImg == null)
            {
                RobotLog.dd(TAG, "ballImg is null");
                sleep(100);
                continue;
            }

            int cvt = CvType.CV_8UC1;
            int inH = ballImg.getHeight();
            int inW = ballImg.getWidth();

            RobotLog.dd(TAG, "We have a ball image %d x %d", inH, inW);

            if(cvImg == null) cvImg = new Mat(inH, inW, cvt);
            if(cvImg == null) RobotLog.dd(TAG, "cvImg still null!!");

            Utils.bitmapToMat(rgbImage, cvImg);

            if(showImg == null) showImg = cvImg.clone();
            else cvImg.copyTo(showImg);

            if(hsvImage == null) hsvImage =
                    new Mat(showImg.width(), showImg.height(), showImg.type());

            Imgproc.cvtColor(showImg, hsvImage, Imgproc.COLOR_RGB2HSV);
            List<Mat> channels = new ArrayList<>();
            Core.split(hsvImage, channels);

            if(histHue == null) histHue = new Mat();
            if(histSat == null) histSat = new Mat();
            if(chnls == null) chnls = new MatOfInt(0);
            if(mask == null) mask = new Mat();
            if(histSize == null) histSize = new MatOfInt(BINS);
            if(hRange == null) hRange = new MatOfFloat(MIN_VALUE, 179);
            if(sRange == null) sRange = new MatOfFloat(MIN_VALUE, MAX_VALUE);
            //Histogram for hue
            Imgproc.calcHist(Arrays.asList(new Mat[]{channels.get(0)}),
                    chnls, mask, histHue, histSize, hRange);

            //Histogram for saturation
            Imgproc.calcHist(Arrays.asList(new Mat[]{channels.get(1)}),
                    chnls, mask, histSat, histSize, sRange);

            double sum = Core.sumElems(histHue).val[0];
            double[] values = new double[histHue.height()+histSat.height()];

            RobotLog.dd(TAG, "Sum_Hue %f", sum);
            RobotLog.dd(TAG, "HueMat %dx%d", histHue.width(), histHue.height());

            int k=0;
            for(int i=0; i < histHue.height(); ++i)
            {
                values[k] = histHue.get(i, 0)[0]/sum;
                RobotLog.dd(TAG, "Val_hue %d %f", k, values[k]);
                k++;
            }

            sum = Core.sumElems(histSat).val[0];
            for(int i=0; i < histSat.height(); ++i)
            {
                values[k] = histSat.get(i, 0)[0]/sum;
                k++;
            }

            //0 & 7 Red
            //4 & 5 Blue
            double total = 0.0;

            for(int i=0; i < BINS; i++)
            {
                total+=values[i];
                RobotLog.dd(TAG, "Total %4.2f", total);
            }
            double red = values[0] + values[7];
            double blu = values[4] + values[5];
            double redp = red/total;
            double blup = blu/total;

            if      (redp >= THRESHOLD && redp > blup) foundColor = JewelColor.RED;
            else if (blup >= THRESHOLD && blup > redp) foundColor = JewelColor.BLUE;
            else                                       foundColor = NONE;

        }

        jewelColor = foundColor;
        lastJewel = jewelColor;
        return foundColor;
    }


    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        initOpenCv();
        initVuforia();
        boolean useLight = true;

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        boolean lightOn = false;
        CameraDevice.getInstance().setFlashTorchMode(false);

        if(useLight)
        {
            CameraDevice.getInstance().setFlashTorchMode(true);
            lightOn = true;
        }

        RelicRecoveryVuMark lastKey = RelicRecoveryVuMark.UNKNOWN;
        while (opModeIsActive())
        {
            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                lastKey = vuMark;

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null)
                {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;

                    RobotLog.dd(TAG, "KEY=%s POS=%6.2f %6.2f %6.2f ROT=%6.2f %6.2f &6.2f",
                            vuMark, tX, tY, tZ, rX, rY, rZ);
                }
            }
            else
            {
                telemetry.addData("VuMark", "not visible");
            }

            jewelColor = getJewelColor();

            telemetry.addData("-", "Key %s Ball %s", lastKey, lastJewel);

            if(lastJewel != NONE)
                RobotLog.dd(TAG, "Key %s Ball %s", lastKey, lastJewel);

            telemetry.update();
        }

        if(lightOn) CameraDevice.getInstance().setFlashTorchMode(false);
    }

    String format(OpenGLMatrix transformationMatrix)
    {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
