/* Copyright (c) 2018 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraCalibration;
import com.vuforia.CameraDevice;
import com.vuforia.Vec2F;
import com.vuforia.Vec4F;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


/**
 * This 2018-2019 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "square" field configuration where the red and blue alliance stations
 * are on opposite walls of each other.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * The four vision targets are located in the center of each of the perimeter walls with
 * the images facing inwards towards the robots:
 *     - BlueRover is the Mars Rover image target on the wall closest to the blue alliance
 *     - RedFootprint is the Lunar Footprint target on the wall closest to the red alliance
 *     - FrontCraters is the Lunar Craters image target on the wall closest to the audience
 *     - BackSpace is the Deep Space image target on the wall farthest from the audience
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@SuppressWarnings({"unused", "ConstantConditions"})
@TeleOp(name="Concept: Vuforia Rover Nav", group ="Concept")
//@Disabled
public class ConceptVuforiaNavRoverRuckus extends LinearOpMode {

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
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AQgIvJ7/////AAAAGQSociXWO0kDvfP15zd4zOsS+fHJygDMLA" +
            "1HhOJQ3FkeiPLGU6YW3ru+jzC6MGxM5tY1ajF4Y0plOpxhQGfS" +
            "R4g3zFiP0IQavezWhGbjBCRMmYu8INy8KvoZ03crZe9wxxQJu9" +
            "9KiNX3ZrbUevNXODKKzWyA9RqxxQHbJ3gpXoff4z1O9n211VOg" +
            "EsJjrNZq8xJnznilyXwc8colJnZD/Adr6UmOzxoUGgaMrdPrlj" +
            "McDJZU6uyoIrOjiv1G2r3iNjtd7LzKAANKrK/0IrO90MgRqQDr" +
            "CAAJVHqqyyubMy8EqE5onzw/WFEcEwfQ6nolsNwYTEZb/JppU8" +
            "9Q6DZmhz4FCT49shA+4PyNOzqsjhRC";


    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    @SuppressWarnings("JavaDoc")
    private VuforiaLocalizer vuforia;
    private static final String TAG = "SJH_Vuf";

    @SuppressWarnings("UnusedAssignment")
    @Override public void runOpMode() {

        ManagedGamepad gpad1 = new ManagedGamepad(gamepad1);

        boolean targetVisible;
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        final boolean useFieldLocs = false;
        final boolean useAltDat    = false;
        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        int br;
        int rf;
        int fc;
        int bs;

        if(useAltDat)
        {
            br = 3;
            rf = 2;
            fc = 1;
            bs = 0;
        }
        else
        {
            br = 0;
            rf = 1;
            fc = 2;
            bs = 3;
        }

        String assetName;
        if(useAltDat) assetName ="RoverRuckus_SJH";
        else assetName = "RoverRuckus";
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset(assetName);
        VuforiaTrackable blueRover = targetsRoverRuckus.get(br);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(rf);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(fc);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(bs);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */

        List<VuforiaTrackable> allTrackables = new ArrayList<>(targetsRoverRuckus);

        /*
          In order for localization to work, we need to tell the system where each target is on the field, and
          where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
          Transformation matrices are a central, important concept in the math here involved in localization.
          See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
          for detailed information. Commonly, you'll encounter transformation matrices as instances
          of the {@link OpenGLMatrix} class.

          If you are standing in the Red Alliance Station looking towards the center of the field,
              - The X axis runs from your left to the right. (positive from the center to the right)
              - The Y axis runs from the Red Alliance Station towards the other side of the field
                where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
              - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)

          This Rover Ruckus sample places a specific target in the middle of each perimeter wall.

          Before being transformed, each target image is conceptually located at the origin of the field's
           coordinate system (the center of the field), facing up.
         */

        OpenGLMatrix blueRoverLocationOnField;
        OpenGLMatrix redFootprintLocationOnField;
        OpenGLMatrix frontCratersLocationOnField;
        OpenGLMatrix backSpaceLocationOnField;

        if(useFieldLocs) {
            /*
              To place the BlueRover target in the middle of the blue perimeter wall:
              - First we rotate it 90 around the field's X axis to flip it upright.
              - Then, we translate it along the Y axis to the blue perimeter wall.
             */
            blueRoverLocationOnField = OpenGLMatrix
                    .translation(0, mmFTCFieldWidth, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));

            /*
              To place the RedFootprint target in the middle of the red perimeter wall:
              - First we rotate it 90 around the field's X axis to flip it upright.
              - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
                and facing inwards to the center of the field.
              - Then, we translate it along the negative Y axis to the red perimeter wall.
             */
            redFootprintLocationOnField = OpenGLMatrix
                    .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));

            /*
              To place the FrontCraters target in the middle of the front perimeter wall:
              - First we rotate it 90 around the field's X axis to flip it upright.
              - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
                and facing inwards to the center of the field.
              - Then, we translate it along the negative X axis to the front perimeter wall.
             */
            frontCratersLocationOnField = OpenGLMatrix
                    .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));

            /*
              To place the BackSpace target in the middle of the back perimeter wall:
              - First we rotate it 90 around the field's X axis to flip it upright.
              - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
                and facing inwards to the center of the field.
              - Then, we translate it along the X axis to the back perimeter wall.
             */
            backSpaceLocationOnField = OpenGLMatrix
                    .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        }
        else
        {
            blueRoverLocationOnField    = genMatrix(new float[] {0,0,0}, new float[]{90,0,0});
            redFootprintLocationOnField = genMatrix(new float[] {0,0,0}, new float[]{90,0,0});
            frontCratersLocationOnField = genMatrix(new float[] {0,0,0}, new float[]{90,0,0});
            backSpaceLocationOnField    = genMatrix(new float[] {0,0,0}, new float[]{90,0,0});
        }
        blueRover.setLocation(blueRoverLocationOnField);
        redFootprint.setLocation(redFootprintLocationOnField);
        frontCraters.setLocation(frontCratersLocationOnField);
        backSpace.setLocation(backSpaceLocationOnField);

        /*
          Create a transformation matrix describing where the phone is on the robot.

          The coordinate frame for the robot looks the same as the field.
          The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
          Z is UP on the robot.  This equates to a bearing angle of Zero degrees.

          SHELBY CHANGE:  TREAT THE ROBOT X OUT RIGHT, Y OUT FRONT

          The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
          pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
          camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.

          If using the rear (High Res) camera:
          We need to rotate the camera around it's long axis to bring the rear camera forward.
          This requires a negative 90 degree rotation on the Y axis

          If using the Front (Low Res) camera
          We need to rotate the camera around it's long axis to bring the FRONT camera forward.
          This requires a Positive 90 degree rotation on the Y axis

          Next, translate the camera lens to where it is on the robot.
          In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        int CAMERA_FORWARD_DISPLACEMENT;
        int CAMERA_VERTICAL_DISPLACEMENT;
        int CAMERA_LEFT_DISPLACEMENT;
        //noinspection PointlessBooleanExpression
        if(!useFieldLocs)
        {
            CAMERA_FORWARD_DISPLACEMENT  = 0;
            CAMERA_VERTICAL_DISPLACEMENT = 0;
            CAMERA_LEFT_DISPLACEMENT     = 0;
        }
        else
        {
            CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
            CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
            CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
        }

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 90, 0));

        /*  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        CameraCalibration camCal = com.vuforia.CameraDevice.getInstance().getCameraCalibration();
        Vec4F distParam = camCal.getDistortionParameters();
        Vec2F camFov    = camCal.getFieldOfViewRads();
        Vec2F camFlen   = camCal.getFocalLength();
        Vec2F camPpt    = camCal.getPrincipalPoint();
        Vec2F camSize   = camCal.getSize();

        RobotLog.dd(TAG, "DistortionParams %f %f %f %f",
                distParam.getData()[0],
                distParam.getData()[1],
                distParam.getData()[2],
                distParam.getData()[3]);
        RobotLog.dd(TAG, "CamFOV %f %f", camFov.getData()[0], camFov.getData()[1]);
        RobotLog.dd(TAG, "CamFlen %f %f", camFlen.getData()[0], camFlen.getData()[1]);
        RobotLog.dd(TAG, "CamPpt %f %f", camPpt.getData()[0], camPpt.getData()[1]);
        RobotLog.dd(TAG, "CamSize %f %f", camSize.getData()[0], camSize.getData()[1]);

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");

        telemetry.addData("A", "DistortionParams %f %f %f %f",
                distParam.getData()[0], distParam.getData()[1],
                distParam.getData()[2], distParam.getData()[3]);
        telemetry.addData("B", "CamFOV %f %f", camFov.getData()[0], camFov.getData()[1]);
        telemetry.addData("C", "CamFlen %f %f", camFlen.getData()[0], camFlen.getData()[1]);
        telemetry.addData("D", "CamPpt %f %f", camPpt.getData()[0], camPpt.getData()[1]);
        telemetry.addData("E","CamSize %f %f", camSize.getData()[0], camSize.getData()[1]);

        telemetry.addData("F", "RedOnFld: " + format(redFootprintLocationOnField, EXTRINSIC, XYZ));

        CameraManager cameraManager = ClassFactory.getInstance().getCameraManager();
        CameraName cameraNameFront = cameraManager.nameFromCameraDirection(VuforiaLocalizer.CameraDirection.FRONT);
        CameraName cameraNameBack = cameraManager.nameFromCameraDirection(VuforiaLocalizer.CameraDirection.BACK);
        OpenGLMatrix camOnBot = ((VuforiaTrackableDefaultListener)allTrackables
                .get(1).getListener()).getCameraLocationOnRobot(cameraNameBack);
        telemetry.addData("G", "camOnBot: " + format(camOnBot, EXTRINSIC, XYZ));
        telemetry.update();

        RobotLog.dd(TAG, "DistortionParams %f %f %f %f",
                distParam.getData()[0], distParam.getData()[1],
                distParam.getData()[2], distParam.getData()[3]);
        RobotLog.dd(TAG,"CamFOV %f %f", camFov.getData()[0], camFov.getData()[1]);
        RobotLog.dd(TAG,"CamFlen %f %f", camFlen.getData()[0], camFlen.getData()[1]);
        RobotLog.dd(TAG,"CamPpt %f %f", camPpt.getData()[0], camPpt.getData()[1]);
        RobotLog.dd(TAG,"CamSize %f %f", camSize.getData()[0], camSize.getData()[1]);
        RobotLog.dd(TAG,"RedOnFld: " + format(redFootprintLocationOnField, EXTRINSIC, XYZ));
        RobotLog.dd(TAG,"camOnBot: " + format(camOnBot, EXTRINSIC, XYZ));

        CameraDevice.getInstance().setFlashTorchMode(true) ;
        waitForStart();

        /* Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();

        boolean aPressed;
        String lastTarget = "none";
        while (opModeIsActive())
        {
            OpenGLMatrix pose = null;
            OpenGLMatrix rawPose = null;
            aPressed = false;
            gpad1.update();

            if(gpad1.just_pressed(ManagedGamepad.Button.A))
            {
                RobotLog.dd(TAG, "ButtonA pressed");
                aPressed = true;
            }

            // check all the trackable target to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible())
                {
                    String tgtCap = "Visible Target:";
                    String tgtStr = trackable.getName();

                    telemetry.addData(tgtCap, tgtStr);
                    if(!lastTarget.equals(tgtStr)) tgtCap = "New " + tgtStr;

                    RobotLog.dd(TAG, tgtCap + tgtStr);

                    targetVisible = true;

                    lastTarget = trackable.getName();

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                        pose = ((VuforiaTrackableDefaultListener)trackable.getListener()).getPose();
                        rawPose = ((VuforiaTrackableDefaultListener)trackable.getListener()).getVuforiaCameraFromTarget();
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible)
            {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                Orientation rotation = Orientation.getOrientation(lastLocation, INTRINSIC, ZYX, DEGREES);

                String posCap = "Pos (in)";
                String rotCap = "Rot (deg)";

                String posStr = String.format(Locale.US,
                        "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch,
                        translation.get(1) / mmPerInch,
                        translation.get(2) / mmPerInch);

                String rotStr = String.format(Locale.US,
                        "{Heading, Pitch, Roll} = %.0f, %.0f, %.0f",
                        rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                telemetry.addData(posCap, posStr);
                telemetry.addData(rotCap, rotStr);

                RobotLog.dd(TAG,posCap + posStr);
                RobotLog.dd(TAG,rotCap + rotStr);

                if(pose != null)
                {
                    telemetry.addData("pose",    format(pose, EXTRINSIC, XYZ));
                    telemetry.addData("rawpose", format(rawPose, EXTRINSIC, XYZ));

                    VectorF poseTrans = pose.getTranslation();
                    Orientation poseRot = Orientation.getOrientation(pose, EXTRINSIC, ZYX, DEGREES);

                    String posePosCap = "Pose Pos (in)";
                    String poseRotCap = "Pose Rot (deg)";

                    String posePosStr = String.format(Locale.US,
                            "{X, Y, Z} = %.1f, %.1f, %.1f",
                            poseTrans.get(0) / mmPerInch,
                            poseTrans.get(1) / mmPerInch,
                            poseTrans.get(2) / mmPerInch);
                    String poseRotStr = String.format(Locale.US,
                            "{Heading, Pitch, Roll} = %.0f, %.0f, %.0f",
                            poseRot.firstAngle,
                            poseRot.secondAngle,
                            poseRot.thirdAngle);

                    telemetry.addData(posePosCap, posePosStr);
                    telemetry.addData(poseRotCap, poseRotStr);

                    RobotLog.dd(TAG, posePosCap + posePosStr);
                    RobotLog.dd(TAG, poseRotCap + poseRotStr);
                }
            }
            else
            {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
            sleep(100);
        }
    }

    private static OpenGLMatrix genMatrix(float[] pos, float[] rot) {
        return OpenGLMatrix
                .translation(pos[0], pos[1], pos[2])
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,
                        rot[0], rot[1], rot[2]));
    }

    @SuppressWarnings("SameParameterValue")
    private String format(OpenGLMatrix transformationMatrix, AxesReference axRef, AxesOrder axOrd)
    {
        return transformationMatrix.formatAsTransform(axRef, axOrd, AngleUnit.DEGREES);
    }
}
