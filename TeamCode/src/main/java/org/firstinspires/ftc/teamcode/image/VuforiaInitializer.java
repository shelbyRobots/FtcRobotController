package org.firstinspires.ftc.teamcode.image;

import android.app.Activity;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.HINT;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.util.CommonUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@SuppressWarnings("unused")
public class VuforiaInitializer
{
    private VuforiaLocalizer vuforia;
    private VuforiaLocalizer.Parameters parameters;
    private static boolean initialized = false;

    private HardwareMap hardwareMap;

    private VuforiaTrackables trackables;
    private List<VuforiaTrackable> allTrackables = new ArrayList<>();

    private static final String TAG = "SJH_vInit";
    private static final double MM_PER_INCH = 25.4;
    private static final int NUM_FRAME_IMAGES = 10;

    public enum Challenge
    {
        VV,
        RR,
        RoRu
    }

    public VuforiaInitializer()
    {
    }

    public VuforiaLocalizer getLocalizer(boolean useScreen)
    {
        if(initialized && vuforia !=null) return vuforia;

        genParameters(useScreen);

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        //Set the image sets to allow getting frames from vuforia
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(NUM_FRAME_IMAGES);

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,2);

        RobotLog.ii("SJH", "Vuforia LicKey: " + parameters.vuforiaLicenseKey);

        initialized = true;

        return vuforia;
    }

    private void genParameters(boolean useScreen)
    {
        CommonUtil com = CommonUtil.getInstance();
        Activity act = com.getActivity();
        String pName = act.getPackageName();
        int viewId = com.getCameraMonitorViewId();

        if(useScreen) parameters = new VuforiaLocalizer.Parameters(viewId);
        else          parameters = new VuforiaLocalizer.Parameters();
        //SJH Teams license
        parameters.vuforiaLicenseKey =
                "AQgIvJ7/////AAAAGQSociXWO0kDvfP15zd4zOsS+fHJygDMLA" +
                "1HhOJQ3FkeiPLGU6YW3ru+jzC6MGxM5tY1ajF4Y0plOpxhQGfS" +
                "R4g3zFiP0IQavezWhGbjBCRMmYu8INy8KvoZ03crZe9wxxQJu9" +
                "9KiNX3ZrbUevNXODKKzWyA9RqxxQHbJ3gpXoff4z1O9n211VOg" +
                "EsJjrNZq8xJnznilyXwc8colJnZD/Adr6UmOzxoUGgaMrdPrlj" +
                "McDJZU6uyoIrOjiv1G2r3iNjtd7LzKAANKrK/0IrO90MgRqQDr" +
                "CAAJVHqqyyubMy8EqE5onzw/WFEcEwfQ6nolsNwYTEZb/JppU8" +
                "9Q6DZmhz4FCT49shA+4PyNOzqsjhRC";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
    }

    private void setupTrackables(String assetName,
                                 List<String> targetNames,
                                 List<OpenGLMatrix> transforms)
    {
        /*
          A brief tutorial: here's how all the math is going to work:

          C = phoneLocationOnRobot     maps   phone coords        -> robot coords
          P = tracker.getPose()        maps   image target coords -> phone coords
          L = redTargetLocationOnField maps   image target coords -> field coords

          C.inverted()                 maps   robot coords -> phone coords
          P.inverted()                 maps   phone coords -> imageTarget coords

          L x P.inverted() x C.inverted() maps robot coords to field coords.

          @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

        RobotLog.dd(TAG, "Setting up trackables " + assetName);
        trackables = vuforia.loadTrackablesFromAsset(assetName);

        RobotLog.dd(TAG, "Set up trackables " + assetName);
        OpenGLMatrix phoneLocationOnRobot = ShelbyBot.phoneLocationOnRobot;
        RobotLog.ii(TAG, "phone=%s", getLocString(phoneLocationOnRobot));

        for(int a = 0; a < trackables.size(); a++)
        {
            VuforiaTrackable t = trackables.get(a);
            t.setName(targetNames.get(a));
            OpenGLMatrix m = transforms.get(a);
            t.setLocation(m);
            VuforiaTrackableDefaultListener l =
                    (VuforiaTrackableDefaultListener) t.getListener();
            l.setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            RobotLog.dd(TAG, "Setup trackable " + t.getName());
            RobotLog.dd(TAG, getLocString(m));
        }

        allTrackables.addAll(trackables);
    }

    List<VuforiaTrackable> setupTrackables(Field fld)
    {
        if (fld != null && fld.getImageNames().size() > 0)
        {
            RobotLog.dd(TAG, "Setting up trackables from %s", fld.getAssetName());

            StringBuilder nsb = new StringBuilder();
            for (String s : fld.getImageNames()) nsb.append(s);

            RobotLog.dd(TAG, "Trackable names %s", nsb.toString());

            StringBuilder osb = new StringBuilder();
            for (OpenGLMatrix o : fld.getImageTransforms()) osb.append(o.toString());

            RobotLog.dd(TAG, "Transforms %s", osb.toString());

            setupTrackables(fld.getAssetName(), fld.getImageNames(), fld.getImageTransforms());
        }
        return allTrackables;
    }

    void setActive(boolean active)
    {
        if(active)
        {
            trackables.activate();
            //vuforia.setFrameQueueCapacity(NUM_FRAME_IMAGES);
        }
        else
        {
            //vuforia.setFrameQueueCapacity(0);
            trackables.deactivate();
        }
    }

    static String getLocString(OpenGLMatrix mat)
    {
        String locStr = null;
        if (mat != null)
        {
            float[] xyz = mat.getTranslation().getData();
            Orientation ori = Orientation.getOrientation(mat,
                    AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

            locStr = String.format(Locale.US,
                    "POS: %5.2f, %5.2f, %5.2f ROT: %4.1f, %4.1f, %4.1f",
                    xyz[0] / MM_PER_INCH, xyz[1] / MM_PER_INCH, xyz[2] / MM_PER_INCH,
                    ori.firstAngle, ori.secondAngle, ori.thirdAngle);
        }
        return locStr;
    }
}