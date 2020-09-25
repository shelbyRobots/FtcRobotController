package org.firstinspires.ftc.teamcode.image;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Units;

import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import hallib.HalDashboard;

@SuppressWarnings({"WeakerAccess, unused", "FieldCanBeLocal"})
public class ImageTracker
{
    public ImageTracker(Field field)
    {
        cmu = CommonUtil.getInstance();
        dashboard = cmu.getDashboard();
        this.field = field;
        setupVuforia();
        //setTrackableCorners();
    }

    public ImageTracker()
    {
        this(null);
    }

    private void setupVuforia()
    {
        vInit = cmu.getVuforiaInitializer();
        trackables = vInit.setupTrackables(field);
        vuforia = cmu.getVuforiaLocalizer();
    }

    public RelicRecoveryVuMark getKeyLoc() {return keyLoc;}

    @SuppressWarnings("unused")
    public OpenGLMatrix getRobotLocation()
    {
        /*
          getUpdatedRobotLocation() will return null if no new information is available
          since the last time that call was made, or if the trackable is not currently
          visible.
          getRobotLocation() will return null if the trackable is not currently visible.
         */
        OpenGLMatrix robotLocationTransform = null;
        for (VuforiaTrackable trackable : trackables)
        {
            VuforiaTrackableDefaultListener l =
                    (VuforiaTrackableDefaultListener) trackable.getListener();

            RobotLog.dd(TAG, "Trackable " + trackable.getName() + " " + l.isVisible());
            dashboard.displayPrintf(5, "%s", trackable.getName(),
                                             l.isVisible() ? "Visible" : "Not Visible");

            robotLocationTransform = l.getUpdatedRobotLocation();
            if (robotLocationTransform != null)
            {
                lastVisName = trackable.getName();
                break;
            }
        }
        return robotLocationTransform;
    }

    @SuppressWarnings("unused")
    public OpenGLMatrix getImagePose(VuforiaTrackable target, boolean raw)
    {
        VuforiaTrackableDefaultListener l = (VuforiaTrackableDefaultListener) target.getListener();
        if (!l.isVisible()) return null;
        OpenGLMatrix pose;
        String poseType = "";
        if (!raw)
        {
            pose = l.getPose();
        }
        else
        {
            pose = l.getVuforiaCameraFromTarget();
            poseType += "Raw";
        }
        if (pose != null)
        {
            RobotLog.dd("SJH", poseType + "Pose: " + VuforiaInitializer.getLocString(pose));
        }
        return pose;
    }

    public OpenGLMatrix getRawImagePose()
    {
        return rawPose;
    }

    public void updateImages()
    {
        RobotLog.dd(TAG, "ImageTracker.updateImages");
        Bitmap fullPic = getImage();
        lastImage      = fullPic;
        lastCroppedImage = fullPic;
    }

    //public OpenGLMatrix getRobotLocation()
    public void updateRobotLocationInfo()
    {
        /*
          getUpdatedRobotLocation() will return null if no new information is available
          since the last time that call was made, or if the trackable is not currently
          visible.
          getRobotLocation() will return null if the trackable is not currently visible.
         */

        OpenGLMatrix robotLocationTransform;
        for (VuforiaTrackable trackable : trackables)
        {
            VuforiaTrackableDefaultListener l =
                    (VuforiaTrackableDefaultListener) trackable.getListener();
            robotLocationTransform = l.getUpdatedRobotLocation();

            if (robotLocationTransform != null)
            {
                lastVisName = trackable.getName();
                float[] xyz = robotLocationTransform.getTranslation().getData();
                currPos = new Point2d(xyz[0] / Units.MM_PER_INCH,
                                      xyz[1] / Units.MM_PER_INCH);
                currOri = Orientation.getOrientation(robotLocationTransform,
                        AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
                currYaw = (double) currOri.firstAngle;

                updateImages();

                if (field != null && field.hasVuMarks() && keyLoc == RelicRecoveryVuMark.UNKNOWN &&
                    RelicRecoveryVuMark.from(trackable) != RelicRecoveryVuMark.UNKNOWN)
                {
                    keyLoc = RelicRecoveryVuMark.from(trackable);
                    RobotLog.ii("SJH", "VuMark KEY = " + keyLoc);
                    rawPose = getImagePose(trackable, true);
                    dashboard.displayPrintf(6, "VuMark key = %s", keyLoc);
                    RobotLog.dd(TAG, "Rawpose: " + format(rawPose));

                    List<Point2d>  trackablePixCorners =
                            getTrackableCornersInCamera(rawPose);

                    List<Point2d> jewelBoxPixCorners =
                            getCropCornersInCamera(rawPose);
                    if(jewelBoxPixCorners == null) continue;
                    lastCroppedImage = getCroppedImage(jewelBoxPixCorners, lastImage);

                    if (breakOnVumarkFound)
                    {
                        vInit.setActive(false);
                        break;
                    }
                }
                else
                {
                    rawPose = getImagePose(trackable, true);
                    RobotLog.dd(TAG, "Rawpose: " + format(rawPose));
                }
            }
            else
            {
                currPos = null;
                currOri = null;
                currYaw = null;
            }
        }
    }

    public Point2d getSensedPosition()
    {
        return currPos;
    }

    public Double getSensedFldHeading()
    {
        return currYaw;
    }

    public String getLastVisName()
    {
        return lastVisName;
    }

    public String getLocString()
    {
        String locStr = "Unknown sensed location";
        if (currPos != null && currYaw != null)
        {
            locStr = "";
            if (lastVisName != null) locStr = String.format(Locale.US,
                    "%10s ", lastVisName);

            locStr += String.format(Locale.US,
                    "POS: %5.2f, %5.2f  ROT: %4.1f",
                    currPos.getX() / Units.MM_PER_INCH,
                    currPos.getY() / Units.MM_PER_INCH,
                    currYaw);
        }
        return locStr;
    }

    public Bitmap getLastImage()
    {
        return lastImage;
    }

    public Bitmap getLastCroppedImage()
    {
        return lastCroppedImage;
    }

    String format(OpenGLMatrix transformationMatrix)
    {
        return (transformationMatrix != null) ?
                       transformationMatrix.formatAsTransform() :
                       "null";
    }

    public Bitmap getImage()
    {
        VuforiaLocalizer.CloseableFrame frame = null;
        try
        {
            frame = vuforia.getFrameQueue().take();
        } catch (InterruptedException e)
        {
            RobotLog.ee("SJH", "InterruptedException in ImageTracker.getImage");
        }

        if (frame == null)
        {
            RobotLog.ii("SJH", "getImage frame null");
            return null;
        }

        long numImages = frame.getNumImages();
        RobotLog.dd(TAG, "Got %d images", numImages);

        Image imgdata = null;
        for (int i = 0; i < numImages; i++)
        {
            int format = frame.getImage(i).getFormat();
            if (format == PIXEL_FORMAT.RGB565)
            {
                RobotLog.dd(TAG, "Image is RGB");
                imgdata = frame.getImage(i);
                break;
            }
        }

        if (imgdata == null)
        {
            RobotLog.ii("SJH", "imgData null");
            return null;
        }

        int imgW = imgdata.getWidth();
        int imgH = imgdata.getHeight();
        Bitmap.Config imgT = Bitmap.Config.RGB_565;
        if (rgbImage == null) rgbImage = Bitmap.createBitmap(imgW, imgH, imgT);

        rgbImage.copyPixelsFromBuffer(imgdata.getPixels());

        RobotLog.dd(TAG, "Got Image WxH %dx%d", imgW, imgH);

        frame.close();

        return rgbImage;
    }

    private Bitmap getCroppedImage(List<Point2d> cropCorners, Bitmap fullImage)
    {
        Point2d ptl = cropCorners.get(0);
        Point2d ptr = cropCorners.get(1);
        Point2d pbr = cropCorners.get(2);
        Point2d pbl = cropCorners.get(3);

        int minX = (int)Math.min(Math.min(ptl.getX(), pbl.getX()),
                                 Math.min(ptr.getX(), ptl.getX()));
        int maxX = (int)Math.max(Math.max(ptl.getX(), pbl.getX()),
                                 Math.max(ptr.getX(), ptl.getX()));
        int minY = (int)Math.min(Math.min(ptl.getY(), pbl.getY()),
                                 Math.min(ptr.getY(), ptl.getY()));
        int maxY = (int)Math.max(Math.max(ptl.getY(), pbl.getY()),
                                 Math.max(ptr.getY(), ptl.getY()));

        int w = Math.abs(maxX - minX);
        int h = Math.abs(maxY - minY);

        int fullImgW = fullImage.getWidth();
        int fullImgH = fullImage.getHeight();

        if(maxX < 0 || maxY < 0 || minX > fullImgW || maxY > fullImgH)
        {
            RobotLog.ww(TAG, "Crop pixel box outside of full image");
            return null;
        }

        if(minX + w > fullImgW) w = fullImgW - minX;
        if(minY + h > fullImgH) h = fullImgH - minY;
        if(minX < 0) {minX = 0; w = maxX;}
        if(minY < 0) {minY = 0; h = maxY;}

        RobotLog.dd(TAG, "Cropping " + w + "x" + h + " out of " + fullImgW + "x" + fullImgH);

        Bitmap bitmap = Bitmap.createBitmap(fullImage, minX, minY, w, h);

        String fileName = "img_test_" + imgNum++ + ".png";
        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        //String directoryPath  = Environment.getExternalStorageDirectory().getPath() +
        //                                "/FIRST/DataLogger";
        //String filePath       = directoryPath + "/" + fileName ;
        File dest = new File(path, fileName);

        //File dest = new File(filePath);
        FileOutputStream out;
        try
        {
            RobotLog.dd(TAG, "Saving crop file %s %s", path.getName() , fileName);
            out = new FileOutputStream(dest);
            bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
            out.close();
        }
        catch(Exception e)
        {
            RobotLog.ee(TAG, "ERROR saving file. " + e);
        }

        return bitmap;
    }

    /**
     * @param rawPose the pose of the beacon image VuforiaTrackable object
     * @return the list of 4 Point2d for the trackable image corners
     */
    public List<Point2d> getTrackableCornersInCamera(OpenGLMatrix rawPose)
    {
        Matrix34F rawPoseMx = new Matrix34F();
        OpenGLMatrix poseTransposed = rawPose.transposed();

        if (poseTransposed == null) return null;
        rawPoseMx.setData(Arrays.copyOfRange(poseTransposed.getData(), 0, 12));

        //Project the trackable "real" corners in field or trackable picture
        //local coords to pixel coordinates in camera image.
        //To do this, use the inverse of the raw pose matrix and the corner
        //locations from the printed picture size
        com.vuforia.CameraCalibration camCal =
                com.vuforia.CameraDevice.getInstance().getCameraCalibration();

        List<Vec2F> trackableImageCorners = new ArrayList<>(4);
        for(Vec3F vpt : trackableCorners)
        {
            trackableImageCorners.add(Tool.projectPoint(camCal, rawPoseMx, vpt));
        }

        //These are trackable corners in camera pixel space
        List<Point2d> trackablePixelCorners = new ArrayList<>(Arrays.asList(
                new Point2d("TPTL", trackableImageCorners.get(TL).getData()),
                new Point2d("TPTR", trackableImageCorners.get(TR).getData()),
                new Point2d("TPBR", trackableImageCorners.get(BR).getData()),
                new Point2d("TPBL", trackableImageCorners.get(BL).getData())));

        RobotLog.ii(TAG, "trackable size (mm): " + target_width + " , " + target_height);
        RobotLog.ii(TAG, "Trackable corners in trackable sheet space");
        for(Point2d p : trackableSheetCorners)
        {
            RobotLog.ii(TAG, "  " + p);
        }
        RobotLog.ii(TAG, "Trackable corners in pixel space");
        for(Point2d p : trackablePixelCorners)
        {
            RobotLog.ii(TAG, "  " + p);
        }

        return trackablePixelCorners;
    }

    private List<Point2d> getCropCornersInCamera(OpenGLMatrix rawPose)
    {
        Matrix34F rawPoseMx = new Matrix34F();
        OpenGLMatrix poseTransposed = rawPose.transposed();

        if (poseTransposed == null) return null;
        rawPoseMx.setData(Arrays.copyOfRange(poseTransposed.getData(), 0, 12));

        com.vuforia.CameraCalibration camCal =
                com.vuforia.CameraDevice.getInstance().getCameraCalibration();

        List<Vec2F> cropImageCorners = new ArrayList<>(4);
        for(Vec3F vpt : cropCorners)
        {
            cropImageCorners.add(Tool.projectPoint(camCal, rawPoseMx, vpt));
        }

        List<Point2d> cropPixelCorners = new ArrayList<>(Arrays.asList(
                new Point2d("JPTL", cropImageCorners.get(TL).getData()),
                new Point2d("JPTR", cropImageCorners.get(TR).getData()),
                new Point2d("JPBR", cropImageCorners.get(BR).getData()),
                new Point2d("JPBL", cropImageCorners.get(BL).getData())
        ));

        RobotLog.ii(TAG, "Crop corners in trackable sheet space");
        for(Point2d p : cropSheetCorners)
        {
            RobotLog.ii(TAG, "" + p);
        }
        RobotLog.ii(TAG, "Crop corners in pixel space");
        for(Point2d p : cropPixelCorners)
        {
            RobotLog.ii(TAG, "" + p);
        }

        return cropPixelCorners;
    }

    public void setTrackableCorners()
    {
        trackableSheetCorners.clear();
        trackableCorners.clear();

        int htw = target_width/2;
        int hth = target_height/2;

        //These are trackable corners in trackable sheet space - using Point2d for naming
        trackableSheetCorners.add(new Point2d("TSTL", -htw,  hth));
        trackableSheetCorners.add(new Point2d("TSTR",  htw,  hth));
        trackableSheetCorners.add(new Point2d("TSBR",  htw, -hth));
        trackableSheetCorners.add(new Point2d("TSBL", -htw, -hth));

        for(Point2d pt : trackableSheetCorners)
        {
            trackableCorners.add(new Vec3F((float)pt.getX(), (float)pt.getY(), 0.0f));
        }
    }

    public void setTrackableSize(int width, int height)
    {
        target_height = height;
        target_width  = width;
    }

    public void setTrackableRelativeCropCorners(List<Point2d> corners)
    {
        cropSheetCorners = corners;
        cropCorners.clear();

        for(Point2d pt : corners)
        {
            cropCorners.add(new Vec3F((float)pt.getX(), (float)pt.getY(), 0.0f));
        }
    }

    public void resetKey() {keyLoc = RelicRecoveryVuMark.UNKNOWN;}

    public void setFrameQueueSize(int size)
    {
        vuforia.setFrameQueueCapacity(size);
    }

    public void setActive(boolean active)
    {
        vInit.setActive(active);
    }

    // Vuforia units are mm = units used in XML for the trackables
    private static final String TAG = "SJH ImageTracker";

    private static int imgNum = 0;

    private int target_width  = Field.target_width;
    private int target_height = Field.target_height;
    CommonUtil cmu;
    private Field field;
    private VuforiaLocalizer vuforia;
    private VuforiaLocalizer.Parameters parameters;
    private List<VuforiaTrackable> trackables;
    private RelicRecoveryVuMark keyLoc = RelicRecoveryVuMark.UNKNOWN;

    VuforiaInitializer vInit = null;

    private boolean breakOnVumarkFound = true;

    private Point2d currPos = null;
    private Double  currYaw = null;
    private Orientation currOri = null;
    private OpenGLMatrix rawPose = null;
    private String lastVisName = "UNKNOWN";
    private boolean useScreen = true;

    private Bitmap rgbImage = null;
    private Bitmap lastImage = null;
    private Bitmap lastCroppedImage = null;
    private HalDashboard dashboard;
    private boolean configureLayout = false;

    private static final int TL = 0;
    private static final int TR = 1;
    private static final int BR = 2;
    private static final int BL = 3;

    List<Vec3F>   trackableCorners      = new ArrayList<>(4);
    List<Point2d> trackableSheetCorners = new ArrayList<>(4);
    List<Vec3F>   cropCorners           = new ArrayList<>(4);
    List<Point2d> cropSheetCorners      = new ArrayList<>(4);
}
