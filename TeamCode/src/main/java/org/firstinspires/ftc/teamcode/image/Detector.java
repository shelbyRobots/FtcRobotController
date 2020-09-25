package org.firstinspires.ftc.teamcode.image;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Calendar;
import java.util.Locale;

public abstract class Detector implements ImageProcessor
{
    protected String name = "Detector";
    boolean sensingActive = false;
    boolean firstCalcsDone = false;

    /* package-private */ boolean newImage = false;

    /* package-private */ Mat showImg;
    private Mat cvImage;
    private OpenCvInitializer ocvInit = null;
    protected Telemetry telemetry = null;
    private static String dateStr;

    private static String TAG = "SJH_Detector";

    private static int imgNum = 0;

    Detector()
    {
        CommonUtil com = CommonUtil.getInstance();
        ocvInit = com.getOcvInit();
        RobotLog.dd(TAG, "Setting ocvInit image processor in Detector??");
        ocvInit.setImageProcessor(this);
        RobotLog.dd(TAG, "Back from Setting ocvInit image processor");

        Calendar cal = Calendar.getInstance();
        int dom = cal.get(Calendar.DATE);
        int mon = cal.get(Calendar.MONTH) + 1;
        int yr  = cal.get(Calendar.YEAR);
        int hr  = cal.get(Calendar.HOUR_OF_DAY);
        int min = cal.get(Calendar.MINUTE);
        dateStr = String.format(Locale.US,"%4d%02d%02d_%02d%02d", yr, mon, dom, hr, min);
    }

    @Override
    public void startSensing()
    {
        firstCalcsDone = false;
        sensingActive = true;
    }

    public void stopSensing()
    {
        firstCalcsDone = false;
        sensingActive = false;
    }

    public void setImage( Mat img )
    {
        if(img == null) return;
        if(showImg == null) showImg = img.clone();
        else img.copyTo(showImg);

        if ( !sensingActive ) return;
        firstCalcsDone = true;
    }

    public void setBitmap(Bitmap rgbImage)
    {
        if (rgbImage == null) return;

        int cvt = CvType.CV_8UC1;
        int inHeight = rgbImage.getHeight();
        int inWidth = rgbImage.getWidth();

        if (cvImage == null) cvImage = new Mat(inHeight, inWidth, cvt);

        Utils.bitmapToMat(rgbImage, cvImage);
        setImage(cvImage);
    }

    public Mat draw()
    {
        return showImg;
    }

    public void setName(String name)
    {
        this.name = name;
    }

    public String getName()
    {
        return name;
    }

    public void saveImage(Mat img)
    {
        saveImage(img, TAG);
    }

    public void saveImage(Mat img, String imgTag)
    {
        if(img == null) return;
        String fileName = imgTag + "_" + imgNum++ + "_" + dateStr + ".bmp";

        Bitmap bmp = null;
        try
        {
            bmp = Bitmap.createBitmap(img.cols(), img.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(img, bmp);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, e.getMessage());
        }

        String directoryPath  = Environment.getExternalStorageDirectory().getPath() +
                                        "/FIRST/DataLogger";
        String filePath       = directoryPath + "/" + fileName ;

        File dest = new File(filePath);
        FileOutputStream out = null;
        try
        {
            out = new FileOutputStream(dest);
            if(bmp != null)
            {
                // bmp is your Bitmap instance
                bmp.compress(Bitmap.CompressFormat.PNG, 100, out);
            }
            // PNG is a lossless format, the compression factor (100) is ignored

        }
        catch (Exception e)
        {
            e.printStackTrace();
            RobotLog.ee(TAG, e.getMessage());
        }
        finally
        {
            try
            {
                if (out != null)
                {
                    out.close();
                    RobotLog.ii(TAG, "ImageSaved: " + fileName);
                }
            }
            catch (IOException e)
            {
                RobotLog.ee(TAG, e.getMessage());
                e.printStackTrace();
            }
        }
    }

    public void saveImage()
    {
        saveImage(showImg);
    }

    public boolean isNewImageReady()
    {
        boolean newImageReady = newImage;
        newImage = false;
        return newImageReady;
    }

    public void cleanupCamera()
    {
        if(ocvInit != null) ocvInit.cleanupCamera();
    }

    public void setTelemetry(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }
}
