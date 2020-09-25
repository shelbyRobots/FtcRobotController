package org.firstinspires.ftc.teamcode.image;


import android.app.Activity;
import android.view.View;
import android.view.ViewGroup;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;

@SuppressWarnings("unused")
public class OpenCvInitializer implements CameraBridgeViewBase.CvCameraViewListener2
{
    private final static String TAG = "SJH_OCVI";

    static
    {
        RobotLog.dd(TAG, "Calling initOpenCv");
        OpenCvLoader.showLibDirs();
    }

    private ImageProcessor imgProc = null;
    private static CameraBridgeViewBase openCVCamera = null;

    private Mat flip                    = null;
    private boolean flipImage           = false;
    private boolean addJavaCameraView;
    private static boolean initialized  = false;
    private static boolean initializing = false;

    private int width;
    private int height;

    int getWidth()  {return  width;}
    int getHeight() {return height;}
    public  static boolean isInitializing() {return initializing;}
    private static boolean isInitialized()  {return initialized;}

    public OpenCvInitializer(boolean addJavaCameraView)
    {
        this.addJavaCameraView = addJavaCameraView;
    }

    public void setupCameraView()
    {
        if(openCVCamera != null)
        {
            RobotLog.dd(TAG, "OCVI_setupCameraView: JCV already setup - just update");
            CommonUtil.getInstance().setVisibility(openCVCamera, View.VISIBLE);
            if(addJavaCameraView)
            {
                openCVCamera.enableView();
                RobotLog.dd(TAG, "Set cameraViewListener");
                openCVCamera.setCvCameraViewListener(this);
            }
            return;
        }

        RobotLog.dd(TAG, "OpenCvLoad: setting up JCV");
        class CameraRunnable implements Runnable
        {
            private CameraBridgeViewBase jcv = null;
            private CameraBridgeViewBase getJcv() { return jcv; }
            private CommonUtil com = CommonUtil.getInstance();
            private Activity act = com.getActivity();
            private int camMonViewId = com.getCameraMonitorViewId();

            public void run() {
                RobotLog.dd(TAG, "Setting up JavaCameraView in thread");

                //Add a JavaCameraView child to cameraMonitorView
                ViewGroup vg = act.findViewById(camMonViewId);

                RobotLog.dd(TAG, "Creating JavaCameraView");
                jcv = new ModCameraView(act, CameraBridgeViewBase.CAMERA_ID_BACK);
                //Note:  Check the supported camera resolutions to set this appropriately.
                //       The size used will be the supported size equal to or the next
                //       smaller supported size.
                //       A reasonable  size will help frame rates.
                int maxW = 640;
                int maxH = 480;
                RobotLog.dd(TAG, "setMaxFrameSize %dx%d", maxW, maxH);
                jcv.setMaxFrameSize(maxW, maxH);

                //ViewGroup.LayoutParams.MATCH_PARENT
                ViewGroup.LayoutParams vglop =
                        new ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT,   //maxW
                                                   ViewGroup.LayoutParams.MATCH_PARENT);  //maxH

                RobotLog.dd(TAG, "setLayoutParams");
                jcv.setLayoutParams(vglop);
                RobotLog.dd(TAG, "Adding jcv to cameraMonitorView");
                vg.addView(jcv, 0);
            }
        }

        initializing = true;
        initialized = false;
        RobotLog.dd(TAG, "Creating runnable instance");
        CameraRunnable mr = new CameraRunnable();
        RobotLog.dd(TAG, "Running on UI thread");

        AppUtil.getInstance().synchronousRunOnUiThread(CommonUtil.getInstance().getActivity(), mr);
        RobotLog.dd(TAG, "Started thread");

        CameraBridgeViewBase jcv = mr.getJcv();

        if(jcv == null)
        {
            RobotLog.ww(TAG, "JavaCameraView not created");
            return;
        }

        RobotLog.dd(TAG, "Got JavaCameraView");

        openCVCamera = jcv;
        RobotLog.dd(TAG, "Set visible JavaCameraView");
        openCVCamera.setVisibility(CameraBridgeViewBase.VISIBLE);

        RobotLog.dd(TAG, "Set cameraViewListener");
        openCVCamera.setCvCameraViewListener(this);

        openCVCamera.enableFpsMeter();

        RobotLog.dd(TAG, "OpenCvLoad: enabling view");
        openCVCamera.enableView();
        RobotLog.dd(TAG, "OpenCvLoad: called enableView");

        initializing = false;
        initialized  = true;
    }

    public void cleanupCamera()
    {
        CommonUtil com = CommonUtil.getInstance();
        com.restoreLayout();
        if (openCVCamera != null)
        {
            RobotLog.dd(TAG, "cleanupCamera");
            RobotLog.dd(TAG, "Disable view");
            openCVCamera.disableView();
            com.setVisibility(openCVCamera, View.GONE);
            initialized = false;
        }
    }

    public void setFlipImage(boolean flipImage) {this.flipImage = flipImage;}

    /*package-private*/ void setImageProcessor(ImageProcessor imgProc)
    {
        this.imgProc = imgProc;
        RobotLog.dd(TAG, "Image Processor = " + this.imgProc.getName());
    }

    public ImageProcessor getImageProcessor() { return imgProc; }

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        RobotLog.ii(TAG, "CAMERA VIEW STARTED %4dx%4d", width, height);
        if (openCVCamera != null)
        {
            RobotLog.dd(TAG, "ocv started - enable view");
            //openCVCamera.enableView();
            CommonUtil.getInstance().setVisibility(openCVCamera, View.VISIBLE);
        }
    }

    @Override
    public void onCameraViewStopped()
    {
        RobotLog.ii(TAG, "CAMERA VIEW STOPPED");
        if(imgProc != null)
        {
            RobotLog.dd(TAG, "ocv stopped - stop Sensing");
            imgProc.stopSensing();
        }

        if (openCVCamera != null)
        {
            RobotLog.dd(TAG, "ocv stopped - disable view");
            openCVCamera.disableView();
            CommonUtil.getInstance().setVisibility(openCVCamera, View.GONE);
        }
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame)
    {
        Mat rgb = inputFrame.rgba();

        Mat rtrnImage = rgb;

        if(imgProc != null)
        {
            if(flipImage)
            {
                if(flip == null)
                    flip = rgb.clone();

                Core.flip(rgb, flip, 1);
                imgProc.setImage(flip);
            }
            else
            {
                imgProc.setImage( rgb );
            }

            rtrnImage = imgProc.draw();
        }

        return rtrnImage;
    }
}
