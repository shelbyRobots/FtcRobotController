package org.firstinspires.ftc.teamcode.image;

import android.content.Context;
import android.content.res.Configuration;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Rect;
import android.util.Log;

import org.opencv.android.JavaCameraView;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

public class ModCameraView extends JavaCameraView {
    private static final String TAG = "SJH_CCV";
    private static float lastScale = 0.0f;

    public ModCameraView(Context context, int cameraId) {
        super(context, cameraId);
    }

    @Override
    protected void deliverAndDrawFrame(CvCameraViewFrame frame) {
        Mat modified;

        int deviceOrientation = getContext().getResources().getConfiguration().orientation;

        if (mListener != null) {
            modified = mListener.onCameraFrame(frame);
        } else {
            modified = frame.rgba();
        }

        boolean bmpValid = true;
        if (modified != null) {
            try {
                // fix bitmap size
                if (mCacheBitmap.getWidth() != modified.cols() || mCacheBitmap.getHeight() != modified.rows()) {
                    Log.d("SBH", "mCacheBitmap: " + mCacheBitmap.getWidth() + "x" + mCacheBitmap.getHeight());
                    Log.d("SBH", "modified:     " + modified.cols() + "x" + modified.rows());
                    mCacheBitmap = Bitmap.createBitmap(modified.cols(), modified.rows(), Bitmap.Config.ARGB_8888);
                }
                Utils.matToBitmap(modified, mCacheBitmap);
            } catch(Exception e) {
                Log.e(TAG, "Mat type: " + modified.cols() + "*" + modified.rows());
                Log.e(TAG, "Bitmap type: " + mCacheBitmap.getWidth() + "*" + mCacheBitmap.getHeight());
                Log.e(TAG, "Utils.matToBitmap() throws an exception: " + e.getMessage());
                bmpValid = false;
            }
        }

        if (bmpValid && mCacheBitmap != null) {
            Canvas canvas = getHolder().lockCanvas();
            if (canvas != null) {
                canvas.drawColor(0, android.graphics.PorterDuff.Mode.CLEAR);

                // commented out bc this can add distortion to the image
                // maximize size of the bitmap to remove black borders in portrait orientation
                //mCacheBitmap = Bitmap.createScaledBitmap(mCacheBitmap, canvas.getHeight(), canvas.getWidth(), true);

                if(mScale != lastScale)
                {
                    Log.d("SBH", "scale: " + mScale + " lastscale: " + lastScale);
                }
                lastScale = mScale;

                float scale = mScale;
                if(scale == 0.0f) scale = 1.0f;

                canvas.drawBitmap(mCacheBitmap, new Rect(0,0,mCacheBitmap.getWidth(), mCacheBitmap.getHeight()),
                        new Rect((int)((canvas.getWidth() - mScale*mCacheBitmap.getWidth()) / 2),
                                 (int)((canvas.getHeight() - mScale*mCacheBitmap.getHeight()) / 2),
                                 (int)((canvas.getWidth() - mScale*mCacheBitmap.getWidth()) / 2 + mScale*mCacheBitmap.getWidth()),
                                 (int)((canvas.getHeight() - mScale*mCacheBitmap.getHeight()) / 2 + mScale*mCacheBitmap.getHeight())), null);

                // temporarily rotate canvas to draw FPS meter in correct orientation in portrait
                if(deviceOrientation == Configuration.ORIENTATION_PORTRAIT) {
                    canvas.save();

                    canvas.rotate(-90, getWidth() / 2, getHeight() / 2);

                    if (mFpsMeter != null) {
                        mFpsMeter.measure();
                        mFpsMeter.draw(canvas, 20, 30);
                    }

                    canvas.restore();
                }

                getHolder().unlockCanvasAndPost(canvas);
            }
        }
    }
}
