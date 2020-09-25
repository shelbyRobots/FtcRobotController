package org.firstinspires.ftc.teamcode.image;

import android.graphics.Bitmap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

public interface ImageProcessor
{
    public void startSensing();
    public void stopSensing();

    public void logDebug();
    public void logTelemetry();
    public void setTelemetry(Telemetry telemetry);

    public void setName(String name);
    public String getName();

    public void setImage( Mat img );
    public void setBitmap(Bitmap rgbImage);
    public void saveImage(Mat img);
    public void saveImage();
    public boolean isNewImageReady();
    public void cleanupCamera();
    public Mat draw();
}
