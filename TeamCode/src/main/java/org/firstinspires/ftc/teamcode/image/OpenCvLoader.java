package org.firstinspires.ftc.teamcode.image;

import android.app.Application;
import android.content.pm.ApplicationInfo;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.StringTokenizer;

public class OpenCvLoader
{
    private static final String TAG="SJH_OCL";
    static
    {
        try
        {
            System.loadLibrary("opencv_java3");
            RobotLog.dd(TAG, "opencv library loaded.\n");
        }
        catch (UnsatisfiedLinkError e)
        {
            RobotLog.ee(TAG, "opencv library failed to load.\n" + e);
        }
    }

    private static final boolean printInfo = false;

    public static void showLibDirs()
    {
        if(printInfo) return;
        final AppUtil appUtil = AppUtil.getInstance();
        final Application app = appUtil.getApplication();
        final ApplicationInfo appInfo = app.getApplicationInfo();

        RobotLog.dd(TAG, "AppInfo:\n" +
        " nativeLibraryDir: " + appInfo.nativeLibraryDir + "\n" +
        " name:             " + appInfo.name + "\n" +
        " className:        " + appInfo.className + "\n" +
        " dataDir:          " + appInfo.dataDir + "\n" +
        " packageName:      " + appInfo.packageName + "\n" +
        " toString:         " + appInfo.toString());

        String property = System.getProperty("java.library.path");
        StringTokenizer parser = new StringTokenizer(property, ";");
        while (parser.hasMoreTokens())
        {
            RobotLog.dd(TAG, parser.nextToken());
        }
    }
}
