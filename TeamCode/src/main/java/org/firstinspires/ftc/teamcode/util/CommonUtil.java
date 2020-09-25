package org.firstinspires.ftc.teamcode.util;

import android.app.Activity;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.view.View;
import android.view.ViewGroup;
import android.view.ViewParent;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.image.OpenCvInitializer;
import org.firstinspires.ftc.teamcode.image.VuforiaInitializer;

import hallib.HalDashboard;

@SuppressWarnings({"unused", "FieldCanBeLocal"})
public class CommonUtil
{
    private static HardwareMap  h;
    private static OpMode       o;
    private static LinearOpMode l;
    private static Telemetry    t;
    private static HalDashboard d;
    private static DataLogger   dl;

    private static final int topLayoutViewId = R.id.entire_screen;
    private static final int cameraViewId = R.id.cameraMonitorViewId;

    private boolean cfgLayout = false;
    private boolean layoutModified = false;
    private boolean logData   = true;

    private boolean useOpenCV  = false;
    private boolean useManualFedOpenCv = false;
    private boolean useOcvCamera = false;
    private OpenCvInitializer ocvInit;

    private boolean useVuforia = false;
    private boolean vuforiaUseScreen = true;
    private VuforiaInitializer vufInit;
    private static VuforiaLocalizer vuforia = null;

    //private View mainView = null;

    private static final String TAG = "SJH_COM";

    private CommonUtil()
    {
    }

    private static class SingletonHelper
    {
        private static final CommonUtil instance = new CommonUtil();
    }

    public static CommonUtil getInstance()
    {
        return SingletonHelper.instance;
    }

    public void init(OpMode o,
                     boolean useVuforia,
                     boolean useOcv,
                     boolean useManualFedOpenCv,
                     boolean cfgLayout)
    {
        this.useVuforia = useVuforia;
        this.useOpenCV  = useOcv;
        this.useManualFedOpenCv = useManualFedOpenCv;
        this.cfgLayout  = cfgLayout;

        this.useOcvCamera = useOpenCV && !useManualFedOpenCv && !useVuforia;

        CommonUtil.o = o;
        initOpModeProps();
        initDataLogger();
        initDashboard();
        initContextAct();

        initVuforia();
        initOpenCV();
        initOpenCvCamera();
    }

    private  void initOpModeProps()
    {
        CommonUtil.h = o.hardwareMap;
        CommonUtil.t = o.telemetry;
        if (o instanceof LinearOpMode)
            CommonUtil.l = (LinearOpMode) o;
    }

    private void initDataLogger()
    {
        if(dl != null) dl.closeDataLogger();
        if (logData)
        {
            dl = new DataLogger();
        }
    }

    private void initDashboard()
    {
        CommonUtil.d = HalDashboard.createInstance(t);
    }

    private void initContextAct()
    {
        Context context = h.appContext;
        if(context instanceof Activity)
        {
            Activity act = (Activity)context;
            CommonUtil.d.setTextView((TextView)act.findViewById(R.id.textOpMode));
        }
        else
        {
            RobotLog.ee(TAG, "h.appContext is not an Activity");
        }
    }

    private void initVuforia()
    {
        RobotLog.dd(TAG, "initVuforia: " +
                " useVuforia: " + useVuforia);

        if(!useVuforia) return;

        if(cfgLayout)
        {
            setupImageLayout(topLayoutViewId, cameraViewId);
        }

        vufInit = new VuforiaInitializer();
        vuforia = vufInit.getLocalizer(vuforiaUseScreen);
    }

    private void initOpenCV()
    {
        RobotLog.dd(TAG, "initOpenCV: " +
                    " useOpenCV: " + useOpenCV +
                    " useOcvCamera: " + useOcvCamera);
        if(!useOpenCV) return;

        if(cfgLayout)
        {
            setupImageLayout(topLayoutViewId, cameraViewId);
        }

        RobotLog.dd(TAG, "Creating new OpenCvInitializer");
        ocvInit = new OpenCvInitializer(useOcvCamera);
        RobotLog.dd(TAG, "Back from Creating new OpenCvInitializer");
    }

    private void initOpenCvCamera()
    {
        RobotLog.dd(TAG, "initOpenCvCamera: " +
                    " useOpenCV: " + useOpenCV +
                    " useOcvCamera: " + useOcvCamera +
                    " ocvInit null: " + (ocvInit == null ? "true" : "false"));
        //if(!useOcvCamera || ocvInit == null) return;
        if(useVuforia || ocvInit == null) return;
        if(!useOpenCV) return;
        setScreenOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        ocvInit.setupCameraView();
    }

    public HardwareMap getHardwareMap()
    {
        return h;
    }

    public OpMode getOpMode()
    {
        return o;
    }

    public LinearOpMode getLinearOpMode()
    {
        return l;
    }

    public Telemetry getTelemetry()
    {
        return t;
    }

    public HalDashboard getDashboard()
    {
        return d;
    }

    public DataLogger getDataLogger()
    {
        return dl;
    }

    public Activity getActivity()
    {
        Context context = h.appContext;
        Activity act = null;
        if(context instanceof Activity)
        {
            act = (Activity) context;
        }
        return act;
    }

    @SuppressWarnings("WeakerAccess")
    public Context getContext()
    {
        return h.appContext;
    }

    public FtcRobotControllerActivity getApp()
    {
        return (FtcRobotControllerActivity)getActivity();
    }

    public int getCameraMonitorViewId() { return cameraViewId; }

    public boolean getConfigLayout() { return cfgLayout; }

    public OpenCvInitializer getOcvInit()
    {
        return ocvInit;
    }

    public VuforiaInitializer getVuforiaInitializer() { return vufInit;}

    public VuforiaLocalizer getVuforiaLocalizer()
    {
        return vuforia;
    }

    public void addView(final View v, int containerId)
    {
        final Activity act = getActivity();
        final ViewGroup group = act.findViewById(containerId);

        act.runOnUiThread(() -> group.addView(v));
    }

    public void setVisibility(final View v, final int visibility)
    {
        final Activity act = getActivity();

        act.runOnUiThread(new Runnable()
        {
            @Override
            public void run() {
                v.setVisibility(visibility);
            }
        });
    }

    public void removeView(final View v)
    {
        final Activity act = getActivity();
        final ViewGroup parent = (ViewGroup) v.getParent();

        act.runOnUiThread(new Runnable()
        {
            @Override
            public void run()
            {
                parent.removeView(v);
            }
        });
    }

    public void emptyView(int containerId)
    {
        final Activity act = getActivity();
        final ViewGroup group = act.findViewById(containerId);

        RobotLog.dd(TAG, "Emptying ViewId " + containerId);

        act.runOnUiThread(new Runnable()
        {
            @Override
            public void run()
            {
                group.removeAllViews();
            }
        });
    }

    private void setScreenOrientation(final int actInfo)
    {
        final Activity act = getActivity();
        AppUtil appUtil = AppUtil.getInstance();
        appUtil.synchronousRunOnUiThread(act, new Runnable()
           { public void run() { act.setRequestedOrientation(actInfo); }});
    }

    private void setupImageLayout(final int topId, final int botId)
    {
        if(!cfgLayout || layoutModified) return;

        setScreenOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        final Activity act = getActivity();

        class LayoutRunnable implements Runnable
        {
            private boolean layoutConfigured = layoutModified;
            private boolean getLayoutConfigured() { return layoutConfigured; }
            @Override
            public void run()
            {
                RobotLog.dd(TAG, "Setting up ImageLayout");
                ViewGroup top = act.findViewById(topId);
                ViewGroup bot = act.findViewById(botId);
                for (int v = 0; v < top.getChildCount(); v++)
                {
                    top.getChildAt(v).setVisibility(View.GONE);
                }
                ViewGroup gp = (ViewGroup)((bot.getParent()).getParent());

                for (int v = 0; gp != null && v < gp.getChildCount(); v++)
                {
                    gp.getChildAt(v).setVisibility(View.GONE);
                }
                View vw = bot;
                while (vw != null && vw != top)
                {
                    vw.setVisibility(View.VISIBLE);

                    ViewParent vp = vw.getParent();
                    vw = (ViewGroup)vp;
                }
                layoutConfigured = true;
            }
        }

        LayoutRunnable lrun = new LayoutRunnable();
        AppUtil appUtil = AppUtil.getInstance();
        appUtil.synchronousRunOnUiThread(act, lrun);

        layoutModified = true;
    }

    public void restoreLayout()
    {
        if(!cfgLayout || !layoutModified) return;
        final Activity act = getActivity();
        setScreenOrientation(ActivityInfo.SCREEN_ORIENTATION_UNSPECIFIED);
        RobotLog.dd(TAG, "Restoring Layout");

        class LayoutRunnable implements Runnable
        {
            private boolean layoutConfigured = layoutModified;
            private boolean getLayoutConfigured() { return layoutConfigured; }
            @Override
            public void run()
            {
                ViewGroup top = act.findViewById(topLayoutViewId);
                for (int v = 0; v < top.getChildCount() - 1; v++)
                {
                    RobotLog.dd(TAG, "Restoring child %d of %d", v, top.getChildCount());
                    top.getChildAt(v).setVisibility(View.VISIBLE);
                }
                layoutConfigured = false;
            }
        }

        LayoutRunnable lrun = new LayoutRunnable();
        AppUtil appUtil = AppUtil.getInstance();
        appUtil.synchronousRunOnUiThread(act, lrun);

        layoutModified = false;
    }

    //Returns the current root view so it can be reset later
    public View switchFullLayout(final View view)
    {
        final Activity act = getActivity();
        View mainView = getMainView();
        AppUtil appUtil = AppUtil.getInstance();
        appUtil.synchronousRunOnUiThread(act,
           new Runnable()
           {
            @Override
            public void run() { if(view != null) act.setContentView(view); }
           }
        );
        return mainView;
    }

    @SuppressWarnings("ConstantConditions")
    private View getMainView()
    {
        final Activity act = getActivity();
        return act.getCurrentFocus().getRootView();
    }
}
