package org.firstinspires.ftc.teamcode.image;

import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.content.res.Resources;
import android.view.View;
import android.view.ViewGroup;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.CommonUtil;

public class LayoutModifier
{
    private Activity act = null;

    private Resources rsrcs;
    private String pName;

    private ViewGroup es;
    private ViewGroup tb;
    private ViewGroup ih;
    private ViewGroup rl;
    private View      nc;
    private View      rs;
    private View      om;
    private View      g1;
    private View      g2;
    private View      wv;

    public LayoutModifier()
    {
        CommonUtil com = CommonUtil.getInstance();
        act   = com.getActivity();
        rsrcs = act.getResources();
        pName = act.getPackageName();
        setupRemoveViews();
    }

    private void setupRemoveViews()
    {
        es = act.findViewById(getId("entire_screen"));
        tb = act.findViewById(getId("top_bar"));
        ih = act.findViewById(getId("included_header"));
        rl = act.findViewById(getId("RelativeLayout"));
        nc = act.findViewById(getId("textNetworkConnectionStatus"));
        rs = act.findViewById(getId("textRobotStatus"));
        om = act.findViewById(getId("textOpMode"));
        g1 = act.findViewById(getId("textGamepad1"));
        g2 = act.findViewById(getId("textGamepad2"));
        wv = act.findViewById(getId("webViewBlocksRuntime"));
    }

    private int getId(String vName)
    {
        return rsrcs.getIdentifier(vName, "id", pName);
    }

    public void configureViewLayout()
    {
        act.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);

        class CameraRunnable implements Runnable
        {
            public void run()
            {
                RobotLog.dd("SJH", "Configuring Layout");
                if(tb != null) es.removeView(tb);
                if(ih != null) es.removeView(ih);
                if(nc != null) rl.removeView(nc);
                if(rs != null) rl.removeView(rs);
                if(om != null) rl.removeView(om);
                if(g1 != null) rl.removeView(g1);
                if(g2 != null) rl.removeView(g2);
                if(wv != null) es.removeView(wv);

                if(rl != null) rl.setPadding(0, 0, 0, 0);
            }
        }

        CameraRunnable mr = new CameraRunnable();
        act.runOnUiThread(mr);
    }

    public void restoreLayout()
    {
        class CameraRunnable implements Runnable
        {
            public void run()
            {
                act.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_UNSPECIFIED);

                if(es != null && tb != null) es.addView(tb, 0);
                if(es != null && ih != null) es.addView(ih, 1);
                if(rl != null && nc != null) rl.addView(nc, 0);
                if(rl != null && rs != null) rl.addView(rs, 1);
                if(rl != null && rs != null) rl.addView(om, 2);
                if(rl != null && g1 != null) rl.addView(g1);
                if(rl != null && g2 != null) rl.addView(g2);
                if(rl != null && es != null) es.addView(wv);
            }
        }

        CameraRunnable mr = new CameraRunnable();
        act.runOnUiThread(mr);
    }
}
