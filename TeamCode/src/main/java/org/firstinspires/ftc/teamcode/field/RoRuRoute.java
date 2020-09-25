package org.firstinspires.ftc.teamcode.field;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Segment;

import java.util.Vector;

@SuppressWarnings("unused")
public class RoRuRoute extends Route
{
    private static final String TAG = "SJH_RRP";

    @Override
    protected Vector<Point2d> initPoints()
    {
        RobotLog.dd(TAG, "In RoroRoute initPoints alliance=%s startPos=%s goForTwo=%s",
                alliance, startPos, goForTwo);
        Vector<Point2d> points = new Vector<>(MAX_SEGMENTS);

        //convenience declarations to make call params shorter
        ShelbyBot.DriveDir fwd = ShelbyBot.DriveDir.INTAKE;
        ShelbyBot.DriveDir rev = ShelbyBot.DriveDir.PUSHER;

        Segment.Action none    = Segment.Action.NOTHING;
        Segment.Action scan    = Segment.Action.SCAN_IMAGE;
        Segment.Action align   = Segment.Action.SET_ALIGN;
        Segment.Action drop    = Segment.Action.DROP;
        Segment.Action push    = Segment.Action.PUSH;
        Segment.Action park    = Segment.Action.PARK;
        Segment.TargetType encType = Segment.TargetType.ENCODER;
        Segment.TargetType colType = Segment.TargetType.COLOR;

//        Point2d tmpPt = new Point2d("TMP", -12.5, 11.5);
        if(startPos == StartPos.START_1) {
            points.add(RoRuField.RLLP);
//            addPoint(points, fwd, 0.5, 1.00, encType, drop, tmpPt);
            addPoint(points, fwd, 0.25, 1.00, colType, align, RoRuField.RLTP);
            addPoint(points, fwd, 0.35, 1.00, encType, scan,  RoRuField.RLTP);
            addPoint(points, fwd, 0.50, 1.00, encType, push,  RoRuField.RLM2);
            addPoint(points, rev, 0.50, 1.00, encType, none,  RoRuField.RLR1);
            addPoint(points, fwd, 0.60, 1.00, encType, none,  RoRuField.RLR2);
            addPoint(points, fwd, 0.60, 1.00, encType, none,  RoRuField.RLDT);
            addPoint(points, fwd, 0.60, 1.00, encType, drop,  RoRuField.RLDP);
            if(goForTwo) {
                 addPoint(points, rev, 0.70, 1.00, encType, push, RoRuField.RRM2);
            }
            else
            {
                addPoint(points, fwd, 0.70, 1.00, encType, none, RoRuField.RLDP);
                addPoint(points, rev, 0.70, 1.00, encType, none, RoRuField.RLDT);
                addPoint(points, rev, 0.70, 1.00, encType, park, RoRuField.RLPP);
        }
        }
        else if(startPos == StartPos.START_2)
        {
            points.add(RoRuField.RRLP);
            addPoint(points, fwd, 0.25, 1.00, colType, align, RoRuField.RRTP);
            addPoint(points, fwd, 0.35, 1.00, encType, scan,  RoRuField.RRTP);
            addPoint(points, fwd, 0.50, 1.00, encType, push,  RoRuField.RRM2);
            addPoint(points, fwd, 0.60, 1.00, encType, drop,  RoRuField.RRDP);
            addPoint(points, rev, 0.60, 1.00, encType, none,  RoRuField.RRR1);
            addPoint(points, rev, 0.60, 1.00, encType, park,  RoRuField.RRPP);
        }

        return points;
    }

    public RoRuRoute(PositionOption startPos,
                     Field.Alliance alliance,
                     String robotName)
    {
        super(startPos, alliance);
    }

    private boolean goForTwo = false;

    public void setGoForTwo(boolean goForTwo)
    {
        this.goForTwo = goForTwo;
    }

    protected Point2d convertRtoB(Point2d rpt)
    {
        double bx = -rpt.getX();
        double by = -rpt.getY();
        String nm = "B" + rpt.getName().substring(1);

        RobotLog.dd(TAG, "convertRtoB %s to %s", rpt.getName(), nm);

        return new Point2d(nm, bx, by);
    }
}