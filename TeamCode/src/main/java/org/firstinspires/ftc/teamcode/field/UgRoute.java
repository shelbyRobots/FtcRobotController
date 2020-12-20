package org.firstinspires.ftc.teamcode.field;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Segment;

import java.util.Vector;

@SuppressWarnings("unused")
public class UgRoute extends Route
{
    private static final String TAG = "SJH_RRP";

    @Override
    protected Vector<Point2d> initPoints()
    {
        RobotLog.dd(TAG, "In UgRoute initPoints alliance=%s startPos=%s goForTwo=%s",
                alliance, startPos, goForTwo);
        Vector<Point2d> points = new Vector<>(MAX_SEGMENTS);

        //convenience declarations to make call params shorter
        ShelbyBot.DriveDir fwd = RobotConstants.DT_DIR;
        ShelbyBot.DriveDir rev = ShelbyBot.DriveDir.INTAKE;
        if(fwd == ShelbyBot.DriveDir.INTAKE) rev = ShelbyBot.DriveDir.PUSHER;
        ShelbyBot.DriveDir rgt = ShelbyBot.DriveDir.RIGHT;
        ShelbyBot.DriveDir lft = ShelbyBot.DriveDir.LEFT;

        Segment.Action none    = Segment.Action.NOTHING;
        Segment.Action scan    = Segment.Action.SCAN_IMAGE;
        Segment.Action algn    = Segment.Action.SET_ALIGN;
        Segment.Action drop    = Segment.Action.DROP;

        Segment.Action park    = Segment.Action.PARK;
        Segment.Action push    = Segment.Action.PUSH;
        Segment.Action rtct    = Segment.Action.RETRACT;
        Segment.Action grab    = Segment.Action.GRAB;
        Segment.Action shot    = Segment.Action.SHOOT;
        Segment.TargetType encType = Segment.TargetType.ENCODER;
        Segment.TargetType colType = Segment.TargetType.COLOR;

        ShelbyBot.DriveDir sdr = fwd;

        boolean runTest = false;
        boolean useStrafe = true;

        if(RobotConstants.bot == RobotConstants.Chassis.MEC1) sdr = rev;

        boolean goForTwo = true;

        if(startPos == StartPos.START_1)  //Wall Start
        {
            if(runTest)
            {
                Point2d tp1 = new Point2d("TP1", 0.0,  0.0);
                Point2d tp2 = new Point2d("TP2", 0.05, 0.0);
                Point2d tp3 = new Point2d("TP2", 0.0, 48.0);
                points.add(tp1);
                addPoint(points, fwd, 0.30, 1.00, encType, none, tp2);
                addPoint(points, lft, 0.30, 1.00, encType, shot, tp3);
                addPoint(points, rgt, 0.40, 1.00, encType, none, tp1);
                return points;
            }

            points.add(UgField.ROS1);
            addPoint(points, fwd, 0.60, 1.00, encType, scan, UgField.ROSP);
            addPoint(points, fwd, 0.60, 1.00, encType, none, UgField.ROTP);
            addPoint(points, fwd, 0.60, 1.00, encType, none, UgField.RODP);
            addPoint(points, fwd, 0.50, 1.00, encType, drop, UgField.ROWA);
            addPoint(points, rev, 0.60, 1.00, encType, none, UgField.RODP);
            addPoint(points, sdr, 0.50, 1.00, encType, shot, UgField.ROSA);
            if (goForTwo)
            {
                if(useStrafe)
                {
                    addPoint(points, rev, 0.60, 1.00, encType, none, UgField.RODP);
                    addPoint(points, rev, 0.60, 1.00, encType, none, UgField.ROSS);
                    addPoint(points, lft, 0.30, 1.00, encType, none, UgField.ROSE);
                    addPoint(points, fwd, 0.60, 1.00, encType, none, UgField.ROW2);
                }
                else
                {
                    addPoint(points, fwd, 0.60, 1.00, encType, none, UgField.ROT1);
                    addPoint(points, rev, 0.60, 1.00, encType, none, UgField.ROT2);
                    addPoint(points, fwd, 0.60, 1.00, encType, none, UgField.ROT3);
                    addPoint(points, fwd, 0.60, 1.00, encType, none, UgField.RODP);
                }

                addPoint(points, fwd, 0.60, 1.00, encType, drop, UgField.ROWA);
            }
                addPoint(points, rev, 0.50, 1.00, encType, park, UgField.ROPA);
        }
        else if(startPos == StartPos.START_2)  //Center Start
        {
            points.add(UgField.RIS1);
            addPoint(points, fwd, 0.60, 1.00, encType, scan, UgField.RISP);
            addPoint(points, fwd, 0.60, 1.00, encType, none, UgField.RITP);
            addPoint(points, fwd, 0.60, 1.00, encType, none, UgField.RIDP);
            addPoint(points, fwd, 0.50, 1.00, encType, drop, UgField.RIWA);
            addPoint(points, rev, 0.60, 1.00, encType, none, UgField.RIDP);
            addPoint(points, sdr, 0.60, 1.00, encType, shot, UgField.RISA);
           /* if (goForTwo) {
                addPoint(points, rev, 0.60, 1.00, encType, none, UgField.RITP);
                addPoint(points, rev, 0.60, 1.00, encType, none, UgField.RIGW);
                addPoint(points, fwd, 0.60, 1.00, encType, none, UgField.RIT2);
                addPoint(points, fwd, 0.60, 1.00, encType, drop, UgField.RIWA);
            }*/
            addPoint(points, rev, 0.50, 1.00, encType, park, UgField.RIPA);
        }

        return points;
    }

    public UgRoute(PositionOption startPos,
                   Field.Alliance alliance)
    {
        super(startPos, alliance);
    }
//Alex was here
    private boolean goForTwo = false;

    public void setGoForTwo(boolean goForTwo)
    {
        this.goForTwo = goForTwo;
    }

    public Point2d convertRtoB(Point2d rpt)
    {
        double bx =  rpt.getX();
        double by = -rpt.getY();

        String nm = "B" + rpt.getName().substring(1);

        RobotLog.dd(TAG, "convRtoB %s to %s %4.1f %4.1f", rpt.getName(), nm,
                bx, by);

        return new Point2d(nm, bx, by);
    }
}