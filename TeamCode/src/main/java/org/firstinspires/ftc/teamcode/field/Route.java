package org.firstinspires.ftc.teamcode.field;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Segment;

import java.util.Vector;


@SuppressWarnings("unused")
public abstract class Route
{
    private static final String TAG = "SJH_PNT";

    protected Vector<Point2d> initPoints()
    {
        return new Vector<>(MAX_SEGMENTS);
    }

    Route(PositionOption startPos,
          Field.Alliance alliance)
    {
        this.startPos     = startPos;
        this.alliance     = alliance;

        RobotLog.dd(TAG, "In base Route ctor.  Alliance=%s", alliance);

        Vector<Point2d> pts = initPoints();
        Vector<Point2d> points;
        if(alliance == Field.Alliance.RED)
        {
            RobotLog.dd(TAG, "InitRedpoints");
            points = initRedPoints(pts);
        }
        else
        {
            RobotLog.dd(TAG, "InitBluepoints");
            points = initBluePoints(pts);
        }

        segments  = initSegments(points);
    }

    void addPoint(Vector<Point2d> points,
                  ShelbyBot.DriveDir dir,
                  double speed,
                  @SuppressWarnings("SameParameterValue") double tune,
                  Segment.TargetType targetType,
                  Segment.Action action,
                  Point2d pt)
    {
        segDirs.add(dir);
        segSpeeds.add(speed);
        ttypes.add(targetType);
        actions.add(action);
        tuners.add(tune);
        points.add(pt);
    }

    public final Segment[] getSegments()
    {
        return segments;
    }

    private Vector<Point2d> initRedPoints(Vector<Point2d> inpts)
    {
        Vector<Point2d> rpts = new Vector<>(inpts.size());

        rpts.addAll(inpts);
        return rpts;
    }

    private Vector<Point2d> initBluePoints(Vector<Point2d> inpts)
    {
        Vector<Point2d> bpts = new Vector<>(inpts.size());

        for(Point2d rpt : inpts)
        {
            bpts.add(convertRtoB(rpt));
        }
        return bpts;
    }

    private Segment[] initSegments(Vector<Point2d> pts)
    {
        int numSegs = pts.size() - 1;
        Segment[] pathSegs = new Segment[numSegs];
        Segment seg;
        for(int s = 0; s < numSegs; ++s)
        {
            String sname = pts.get(s+1).getName();

            seg = new Segment(sname, pts.get(s), pts.get(s+1));
            seg.setDir(segDirs.get(s));
            seg.setSpeed(segSpeeds.get(s));
            seg.setAction(actions.get(s));
            seg.setTgtType(ttypes.get(s));
            seg.setDrvTuner(tuners.get(s));

            RobotLog.ii(TAG, "setting up segment %s %s %s %4.1f tune: %4.2f",
                    seg.getName(), seg.getStrtPt(), seg.getTgtPt(),
                    seg.getFieldHeading(), seg.getDrvTuner());

            pathSegs[s] = seg;
        }
        return pathSegs;
    }

    @SuppressWarnings("unused")
    private Segment getSegment(String name, Segment[] segs)
    {
        for (Segment pathSeg : segs)
        {
            String n = pathSeg.getName();
            if (n.equals(name)) return pathSeg;
        }
        return null;
    }

    protected Point2d convertRtoB(Point2d rpt)
    {
        double bx = -rpt.getY();
        double by = -rpt.getX();
        rpt.setX(bx);
        rpt.setY(by);
        return rpt;
    }

    public String toString()
    {
        StringBuilder sbldr = new StringBuilder();
        sbldr.append("Segments\n");
        for (Segment segment : segments)
        {
            sbldr.append(segment.toString()).append("\n");
        }
        return sbldr.toString();
    }

    static final double S_WALL = Field.S_WALL_Y;
    static final double W_WALL = Field.W_WALL_X;

    final static int    MAX_SEGMENTS = 16;

    private Segment[] segments;

    private Vector<Segment.Action> actions = new Vector<>(MAX_SEGMENTS);
    private Vector<Double> segSpeeds = new Vector<>(MAX_SEGMENTS);
    private Vector<ShelbyBot.DriveDir> segDirs = new Vector<>(MAX_SEGMENTS);
    private Vector<Double> tuners = new Vector<>(MAX_SEGMENTS);
    private Vector<Segment.TargetType> ttypes = new Vector<>(MAX_SEGMENTS);

    PositionOption     startPos;
    PositionOption     parkChoice;
    Field.Alliance     alliance;

    public enum StartPos implements PositionOption
    {
        START_1,
        START_2,
        START_3,
        START_4,
        START_5,
        START_6
    }

    public enum ParkPos implements PositionOption
    {
        CENTER_PARK,
        CORNER_PARK,
        DEFEND_PARK
    }
}


