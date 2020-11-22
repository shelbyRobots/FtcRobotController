package org.firstinspires.ftc.teamcode.util;

import java.util.Locale;

public class Point2d
{
    public Point2d(String name, double x, double y)
    {
        this.name = name;
        this.x = x;
        this.y = y;
    }

    public Point2d(double x, double y)
    {
        this("PT" + numpts++, x, y);
    }

    public Point2d(float[] fltarr) {this((double)fltarr[0], (double)fltarr[1]);}

    public Point2d(String name, float[] fltarr)
    {
        this(name, (double)fltarr[0], (double)fltarr[1]);
    }

    public Point2d(String name, Point2d inPt)
    {
        this(name, inPt.x, inPt.y);
    }

    public double distance(Point2d tgtPt)
    {
        double sq_dist = (tgtPt.x - x)*(tgtPt.x -x) + (tgtPt.y - y)*(tgtPt.y - y);
        return Math.sqrt(sq_dist);
    }

    @SuppressWarnings("unused")
    public double angle(Point2d prvPt, Point2d nxtPt)
    {
        double seg1FldHdg = Math.atan2(y - prvPt.getY(), x - prvPt.getX());
        double seg2FldHdg = Math.atan2(nxtPt.getY() - y, (nxtPt.getX() - x));
        return Math.toDegrees(seg2FldHdg - seg1FldHdg);
    }

    //Note - the class is named point2d - but the following norm and angle are vectors ops
    //norm is the length of the vector from 0,0 to pt
    public double norm()
    {
        return Math.sqrt(x*x + y*y);
    }

    //ang is the angle of the vector from 0,0 to pt.  Do we need to "normalize on 0-360 or +/-180?
    public double ang()
    {
        return Math.atan2(y, x);
    }

    public String getName() { return name; }
    public void setName(String name) { this.name = name; }

    public double getX()
    {
        return x;
    }
    public double getY()
    {
        return y;
    }

    public void setX(double x) { this.x = x; }
    public void setY(double y) { this.y = y; }

    public String toString()
    {
        return String.format(Locale.US, "%s(%5.2f: %5.2f)",
                name, x, y);
    }

    private String name;
    private double x;
    private double y;
    private static int numpts = 0;
}
