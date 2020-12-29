package org.firstinspires.ftc.teamcode.util;

public class Pose2d
{
    public Pose2d()
    {
        this(0, 0, 0);
    }

    public Pose2d(double x, double y, double h)
    {
        pt = new Point2d(x, y);
        hdg = h;
    }

    public Pose2d(double x, double y)   { this(x, y,0);  }
    public Pose2d(Point2d pt, double h)
    {
        this(pt.getX(), pt.getY(), h);
    }
    public Pose2d(Point2d pt)           { this(pt, 0); }

    public Point2d getPt() { return pt; }
    public double getHdg() { return hdg; }

    public Pose2d plus(Pose2d other)
    {
        return new Pose2d(pt.getX() + other.getPt().getX(),
                          pt.getY() + other.getPt().getX(),
                          hdg + other.getHdg());
    }

    public Pose2d minus(Pose2d other)
    {
        return new Pose2d(pt.getX() - other.getPt().getX(),
                          pt.getY() - other.getPt().getX(),
                          hdg - other.getHdg());
    }

    public Pose2d neg()
    {
        return new Pose2d(-pt.getX(), -pt.getY(), -hdg);
    }

//    public Point2d fromPolar(double r, double ang)
//    {
//        return new Point2d(r * Math.cos(ang), r *Math.sin(ang));
//    }

    private Point2d pt;
    private double hdg;
}
