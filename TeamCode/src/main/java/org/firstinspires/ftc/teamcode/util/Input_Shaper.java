package org.firstinspires.ftc.teamcode.util;

public class Input_Shaper
{
    private boolean inputActive;
    private boolean inputSlow;

    public double shape(double inValue)
    {
        return  inValue * Math.abs(inValue);
        //return Math.pow(inValue, 3);
    }

    public double shape(double inValue, double minVal)
    {
        double deadVal = 0.01;
        double inAbs = Math.abs(inValue);
        double outVal = 0.0;
        if(inAbs <= deadVal)
        {
            outVal = 0.0;
        }
        else if(inAbs < minVal)
        {
            outVal = Math.signum(inValue) * minVal;
        }
        else if(inAbs <= 0.95)
        {
            double shftVal = inAbs - minVal;
            outVal = Math.signum(inValue) * (shftVal * Math.abs(shftVal) + minVal);
        }
        else
        {
            outVal = Math.signum(inValue) * 1.0;
        }
        return outVal;
    }
}
