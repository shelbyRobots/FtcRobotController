package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class RateChangeLimiter
{
  private final double m_rateLimit;
  private double m_prevVal;
  private double m_prevTime;
  private final ElapsedTime tmr = new ElapsedTime();

  public RateChangeLimiter(double rateLimit, double initialValue)
  {
    m_rateLimit = rateLimit;
    m_prevVal = initialValue;
    m_prevTime = tmr.seconds();
  }

  public RateChangeLimiter(double rateLimit)
  {
    this(rateLimit, 0);
  }

  public double calculate(double input)
  {
    double currentTime = tmr.seconds();
    double elapsedTime = currentTime - m_prevTime;
    m_prevVal +=
        Range.clip(input - m_prevVal,-m_rateLimit * elapsedTime,m_rateLimit * elapsedTime);
    m_prevTime = currentTime;
    return m_prevVal;
  }

  public void reset(double value)
  {
    m_prevVal = value;
    m_prevTime = tmr.seconds();
  }
}
