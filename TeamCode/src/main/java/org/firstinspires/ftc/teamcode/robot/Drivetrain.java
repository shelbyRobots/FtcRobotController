package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.Point2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@SuppressWarnings({"unused", "WeakerAccess", "FieldCanBeLocal", "UnusedAssignment", "SpellCheckingInspection", "UnnecessaryLocalVariable"})
public class Drivetrain
{
    public Drivetrain()
    {
        rt.reset();
    }

    public void moveInit(double lPwr, double rPwr)
    {
        resetLastPos();
        move(lPwr, rPwr);
        logData(true, "INIT PWR SET");
    }

    private double lastSetLpower = 0.0;
    private double lastSetRpower = 0.0;
    public void move(double lPwr, double rPwr)
    {
        lPwr = Math.round(lPwr*100)/100.0;
        rPwr = Math.round(rPwr*100)/100.0;

        boolean setL = true;
        boolean setR = true;
        if(lPwr == lastSetLpower) {setL = false;}
        if(rPwr == lastSetRpower) {setR = false;}

        curLpower = lPwr;
        curRpower = rPwr;

        if(setL || setR) RobotLog.dd(TAG, "MOVE: " +
                                          " lpwr " + lPwr + " rpwr " + rPwr );

        if(lFirst)
        {
            if(setL)
            {
                setPower(robot.leftMotors, curLpower);
                lastSetLpower = lPwr;
            }
            if(setR)
            {
                setPower(robot.rightMotors, curRpower);
                lastSetRpower = rPwr;
            }
        }
        else
        {
            if(setR)
            {
                setPower(robot.rightMotors, curRpower);
                lastSetRpower = rPwr;
            }
            if(setL)
            {
                setPower(robot.leftMotors, curLpower);
                lastSetLpower = lPwr;
            }
        }
        if(setL || setR) RobotLog.dd(TAG, "Powers set");
    }

    public void move(double pwr)
    {
        move(pwr, pwr);
    }

    public void stopMotion()
    {
        move(0.0, 0.0);
        logData(true, "MOTORS STOPPED");
    }

    private void resetCounts()
    {
        setMode(robot.leftMotors, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(robot.rightMotors, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        op.idle();
        setMode(robot.leftMotors, DcMotor.RunMode.RUN_USING_ENCODER);
        setMode(robot.rightMotors, DcMotor.RunMode.RUN_USING_ENCODER);
        for(int l = 0; l < robot.leftMotors.size(); l++)
        {
            begLpositions.set(l, 0);
            lstLpositions.set(l, 0);
        }
        for(int r = 0; r < robot.rightMotors.size(); r++)
        {
            begRpositions.set(r, 0);
            lstRpositions.set(r, 0);
        }
    }

    public void stopAndReset()
    {
        move(0.0, 0.0);
        logData(true, "MOTORS STOPPED - RESETTING");
    }

    public void resetLastPos()
    {
        accelTimer.reset();
        noMoveTimer.reset();
        setPos(lstLpositions, robot.leftMotors);
        setPos(lstRpositions, robot.rightMotors);
    }

    public void driveToTarget(double pwr, int thresh)
    {
        setBusyAnd(false);
        setMode(robot.leftMotors, DcMotor.RunMode.RUN_TO_POSITION);
        setMode(robot.rightMotors, DcMotor.RunMode.RUN_TO_POSITION);
        setTargetPositions(robot.leftMotors, tgtLpositions);
        setTargetPositions(robot.rightMotors, tgtRpositions);
        setInitValues();
        logStartValues("DRIVE_TRGT " + curLpositions.get(0) + " " + curRpositions.get(0)+
                               " - " + tgtLpositions.get(0) + " " + tgtRpositions.get(0));
        moveInit(pwr, pwr);
        while(isBusy(thresh)         &&
              !op.isStopRequested()  &&
              !areDriveMotorsStuck())
        {
            setCurValues();
            logData();

            if(tickRate > 0) waitForTick(tickRate);
            frame++;
        }

        stopMotion();
        setEndValues("DRIVE_TRGT");
    }

    public void driveToColor(double pwr, int thresh )
    {
        driveDistance(15, pwr, Direction.FORWARD);

        int totalcolor=0;
        while(op.opModeIsActive()    &&
                !op.isStopRequested()  &&
                totalcolor < thresh)
        {
            setCurValues();
            totalcolor = curRed + curGrn + curBlu;
        }
        stopMotion();
    }

    public void driveDistance(double dst, double pwr, Direction dir)
    {
        int counts = distanceToCounts(dst);

        RobotLog.ii(TAG, "driveDistance: %6.2f Counts %d", dst, counts);

        if(dir == Direction.REVERSE)
        {
            counts*=-1;
        }

        setInitValues();
        setPositions(tgtLpositions, begLpositions, counts);
        setPositions(tgtRpositions, begRpositions, counts);
        String dDistStr = String.format(Locale.US, "DRIVE_DIST %4.2f", dst);
        logStartValues(dDistStr);

        setTargetPositions(robot.leftMotors, tgtLpositions);
        setTargetPositions(robot.rightMotors, tgtRpositions);

        setMode(robot.leftMotors, DcMotor.RunMode.RUN_TO_POSITION);
        setMode(robot.rightMotors, DcMotor.RunMode.RUN_TO_POSITION);

        moveInit(pwr, pwr);
    }

    public int driveDistanceLinear(double dst, double pwr, Direction dir,
                                   double targetHdg, boolean useCol)
    {
        trgtHdg = targetHdg;

        RobotLog.ii(TAG, "Starting drive %s", useCol);
        if(doStopAndReset) stopAndReset();
        logData(true, "LINDST");

        double startPwr = 0.1;
        boolean foundLine = false;

        initLpower = startPwr;
        initRpower = startPwr;
        int pwrSteps = 5;
        double pwrIncr = (pwr - startPwr)/pwrSteps;

        //extend distance if doing a color find to make sure we reach line
        int colOverCnt = 0;
        if(useCol)
        {
            double colOverDist = 1.0;
            colOverCnt = distanceToCounts(colOverDist);
            dst += colOverDist;
        }

        RobotLog.dd(TAG, "DriveDistanceLinear %4.2f %4.2f %4.2f %4.2f %s %s %s %s",
                dst, pwr, pwrIncr, targetHdg, dir, rampUp, rampDown, useCol);

        driveDistance(dst, startPwr, dir);
        int linLpos = tgtLpositions.get(0);
        int linRpos = tgtRpositions.get(0);

        while(op.opModeIsActive()    &&
              !op.isStopRequested()  &&
              isBusy()               &&
              !areDriveMotorsStuck())
        {
            setCurValues();
            logData();

            double ppwr = pwr;
            double tmpPwr = (curLpower + curRpower)/2;

            if(rampUp)
            {
                if(tmpPwr < pwr)
                {
                    tmpPwr = Math.min(pwr, tmpPwr + pwrIncr);
                    RobotLog.dd(TAG, "Ramping up %4.2f", tmpPwr);
                    ppwr = tmpPwr;
                }
            }

            if(rampDown)
            {
                int lcnt = curLpositions.get(0);
                int rcnt = curRpositions.get(0);
                int hiSlow  = rampCntH;
                int midSlow = rampCntM + colOverCnt/4;
                int lowSlow = rampCntL + colOverCnt;
                int remaining = (Math.abs(tgtLpositions.get(0) - lcnt)
                               + Math.abs(tgtRpositions.get(0) - rcnt)) / 2;
                if (Math.abs(remaining) < hiSlow)  ppwr = Math.min(ppwr, rampSpdH);
                if (Math.abs(remaining) < midSlow) ppwr = Math.min(ppwr, rampSpdM);
                if (Math.abs(remaining) < lowSlow) ppwr = Math.min(ppwr, rampSpdL);
            }

            RobotLog.ii(TAG, "ppwr %4.2f curLpower %4.2f curRpower %4.2f pwrIncr %5.3f",
                    ppwr, curLpower, curRpower, pwrIncr);
            RobotLog.ii(TAG, "curLpos0 %d curRpos0 %d tgtL0 %d tgtR0 %d",
                    curLpositions.get(0), curRpositions.get(0),
                    tgtLpositions.get(0), tgtRpositions.get(0));

            double lRem = countsToDistance(Math.abs(tgtLpositions.get(0) -
                                                            curLpositions.get(0)));
            double rRem = countsToDistance(Math.abs(tgtRpositions.get(0) -
                                                            curRpositions.get(0)));
            double colOnDist = 24.0;
            if(useCol && !robot.colorEnabled && (lRem < colOnDist || rRem < colOnDist))
            {
                robot.turnColorOn();
            }

            if(useCol && (curRed > COLOR_THRESH || curBlu > COLOR_THRESH))
            {
                stopMotion();
                setEndValues("COLOR_FIND " + linLpos + " " + linRpos);
                RobotLog.ii(TAG, "FOUND LINE %d %d %d", curRed, curGrn, curBlu);
                foundLine = true;
                //RobotLog.ii(TAG, "colseg tgtLRpos: %d %d",
                //        tgtLpositions.get(0), tgtRpositions.get(0));
                //RobotLog.ii(TAG, "colseg curLRpos: %d %d",
                //        curLpositions.get(0), curRpositions.get(0));
                //RobotLog.ii(TAG, "colseg colSensOff: %d", colSensOffset);
                setPositions(tgtLpositions, curLpositions, -colSensOffset);
                setPositions(tgtRpositions, curRpositions, -colSensOffset);
                //RobotLog.ii(TAG, "TRYING TO DRIVE COLOFFSET");
                //driveDistanceLinear(colSensOffset/CPI, pwr, dir, targetHdg, false);
                break;
            }
            else
            {
                if((robot.gyro == null && robot.imu == null))
                {
                    move(ppwr, ppwr);
                }
                else
                {
                    makeGyroCorrections(ppwr, trgtHdg, dir);
                }
            }

            if(!isBusy()) break;

            if(tickRate > 0) waitForTick(tickRate);
            frame++;
        }

        boolean useOverKludge = false;
        int kludge = 160;
        //noinspection ConstantConditions
        if(useCol && useOverKludge && !foundLine)
        {
            int overK = colOverCnt + kludge;
            setPositions(tgtLpositions, tgtLpositions, -overK);
            setPositions(tgtRpositions, tgtRpositions, -overK);
        }

        if(useCol) robot.turnColorOff();

        setEndValues("LINDST");
        stopMotion();
        if(logOverrun) logOverrun(overtime);

        int lBegTotal = 0;
        int rBegTotal = 0;
        int lEndTotal = 0;
        int rEndTotal = 0;

        for(int pos : begLpositions) lBegTotal += pos;
        for(int pos : begRpositions) rBegTotal += pos;
        for(int pos : endLpositions) lEndTotal += pos;
        for(int pos : endRpositions) rEndTotal += pos;

        return((lEndTotal - lBegTotal + rEndTotal - rBegTotal)/
                       (begLpositions.size() + begRpositions.size()));
    }

    public int driveDistanceLinear(double dst, double pwr, Direction dir, double targetHdg)
    {
        return driveDistanceLinear(dst, pwr, dir, targetHdg, false);
    }

    @SuppressWarnings("UnusedReturnValue")
    public int driveDistanceLinear(double dst, double pwr, Direction dir)
    {
        return driveDistanceLinear(dst, pwr, dir, robot.getGyroFhdg());
    }

    public int driveToPoint(Point2d tgtPt, double pwr, Direction dir, double targetHdg)
    {
        if (tgtPt == null)
        {
            RobotLog.ee(TAG, "tgtPt null in driveToPoint");
            return 0;
        }
        if (currPt == null)
        {
            RobotLog.ee(TAG, "currPt null in driveToPoint");
            return 0;
        }
        double dist = currPt.distance(tgtPt);
        return driveDistanceLinear(dist, pwr, dir, targetHdg);
    }

    public int driveToPointLinear(Point2d tgtPt, double pwr, Direction dir, double targetHdg)
    {
        int dist = driveToPoint(tgtPt, pwr, dir, targetHdg);

        setCurrPt(tgtPt);
        return dist;
    }

    @SuppressWarnings("UnusedReturnValue")
    public int driveToPointLinear(Point2d tgtPt, double pwr, Direction dir)
    {
        return driveToPointLinear(tgtPt, pwr, dir, robot.getGyroFhdg());
    }

    public void ctrTurnEncoder(double angle, double pwr)
    {
        //perform a turn about drive axle center
        //left turns are positive angles

        int counts = angleToCounts(angle, robot.BOT_WIDTH/2.0);

        setInitValues();
        setPositions(tgtLpositions, begLpositions, -counts);
        setPositions(tgtRpositions, begRpositions,  counts);
        trgtHdg  = initHdg  + (int) Math.round(angle);
        while(trgtHdg >   180) trgtHdg -= 360;
        while(trgtHdg <= -180) trgtHdg += 360;
        String angStr = String.format(Locale.US, "ENC_TURN %4.2f", angle);
        logStartValues(angStr);

        RobotLog.ii(TAG, "Angle: %5.2f Counts: %4d CHdg: %6.3f", angle, counts, initHdg);

        setTargetPositions(robot.leftMotors, tgtLpositions);
        setTargetPositions(robot.rightMotors, tgtRpositions);

        setMode(robot.leftMotors, DcMotor.RunMode.RUN_TO_POSITION);
        setMode(robot.rightMotors, DcMotor.RunMode.RUN_TO_POSITION);

        moveInit(pwr, pwr);
    }

    public boolean ctrTurnGyro(double hdg, double pwr)
    {
        double   error;
        double   steer;
        boolean  onTarget = false;
        double leftSpeed;
        double rightSpeed;

        double ihdg = Math.round(hdg);

        // determine turn power based on +/- error
        error = getGyroError(hdg);

        if(error * lastGyroError < 0) //error sign changed - we passed target
        {
            RobotLog.ii(TAG, "ctrTurnGryo overshot lastErr %5.2f error %5.2f " +
                               "hdg %6.3f tgt %6.3f",
                    lastGyroError, error, curHdg, ihdg);
        }

        if (Math.abs(error) <= TURN_TOLERANCE)
        {
            if(!gyroFirstGood)
            {
                gyroFirstGood = true;
                gyroGoodCount = 0;
                gyroGoodTimer.reset();
            }
            gyroGoodCount++;
            logData(true, "GYRO GOOD " + gyroGoodCount + " TIME: " + gyroFrameTime.milliseconds());
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = gyroGoodTimer.milliseconds() > gyroTimeout;
        }
        else
        {
            gyroGoodCount = 0;
            gyroFirstGood = false;
            gyroGoodTimer.reset();
            double deltaErr = (error - lastGyroError)/gyroFrameTime.seconds();
            double d = Kd_GyroTurn * deltaErr;
            gyroFrameTime.reset();
            steer = getTurnSteer(error);
            rightSpeed  = pwr * steer;
            if(useDterm)
            {
                rightSpeed += d;
            }
            rightSpeed = Range.clip(rightSpeed, -1, 1);
            if(Math.abs(rightSpeed) < minGyroTurnSpeed)
            {
                rightSpeed = Math.signum(rightSpeed) * minGyroTurnSpeed;
            }
            leftSpeed   = -rightSpeed;
        }

        move(leftSpeed, rightSpeed);
        lastGyroError = error;

        double ct = ptmr.seconds();
        if(ct > nextPrintTime)
        {
            nextPrintTime = ct + printTimeout;
            RobotLog.ii(TAG, "TGT %6.3f CHDG %6.3f ERR %6.3f STR %4.2f L %4.2f R %4.2f",
                    hdg, curHdg,
                    error, steer, leftSpeed, rightSpeed);
        }
        return onTarget;
    }

    public void ctrTurnLinear(double angle, double pwr, int thresh)
    {
        RobotLog.ii(TAG, "Starting turn of %f", angle);

        if(doStopAndReset) stopAndReset();
        logData(true, "ENC TURN");

        ElapsedTime tTimer = new ElapsedTime();

        double startPwr = 0.1;
        initLpower = startPwr;
        initRpower = startPwr;

        double pwrSteps = 5.0;
        double pwrLIncr = (pwr - initLpower)/pwrSteps;
        double pwrRIncr = (pwr - initRpower)/pwrSteps;

        ctrTurnEncoder(angle, startPwr);

        while(op.opModeIsActive() &&
              !op.isStopRequested() &&
              isBusy(thresh) &&
              !areTurnMotorsStuck() &&
              tTimer.seconds() < turnTimeLimit)
        {
            setCurValues();
            logData();

            double ppwr = pwr;

            double tmpPwr = (curLpower + curRpower)/2;

            if(rampUp)
            {
                if(tmpPwr < pwr) tmpPwr = Math.min(pwr, tmpPwr + pwrLIncr);
                ppwr = tmpPwr;
            }

            if(rampDown)
            {
                int lcnt = curLpositions.get(0);
                int rcnt = curRpositions.get(0);
                int hiSlow  = rampCntH;
                int midSlow = rampCntM;
                int lowSlow = rampCntL;
                int remaining = (Math.abs(tgtLpositions.get(0) - lcnt)
                               + Math.abs(tgtRpositions.get(0) - rcnt)) / 2;
                if (Math.abs(remaining) < hiSlow)  ppwr = Math.min(ppwr, rampSpdH);
                if (Math.abs(remaining) < midSlow) ppwr = Math.min(ppwr, rampSpdM);
                if (Math.abs(remaining) < lowSlow) ppwr = Math.min(ppwr, rampSpdL);
            }
            double lpwr = ppwr;
            double rpwr = ppwr;

            if(stopIndividualMotorWhenNotBusy)
            {
                if(!isMotorBusy(MotorSide.LEFT))  lpwr = 0.0;
                if(!isMotorBusy(MotorSide.RIGHT)) rpwr = 0.0;
            }
            else if(!isBusy(TURN_BUSYTHRESH))
            {
                lpwr = 0.0;
                rpwr = 0.0;
            }

            move(lpwr, rpwr);

            if(tickRate > 0) waitForTick(tickRate);
            frame++;
        }
        setEndValues("ENC_TURN");
        stopMotion();
        if(logOverrun) logOverrun(overtime);
    }

    public void ctrTurnLinear(double angle, double pwr)
    {
        ctrTurnLinear(angle, pwr, TURN_BUSYTHRESH);
    }

    public void ctrTurnToHeading(double tgtHdg, double pwr)
    {
        tgtHdg = Math.round(tgtHdg);

        RobotLog.ii(TAG, "GYRO TURN to HDG %6.3f", tgtHdg);

        gyroFirstGood = false;

        if(doStopAndReset) stopAndReset();

        setMode(robot.leftMotors, DcMotor.RunMode.RUN_USING_ENCODER);
        setMode(robot.rightMotors, DcMotor.RunMode.RUN_USING_ENCODER);

        moveInit(0.0, 0.0);

        setInitValues();
        setPositions(tgtLpositions, 0);
        setPositions(tgtRpositions, 0);

        trgtHdg = (int) tgtHdg;
        String gyroStr = String.format(Locale.US, "GYRO_TURN %4.1f", tgtHdg);
        logStartValues(gyroStr);

        ElapsedTime tTimer = new ElapsedTime();

        RobotLog.ii(TAG, "Starting gyro turn");
        gyroFrameTime.reset();
        while(op.opModeIsActive()       &&
              !op.isStopRequested()     &&
              !ctrTurnGyro(tgtHdg, pwr) &&
              !areTurnMotorsStuck()         &&
              tTimer.seconds() < turnTimeLimit)
        {
            setCurValues();
            logData();

            if(tickRate > 0) waitForTick(tickRate);
            frame++;
        }
        setEndValues("GYRO_TURN");
        stopMotion();
    }

    public void turn(double angle, double pwr, double radius)
    {
        //radius is distance from ctr of bot to ctr of curve
        //it is positive toward the left side
        //A radius of 0 is a ctr turn
        //A radius of +w/2 pivots on the left wheel
        //A radius of -w/2 pivots on the right wheel

        double rl = radius - robot.BOT_WIDTH/2.0;
        double rr = radius + robot.BOT_WIDTH/2.0;
        int lcnts = angleToCounts(angle, rl);
        int rcnts = angleToCounts(angle, rr);

        setBusyAnd(false);
        setMode(robot.leftMotors, DcMotor.RunMode.RUN_TO_POSITION);
        setMode(robot.rightMotors, DcMotor.RunMode.RUN_TO_POSITION);

        setInitValues();

        setPositions(tgtLpositions, begLpositions, lcnts);
        setPositions(tgtRpositions, begRpositions, rcnts);
        trgtHdg  = initHdg  + (int) Math.round(angle);
        while(trgtHdg >   180) trgtHdg -= 360;
        while(trgtHdg <= -180) trgtHdg += 360;
        logStartValues("ENC_CURVE");

        setPos(tgtLpositions, robot.leftMotors);
        setPos(tgtRpositions, robot.rightMotors);

        double arl = Math.abs(rl);
        double arr = Math.abs(rr);
        double rad_ratio = Math.min(arl, arr) / Math.max(arl, arr);

        double opwr = pwr;
        double ipwr = opwr * rad_ratio;

        double lp = ipwr;
        double rp = opwr;

        if (arl > arr)
        {
            lp = opwr;
            rp = ipwr;
        }

        moveInit(lp, rp);

        while(op.opModeIsActive() &&
              isBusy(TURN_BUSYTHRESH) &&
              !areTurnMotorsStuck())
        {
            setCurValues();
            logData();

            if(tickRate > 0) waitForTick(tickRate);
            frame++;
        }
        move(0.0, 0.0);
    }

    public int distanceToCounts(double distance)
    {
        return (int)(distance * CPI);
    }

    public double countsToDistance(double counts)
    {
        return (counts / CPI);
    }

    public int angleToCounts(double angle, double radius)
    {
        return distanceToCounts(Math.toRadians(angle) * radius);
    }

    public void setColorThresh(int thresh)
    {
        COLOR_THRESH = thresh;
    }

    private double countsToAngle(int counts, double radius)
    {
        return (double)counts/(CPI*radius);
    }

    public void setCurrPt(Point2d curPt, boolean resetEstPt)
    {
        RobotLog.ii(TAG, "setCurrPt %s. estPos %s", curPt, estPos);
        currPt = curPt;
        if(numPts == 0 || resetEstPt)
        {
            xPos = currPt.getX();
            yPos = currPt.getY();
            estPos.setX(xPos);
            estPos.setY(yPos);
            estHdg = initHdg;
        }
        ++numPts;
    }

    public  void setCurrPt(Point2d curPt)
    {
        setCurrPt(curPt, false);
    }

    public Point2d getCurrPt() { return currPt; }
    public Point2d getEstPos() { return estPos; }

    public void estimatePosition()
    {
        int curLcnt = curLpositions.get(0);
        int curRcnt = curRpositions.get(0);
        double radHdg = Math.toRadians(curHdg);

        if(curDriveDir != robot.getDriveDir())
        {
            curDriveDir = robot.getDriveDir();
            setPositions(lstLpositions, curLpositions, 0);
            setPositions(lstRpositions, curRpositions, 0);
            logData(true, "DDIR=" + curDriveDir.toString());
        } 

        int dCntL = curLcnt - lstLpositions.get(0);
        int dCntR = curRcnt - lstRpositions.get(0);
        double dX = 0.5*(dCntL+dCntR)/DEF_CPI * Math.cos(radHdg);
        double dY = 0.5*(dCntL+dCntR)/DEF_CPI * Math.sin(radHdg);
        xPos += dX;
        yPos += dY;
        estPos.setX(xPos);
        estPos.setY(yPos);
        double dH = (dCntR-dCntL)/CPI/ robot.BOT_WIDTH;
        estHdg += dH;
        setPositions(lstLpositions, curLpositions, 0);
        setPositions(lstRpositions, curRpositions, 0);
    }

    public void setStartHdg(double startHdg)
    {
        this.startHdg = startHdg;
    }

    public void init(ShelbyBot robot)
    {
        com = CommonUtil.getInstance();
        op = com.getLinearOpMode();
        dl = com.getDataLogger();
        frame = 0;
        this.robot  = robot;

        CPI = robot.CPI;
        DEF_CPI = CPI;

        noMoveThreshLow = (int)CPI;
        noMoveThreshHi  = 4 * noMoveThreshLow;

        RobotLog.ii(TAG, "CPI: %5.2f", CPI);

        curDriveDir = robot.getDriveDir();
        lastDriveDir = curDriveDir;

        RobotLog.dd(TAG, "Drivetrain.init");
        RobotLog.dd(TAG, " numLmotors %d", robot.numLmotors);
        RobotLog.dd(TAG, " numRmotors %d", robot.numRmotors);

        datalogtimer.reset();

        initLPositions(begLpositions, 0);
        initRPositions(begRpositions, 0);
        initLPositions(curLpositions, 0);
        initRPositions(curRpositions, 0);
        initLPositions(tgtLpositions, 0);
        initRPositions(tgtRpositions, 0);
        initLPositions(endLpositions, 0);
        initRPositions(endRpositions, 0);
        initLPositions(lstLpositions, 0);
        initRPositions(lstRpositions, 0);
    }

    public void start()
    {
    }

    public void cleanup()
    {
        stopMotion();
        RobotLog.ii(TAG, "CLOSING Drivetrain");
    }

    public void waitForTick(long periodMs)
    {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            op.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public void makeGyroCorrections(double pwr, double thdg, Direction dir)
    {
        if((robot.gyro == null && robot.imu == null) || !robot.gyroReady) return;

        double ldp;
        double rdp;

        double err = getGyroError(thdg);

        double steer = getSteer(err, Kp_GyrCorrection);
        if (dir == Direction.REVERSE) steer *= -1;

        if     (pwr + steer >  1.0) steer =  1.0 - pwr;
        else if(pwr + steer < -1.0) steer = -1.0 - pwr;
        if     (pwr - steer >  1.0) steer = -1.0 + pwr;
        else if(pwr - steer < -1.0) steer =  1.0 + pwr;

        if     (pwr + steer <  minSpeed && pwr + steer > 0.0) steer =  minSpeed - pwr;
        else if(pwr + steer > -minSpeed && pwr + steer < 0.0) steer = -minSpeed - pwr;
        if     (pwr - steer <  minSpeed && pwr + steer > 0.0) steer = -minSpeed + pwr;
        else if(pwr - steer > -minSpeed && pwr + steer < 0.0) steer =  minSpeed + pwr;

        rdp = pwr + steer;
        ldp = pwr - steer;

        double max = Math.max(Math.abs(rdp), Math.abs(ldp));
        if (max > 1.0)
        {
            rdp = rdp / max;
            ldp = ldp / max;
        }

        if(stopIndividualMotorWhenNotBusy)
        {
            if(!isMotorBusy(MotorSide.LEFT))  ldp = 0.0;
            if(!isMotorBusy(MotorSide.RIGHT)) rdp = 0.0;
        }
        else if(!isBusy())
        {
            ldp = 0.0;
            rdp = 0.0;
        }

        move(ldp, rdp);

        double ct = ptmr.seconds();
        if (ct > nextPrintTime)
        {
            nextPrintTime = ct + printTimeout;
            RobotLog.ii(TAG, "%4d lpwr: %5.3f rpwr: %5.3f err: %5.3f" +
                               "str %5.3f rt %5.3f chdg %6.3f thdg %6.3f",
                    frame, curLpower, curRpower, err, steer, rt.seconds(), curHdg, thdg);
        }
    }

    private double getEncoderError()
    {
        int ldc = Math.abs(curLpositions.get(0));
        int rdc = Math.abs(curRpositions.get(0));

        //convert LR count difference to angle
        return countsToAngle(rdc - ldc, robot.BOT_WIDTH);
    }

    private double getGyroError(double tgtHdg)
    {
        double gHdg = curHdg;
        double robotError = tgtHdg - gHdg;
        while (robotError > 180)   robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    private double getSteer(double error, double PCoeff)
    {
        return Range.clip(error * PCoeff, -1, 1);
    }

    private double getTurnSteer(double error)
    {
        double sign = Math.signum(error);
        double absError = Math.abs(error);
        if      (absError >= maxStrAng) return sign * maxStr;
        else if (absError <= minStrAng) return sign * minStr;

        double str =  strSlope * absError + strB;
        str = Range.clip(str, minStr, maxStr) * sign;
        return str;
    }

    public void setInitValues()
    {
        setPos(begLpositions, robot.leftMotors);
        setPos(begRpositions, robot.rightMotors);
        initHdg = robot.getGyroFhdg();
        while (initHdg <= -180) initHdg += 360;
        while (initHdg >   180) initHdg -= 360;
        setPositions(curLpositions, begLpositions, 0);
        setPositions(curRpositions, begRpositions, 0);
        curHdg  = initHdg;
    }

    public void setCurValues()
    {
        if(robot.leftMotors.size()  > 0) setPos(curLpositions, robot.leftMotors);
        if(robot.rightMotors.size() > 0) setPos(curRpositions, robot.rightMotors);
        boolean printVals = false;

        if (datalogtimer.seconds() > 0.1)
        {
            printVals = true;
            datalogtimer.reset();
        }

        if(printVals)
        {
            StringBuilder sb = new StringBuilder(16);
            for (int i = 0; i < curLpositions.size(); i++)
            {
                sb.append(curLpositions.get(i));
                if (i < curLpositions.size() - 1) sb.append(" ");
            }
            RobotLog.dd(TAG, "curLpos: %s", sb.toString());

            //noinspection ConstantConditions
            if (sb != null) sb.delete(0, sb.length());
            for (int i = 0; i < curRpositions.size(); i++)
            {
                sb.append(curRpositions.get(i));
                if (i < curRpositions.size() - 1) sb.append(" ");
            }
            RobotLog.dd(TAG, "curRpos: %s", sb.toString());
        }

        curHdg  = robot.getGyroFhdg();
        if(robot.colorEnabled)
        {
            curRed = robot.colorSensor.red();
            curGrn = robot.colorSensor.green();
            curBlu = robot.colorSensor.blue();
        }
        else
        {
            curRed = 0;
            curGrn = 0;
            curBlu = 0;
        }
        estimatePosition();
    }

    public void logStartValues(String note)
    {
        if(logData && dl != null)
        {
            dl.addField("BEGIN " + note);
            dl.addField(frame);
            dl.addField(initHdg);
            dl.addField(begLpositions.get(0));
            dl.addField(begRpositions.get(0));
            dl.addField(trgtHdg);
            dl.addField(tgtLpositions.get(0));
            dl.addField(tgtRpositions.get(0));
            dl.addField("");
            dl.newLine();
        }
        RobotLog.ii(TAG, "Begin ldc %6d rdc %6d Hdg %6.3f",
                begLpositions.get(0), begRpositions.get(0), initHdg);
    }

    public void setEndValues(String note)
    {
        setPos(endLpositions, robot.leftMotors);
        setPos(endRpositions, robot.rightMotors);
        doneHdg  = robot.getGyroFhdg();
        logData(true, "DONE " + note);

        RobotLog.ii(TAG, "End ldc %6d rdc " +
                                   "%6d Hdg %6.3f",
                endLpositions.get(0), endRpositions.get(0), doneHdg);
    }

    public void logOverrun(double t)
    {
        setCurValues();
        logData(true, "LOGGING OVERRUN");
        int overCnt = 0;
        int goodCnt = 0;
        int overLpos = curLpositions.get(0);
        int overRpos = curRpositions.get(0);
        int overThresh = 3;
        ElapsedTime et = new ElapsedTime();
        while(op.opModeIsActive() && et.seconds() < t)
        {
            setCurValues();
            logData();

            if(Math.abs(curLpositions.get(0) - overLpos) <= overThresh &&
               Math.abs(curRpositions.get(0) - overRpos) <= overThresh)
            {
               goodCnt++;
            }
            else
            {
               goodCnt = 0;
               overLpos = curLpositions.get(0);
               overRpos = curRpositions.get(0);
            }

            if(overCnt >= 4 && goodCnt >=3)
            {
               break;
            }

            if(tickRate > 0) waitForTick(tickRate);
            frame++;
            overCnt++;
        }
        logData(true, "ENDOVER");
    }

    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    private boolean areTurnMotorsStuck()
    {
        if(usePosStop)
        {
            double lp = Math.abs(curLpower);
            double rp = Math.abs(curRpower);

            if(accelTimer.seconds() < 0.5)
            {
                setPositions(lstLpositions, curLpositions, 0);
                setPositions(lstRpositions, curRpositions, 0);
                noMoveTimer.reset();
                return false;
            }

            int dLpos = Math.abs(lstLpositions.get(0) - curLpositions.get(0));
            int dRpos = Math.abs(lstRpositions.get(0) - curRpositions.get(0));

            //If power is above threshold and encoders aren't changing,
            //stop after noMoveTimeout
            if(noMoveTimer.seconds() > noMoveTimeout)
            {
                if ((lp >= minSpeed && dLpos < noMoveThreshLow) &&
                    (rp >= minSpeed && dRpos < noMoveThreshLow))
                {
                    RobotLog.ii(TAG,
                            "TURN MOTORS HAVE POWER BUT AREN'T MOVING " +
                                    " - STOPPING LPWR:%4.2f RPWR:%4.2f " +
                                    " dLpos:%d dRpos:%d thresh:%d TO:%.2f",
                            lp, rp, dLpos, dRpos, noMoveThreshLow, noMoveTimeout);
                    return true;
                }
                setPositions(lstLpositions, curLpositions, 0);
                setPositions(lstRpositions, curRpositions, 0);
                noMoveTimer.reset();
            }
        }
        return false;
    }

    public boolean areDriveMotorsStuck()
    {
        if(usePosStop)
        {
            double lp = Math.abs(curLpower);
            double rp = Math.abs(curRpower);

            if(accelTimer.seconds() < 0.5)
            {
                setPositions(lstLpositions, curLpositions, 0);
                setPositions(lstRpositions, curRpositions, 0);
                noMoveTimer.reset();
                return false;
            }

            int dLpos = Math.abs(lstLpositions.get(0) - curLpositions.get(0));
            int dRpos = Math.abs(lstRpositions.get(0) - curRpositions.get(0));

            //If power is above threshold and encoders aren't changing,
            //stop after noMoveTimeout
            if(noMoveTimer.seconds() > noDriveMoveTimeout)
            {
                if ((lp >= minSpeed && dLpos < noMoveThreshLow) &&
                    (rp >= minSpeed && dRpos < noMoveThreshLow))
                {
                    RobotLog.ii(TAG,
                            "DRIVE MOTORS HAVE POWER BUT AREN'T MOVING " +
                            " - STOPPING LPWR:%4.2f RPWR:%4.2f " +
                            " dLpos:%d dRpos:%d thresh:%d TO:%.2f",
                            lp, rp, dLpos, dRpos, noMoveThreshLow, noDriveMoveTimeout);
                    return true;
                }
                if ((lp >= noMovePwrHi && dLpos < noMoveThreshHi) ||
                    (rp >= noMovePwrHi && dRpos < noMoveThreshHi))
                {
                    RobotLog.ii(TAG, "DRIVE MOTORS HAVE HI POWER BUT AREN'T MOVING - STOPPING %4.2f %4.2f",
                            lp, rp);
                    return true;
                }

                setPositions(lstLpositions, curLpositions, 0);
                setPositions(lstRpositions, curRpositions, 0);
                noMoveTimer.reset();
            }
        }
        return false;
    }

    private boolean isMotorBusy(MotorSide side)
    {
        double btime = busyTimer.milliseconds();
        boolean motorBusy;
        if(side == MotorSide.LEFT)
        {
            motorBusy = Math.abs(tgtLpositions.get(0) - curLpositions.get(0)) > BUSYTHRESH;
            if(motorBusy) lBusyTime = btime + busyTimeOut;
            else if(btime < lBusyTime) motorBusy = true;
        }
        else
        {
            motorBusy = Math.abs(tgtRpositions.get(0) - curRpositions.get(0)) > BUSYTHRESH;
            if(motorBusy) rBusyTime = btime + busyTimeOut;
            else if(btime < rBusyTime) motorBusy = true;
        }

        return motorBusy;
    }

    private boolean isBusy(int thresh)
    {
        double ct = ptmr.seconds();
        if(ct > nextBusyPrintTime)
        {
            nextBusyPrintTime = ct + printTimeout;
            RobotLog.ii(TAG, "ldc %6d rdc %6d  mptimer: %4.2f chdg %6.3f",
                    curLpositions.get(0), curRpositions.get(0),
                    noMoveTimer.seconds(), curHdg);
        }

        BUSYTHRESH = thresh;

        boolean lBusy = isMotorBusy(MotorSide.LEFT);
        boolean rBusy = isMotorBusy(MotorSide.RIGHT);
        boolean busy = false;

        if(busyAnd)
        {
            busy = lBusy && rBusy; //true if both are busy
        }
        else
        {
            busy = lBusy || rBusy; //true if 1 is busy
        }
        return busy;
    }

    public boolean isBusy()
    {
       return isBusy(DEF_BUSYTHRESH);
    }

    public void setBusyAnd(boolean busyAnd)
    {
        this.busyAnd = busyAnd;
    }

    public void setRampUp(boolean rampUp)
    {
        this.rampUp = rampUp;
    }

    public boolean getRampDown() {return  rampDown;}
    public void setRampDown(boolean rampDown)
    {
        this.rampDown = rampDown;
    }

    public void setStopIndividualMotorWhenNotBusy(boolean stopIndvid)
    {
        this.stopIndividualMotorWhenNotBusy = stopIndvid;
    }

    public void setColSensOffset(int offset)
    {
        this.colSensOffset = offset;
    }

    public void setLFirst(boolean lFirst)
    {
        this.lFirst = lFirst;
    }

    public void setDrvTuner(double dtnr)
    {
        CPI = robot.CPI / dtnr;
    }

    public void setLogOverrun(boolean lo)
    {
        this.logOverrun = lo;
    }

    public void logData(boolean force, String title)
    {
        double ldt = logDataTimer.time();
        boolean expired = ldt > logTime;
        if(logData && dl != null && (expired || force))
        {
            if(expired) logTime = ldt + logDataTimeout;
            if(force) logTime = 0;
            String comment = "";
            if(title != null) comment = title;
            dl.addField(comment);
            dl.addField(frame);
            if(robot.imu != null || robot.gyro != null) dl.addField(curHdg);
            else                   dl.addField("");
            dl.addField(curLpositions.size() > 0 ? curLpositions.get(0) : -9999);
            dl.addField(curRpositions.size() > 0 ? curRpositions.get(0) : -9999);
            dl.addField(curLpower);
            dl.addField(curRpower);
            dl.addField(curRed);
            dl.addField(curGrn);
            dl.addField(curBlu);
            dl.addField(estPos.getX());
            dl.addField(estPos.getY());
            dl.addField(Math.toDegrees(estHdg));
            dl.newLine();
        }
    }

    public void logData(boolean force)
    {
        logData(force, null);
    }

    public void logData()
    {
        logData(false);
    }

    public void setMode (List<DcMotor> motors, DcMotor.RunMode mode)
    {
        RobotLog.dd(TAG, "Setting mode %s on %d motors", mode, motors.size());
        for (DcMotor m : motors)
            m.setMode(mode);
    }
    private void  setPower (List<DcMotor> motors, double power)
    {
        for (DcMotor m : motors)
            m.setPower(power);
    }

    private void setPos(List<Integer> positions, List<DcMotor> motors)
    {
        for(int i = 0; i < motors.size(); i++)
        {
            DcMotor mot = motors.get(i);
            positions.set(i, mot.getCurrentPosition());
        }
    }

    private  void setTargetPositions(List<DcMotor> motors,
                                     List<Integer> tgtPositions)
    {
        for (int m = 0; m < motors.size(); m++)
        {
            motors.get(m).setTargetPosition(tgtPositions.get(m));
        }
    }

    public  void setPositions(List<Integer> dstPos,
                               List<Integer> srcPos,
                               int deltaPosition)
    {
        for (int p = 0; p < srcPos.size(); p++)
        {
            dstPos.set(p, srcPos.get(p) + deltaPosition);
        }
    }

    public  void setPositions(List<Integer> positions,
                               int tgtPosition)
    {
        for (int p = 0; p < positions.size(); p++)
        {
            positions.set(p, tgtPosition);
        }
    }

    public  void initLPositions(List<Integer> positions,
                              int tgtPosition)
    {
        for (int p = 0; p < robot.numLmotors; p++)
        {
            positions.add(p, tgtPosition);
        }
    }

    public  void initRPositions(List<Integer> positions,
                                int tgtPosition)
    {
        for (int p = 0; p < robot.numRmotors; p++)
        {
            positions.add(p, tgtPosition);
        }
    }

    public void setRampSpdL(double rampSpdL) { this.rampSpdL = rampSpdL; }
    public void setRampSpdM(double rampSpdM) { this.rampSpdM = rampSpdM; }
    public void setRampSpdH(double rampSpdH) { this.rampSpdH = rampSpdH; }
    public void setRampCntL(int rampCntL)    { this.rampCntL = rampCntL; }
    public void setRampCntM(int rampCntM)    { this.rampCntM = rampCntM; }
    public void setRampCntH(int rampCntH)    { this.rampCntH = rampCntH; }

    enum MotorSide {LEFT, RIGHT}

    public enum DrivetrainType
    {
        NONE,
        WCD_6_2X20,
        WCD_6_2X40,
        RWD_2_2X40,
        HWD_4_4X40
    }

    private static double DRV_TUNER = 1.00;
    private final static double TRN_TUNER = 1.0;
    private final static double TURN_TOLERANCE = 2.0;
    private int colSensOffset = 0;

    private static double WHL_DIAMETER = 4.1875; //Diameter of the wheel (inches)
    private int encoder_CPR;
    private final static double GEAR_REDUC = 0.5;                   //Gear ratio

    private static double CIRCUMFERENCE = Math.PI * WHL_DIAMETER;
    private double CPI;
    private double DEF_CPI;

    private static final double Kp_GyrCorrection = 0.008;
    private static final double Kp_EncCorrection = 0.01;
    public  static       double Kp_GyroTurn      = 0.03;
    private static final double Kd_GyroTurn      = 0.001;
    private static final double THRESH = Math.toRadians(0.004);

    public enum Direction {FORWARD, REVERSE}

    private ShelbyBot robot;

    private Point2d currPt = new Point2d(0.0, 0.0);
    private double startHdg = 0.0;

    public int frame   = 0;

    private ElapsedTime period  = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime rt = new ElapsedTime();
    private ElapsedTime ptmr = new ElapsedTime();

    public List<Integer> begLpositions = new ArrayList<>();
    public List<Integer> begRpositions = new ArrayList<>();
    public List<Integer> curLpositions = new ArrayList<>();
    public List<Integer> curRpositions = new ArrayList<>();
    public List<Integer> tgtLpositions = new ArrayList<>();
    public List<Integer> tgtRpositions = new ArrayList<>();
    public List<Integer> endLpositions = new ArrayList<>();
    public List<Integer> endRpositions = new ArrayList<>();
    private List<Integer> lstLpositions = new ArrayList<>();
    private List<Integer> lstRpositions = new ArrayList<>();
    private int overLpos;
    private int overRpos;

    private double initHdg;
    public double curHdg;
    private double trgtHdg;
    private double doneHdg;
    private double overHdg;

    public int curRed = 0;
    public int curGrn = 0;
    public int curBlu = 0;

    private double initLpower;
    private double initRpower;
    private double curLpower;
    private double curRpower;

    private ShelbyBot.DriveDir lastDriveDir;
    private ShelbyBot.DriveDir curDriveDir;

    private double xPos = 0.0;
    private double yPos = 0.0;
    public Point2d estPos = new Point2d(xPos, yPos);
    private double  estHdg = 0.0;
    private int numPts = 0;

    private double noMoveTimeout = 1.0;
    private double noDriveMoveTimeout = 0.5;
    private int noMoveThreshLow = 10;  //is set to CPI in init
    private int noMoveThreshHi = 40;   //is set to 4*noMoveThreshLow in init
    private double noMovePwrHi = 0.20;
    private ElapsedTime accelTimer  = new ElapsedTime();
    private ElapsedTime noMoveTimer = new ElapsedTime();

    private double printTimeout = 0.05;

    private double minSpeed = 0.1;
    private double minGyroTurnSpeed = 0.10;

    private double maxStrAng = 40.0;
    private double minStrAng = 5.0;
    private double maxStr = 1.0;
    private double minStr = minGyroTurnSpeed;
    private double strSlope = (maxStr - minStr)/(maxStrAng - minStrAng);
    private double strB = maxStr - strSlope * maxStrAng;

    private CommonUtil com;
    private LinearOpMode op;
    private DataLogger   dl;

    private boolean usePosStop = false;
    private boolean doStopAndReset = false;

    private double lastGyroError = 0;
    private boolean useDterm = false;
    private boolean gyroFirstGood = false;
    private ElapsedTime gyroGoodTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double gyroTimeout = 60;
    private int gyroGoodCount = 0;

    private ElapsedTime gyroFrameTime = new ElapsedTime();
    private ElapsedTime datalogtimer = new ElapsedTime();

    private boolean logData = true;
    private double logDataTimeout = 5;
    public ElapsedTime logDataTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double logTime = 0.01;

    private int curCntTgt = 0;

    double nextPrintTime = ptmr.seconds();
    double nextBusyPrintTime = ptmr.seconds();

    private boolean logOverrun = true;
    private double overtime = 0.12;

    private double reducePower = 0.3;
    private double reduceTurnPower = 0.2;

    private boolean busyAnd = false;
    private boolean rampUp = true;
    private boolean rampDown = false;
    private boolean stopIndividualMotorWhenNotBusy = false;

    private int tickRate = 10;
    private static final int DEF_BUSYTHRESH = 16;
    public  static final int TURN_BUSYTHRESH = 30;
    private static int BUSYTHRESH = DEF_BUSYTHRESH;

    private ElapsedTime busyTimer = new ElapsedTime();
    private double busyTimeOut = 20;
    private double lBusyTime = 0;
    private double rBusyTime = 0;

    private double rampSpdL = 0.09;
    private double rampSpdM = 0.25;
    private double rampSpdH = 0.50;

    private int rampCntL = 240;
    private int rampCntM = 480;
    private int rampCntH = 960;

    private int COLOR_THRESH = 300;

    private boolean lFirst = true;

    private double turnTimeLimit = 3;

    private static final String TAG = "SJH_DTRN";
}


