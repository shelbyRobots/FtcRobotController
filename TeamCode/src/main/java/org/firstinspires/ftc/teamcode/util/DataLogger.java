package org.firstinspires.ftc.teamcode.util;

import android.os.Environment;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.Calendar;
import java.util.Date;
import java.util.Locale;

@SuppressWarnings({"FieldCanBeLocal", "WeakerAccess", "unused"})
public class DataLogger {
    private Writer writer;
    private BufferedWriter bw;
    private StringBuilder sb;
    private long msBase;
    private long nsBase;
    private boolean lineStart = false;
    private boolean logData   = true;

    private String timeFormat = "%.3f";
    private char fs = ',';
    private char nl = '\n';

    public DataLogger()
    {
        this("date");
    }

    public DataLogger(String fileName)
    {
        if(fileName.equals("date"))
        {
            Date day = new Date();
            Calendar cal = Calendar.getInstance();
            cal.setTime(day);
            fileName = String.format(Locale.US, "%d%02d%02d_%02d%02d",
                    cal.get(Calendar.YEAR),
                    cal.get(Calendar.MONTH)+1,
                    cal.get(Calendar.DAY_OF_MONTH),
                    cal.get(Calendar.HOUR_OF_DAY),
                    cal.get(Calendar.MINUTE));
        }

        if(logData)
        {
            sb = new StringBuilder(128);
            //String directoryPath  = "/sdcard/FIRST/DataLogger";
            String directoryPath  = Environment.getExternalStorageDirectory().getPath() +
                                            "/FIRST/DataLogger";
            String filePath       = directoryPath + "/" + fileName + ".csv";

            File fDir = new File(directoryPath);
            // Create directory if it does not exist
            if (!fDir.isDirectory() && !fDir.mkdirs())
            {
                RobotLog.ee("SJH", "Could not create directory " + directoryPath);
            }

            try
            {
                writer = new FileWriter(filePath);
                bw = new BufferedWriter(writer);
            } catch (IOException e)
            {
                RobotLog.ee("SJH", "Could not create file " + filePath);
            }

            msBase = System.currentTimeMillis();
            nsBase = System.nanoTime();
            try
            {
                bw.append("sec, d ms");
            } catch (IOException e)
            {
                RobotLog.ee("SJH", "Could not write header");
            }
        }
    }

    public void resetTime()
    {
        msBase = System.currentTimeMillis();
        nsBase = System.nanoTime();
    }

    private void flushLineBuffer()
    {
        try
        {
            if(bw != null) bw.append(sb);
            if(sb != null) sb.delete(0, sb.length());
        }
        catch (IOException e)
        {
            RobotLog.ee("SJH", "Could not write in flushLineBuffer()");
        }
        lineStart = true;
    }

    private void logTime()
    {
        long milliTime   = System.currentTimeMillis();
        long nanoTime    = System.nanoTime();

        double pmil = (milliTime - msBase) / 1.0E3;
        double pnan =  (nanoTime - nsBase) / 1.0E6;

        if (!lineStart) sb.append(fs);
        sb.append(String.format(Locale.US, timeFormat + "," + timeFormat, pmil, pnan));

        nsBase      = nanoTime;
        lineStart = false;
    }

    private void sep()
    {
        if(lineStart) logTime();
        sb.append(fs);
    }

    public void closeDataLogger()
    {
        try
        {
            if(bw != null) bw.close();
            if(writer != null) writer.close();
        }
        catch (IOException e)
        {
            RobotLog.ee("SJH", "IoException in closeDataLogger()");
        }
    }

    public void addField(String s)  { sep(); sb.append(s); }
    public void addField(char c)    { sep(); sb.append(c); }
    public void addField(boolean b) { sep(); sb.append(b); }
    public void addField(byte b)    { sep(); sb.append(Byte.toString(b)); }
    public void addField(short s)   { sep(); sb.append(s); }
    public void addField(long l)    { sep(); sb.append(l); }
    public void addField(float f)   { sep(); sb.append(f); }
    public void addField(double d)  { sep(); sb.append(d); }
    public void addField(String light, double d) {
        addField(d);
    }

    public void newLine()
    {
        sb.append(nl);
        flushLineBuffer();
    }

    @Override
    protected void finalize() throws Throwable {
        closeDataLogger();
        super.finalize();
    }

    public static void main(String args[])
    {
        String TAG = "TEST";
        DataLogger dl = new DataLogger("sbhTest.csv");
        dl.addField('A');
        dl.addField("_bcd)");
        dl.addField(true);
        dl.newLine();
        dl.addField(5);
        dl.addField(1.234f);
        dl.addField(Math.PI);
        dl.addField(1234567890);
        dl.newLine();
    }
}
