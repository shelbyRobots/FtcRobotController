package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.image.MineralDetector;
import org.firstinspires.ftc.teamcode.util.Point2d;

@SuppressWarnings({"unused", "WeakerAccess"})
public class RoRuField extends Field
{
    public RoRuField()
    {
        super("RoverRuckus");
        setHasVuMarks(false);
    }

    //Point naming key:
    //1st char: R=Red, B=Blue
    //2nd char: L=Left start, R=Right start (viewed from red side - along field X)
    //3rd-4th chars: Pt description

    private static final String TAG = " SJH_RFD";

    private static final int BLUE  = 0;
    private static final int RED   = 1;
    private static final int STRT1 = 0;
    private static final int STRT2 = 1;
    private static final int LEFT  = 0;
    private static final int CNTR  = 1;
    private static final int RGHT  = 2;

    //Red Route > Left
    static final Point2d RLLP = new Point2d("RLLP", -13.0,  -13.0);
    static final Point2d RLTP = new Point2d("RLTP", -16.0,  -16.0);
    //includes offset for veh center. actual
    // RLM1 -46, -24; RLM2 -36, -36; RLM3 -25, -46
    static final Point2d RLM1 = new Point2d("RLM1", -43.0,  -23.0); //R
    static final Point2d RLM2 = new Point2d("RLM2", -33.0,  -33.0); //C
    static final Point2d RLM3 = new Point2d("RLM3", -21.0,  -45.0); //L

    static final Point2d RLR1 = new Point2d("RLR1", -18.0,  -18.0); //OX -17, -17
    static final Point2d RLR2 = new Point2d("RLR2",  -8.0,  -56.0); //OX -7, -56
    static final Point2d RLDT = new Point2d("RLDT",  24.0,  -58.5);
    static final Point2d RLDP = new Point2d("RLDP",  48.0,  -58.0);
    static final Point2d RLPG = new Point2d("RLPG",  33.0,  -33.0); //prntr gold
    static final Point2d RLPP = new Point2d("RLPP", -19.0,  -59.0);

    static final Point2d RRLP = new Point2d("RRLP",  13.0, -13.0);
    static final Point2d RRTP = new Point2d("RRTP",  16.0, -16.0);
    // RLM3 46, -24; RRM2 36, -36; RRM1 25, -46
    static final Point2d RRM1 = new Point2d("RRM1",  25.0, -57.0); //R
    static final Point2d RRM2 = new Point2d("RRM2",  41.0, -41.0); //C
    static final Point2d RRM3 = new Point2d("RRM3",  57.0, -25.0); //L

    static final Point2d RRR1 = new Point2d("RRR1",  59.0, -24.0);
    //static final Point2d RRR2 = new Point2d("RRR2",  57.0,  -7.0);
    //static final Point2d RRDT = new Point2d("RRDT",  59.0, -24.0);
    static final Point2d RRDP = new Point2d("RRDP",  57.0, -54.0);
    static final Point2d RRPP = new Point2d("RRPP",  59.0,  16.0);

    static final Point2d BLM1 = new Point2d("BLM1",  43.0,  23.0); //R
    static final Point2d BLM2 = new Point2d("BLM2",  33.0,  33.0); //C
    static final Point2d BLM3 = new Point2d("BLM3",  21.0,  45.0); //L

    static final Point2d BRM1 = new Point2d("BRM1", -25.0,  57.0); //R
    static final Point2d BRM2 = new Point2d("BRM2", -41.0,  41.0); //C
    static final Point2d BRM3 = new Point2d("BRM3", -57.0,  25.0); //L

    static final Point2d RLML = new Point2d("RLML", -25.0,  -45.0);
    static final Point2d RLMC = new Point2d("RLMC", -35.0,  -35.0);
    static final Point2d RLMR = new Point2d("RLMR", -45.0,  -25.0);

    static final Point2d RRML = new Point2d("RRML",  45.0,  -25.0);
    static final Point2d RRMC = new Point2d("RRMC",  35.0,  -35.0);
    static final Point2d RRMR = new Point2d("RRMR",  25.0,  -45.0);

    static final Point2d BLML = new Point2d("BLML",  25.0,   45.0);
    static final Point2d BLMC = new Point2d("BLMC",  35.0,   35.0);
    static final Point2d BLMR = new Point2d("BLMR",  45.0,   25.0);

    static final Point2d BRML = new Point2d("BRML", -45.0,   25.0);
    static final Point2d BRMC = new Point2d("BRMC", -35.0,   35.0);
    static final Point2d BRMR = new Point2d("BRMR", -25.0,   45.0);


    private static final int ALNC_RED = 0;
    private static final int ALNC_BLU = 1;
    private static final int STRT_ONE = 0;
    private static final int STRT_TWO = 1;
    private static final int MIN_LEFT = 0;
    private static final int MIN_CNTR = 1;
    private static final int MIN_RGHT = 2;

    private static Point2d[][][] minOffsetPts =
            {{{RLM3, RLM2, RLM1}, {RRM3, RRM2, RRM1}}, {{BLM3, BLM2, BLM1}, {BRM3, BRM2, BRM1}}};

    private static Point2d[][][] minActualPts =
            {{{RLML, RLMC, RLMR}, {RRML, RRMC, RRMR}}, {{BLML, BLMC, BLMR}, {BRML, BRMC, BRMR}}};

    public static Point2d getMinOffsetPt(Alliance alnc,
                                         PositionOption startPos,
                                         MineralDetector.Position minPos)
    {
        int allnc = alnc == Alliance.RED ? 0 : 1;
        int start = startPos == Route.StartPos.START_1 ? 0 : 1;
        int minps = 1; //CENTER or NONE
        if      (minPos == MineralDetector.Position.LEFT) minps = 0;
        else if (minPos == MineralDetector.Position.RIGHT) minps = 2;
        return minOffsetPts[allnc][start][minps];
    }

    public static Point2d getMinActualPt(Alliance alnc,
                                         PositionOption startPos,
                                         MineralDetector.Position minPos)
    {
        int allnc = alnc == Alliance.RED ? 0 : 1;
        int start = startPos == Route.StartPos.START_1 ? 0 : 1;
        int minps = 1; //CENTER or NONE
        if      (minPos == MineralDetector.Position.LEFT) minps = 0;
        else if (minPos == MineralDetector.Position.RIGHT) minps = 2;
        return minActualPts[allnc][start][minps];
    }

    public static Point2d getMineralPt(Alliance alnc,
                                       PositionOption startPos,
                                       MineralDetector.Position minPos)
    {
        Route.StartPos spos = Route.StartPos.START_1;
        if(startPos instanceof Route.StartPos) spos = (Route.StartPos) startPos;
        Point2d minPt = null;
        switch(alnc)
        {
            case RED: {
                switch (spos) {
                    case START_1: {
                        switch (minPos) {
                            case LEFT:    minPt = RLM3; break;
                            case RIGHT:   minPt = RLM1; break;
                            case CENTER:
                            case NONE:    minPt = RLM2; break;
                        }
                        break;
                    }
                    case START_2: {
                        switch (minPos) {
                            case LEFT:    minPt = RRM3; break;
                            case RIGHT:   minPt = RRM1; break;
                            case CENTER:
                            case NONE:    minPt = RRM2; break;
                        }
                        break;
                    }
                }
                break;
            }
            case BLUE: {
                switch (spos) {
                    case START_1: {
                        switch (minPos) {
                            case LEFT:    minPt = BLM3; break;
                            case RIGHT:   minPt = BLM1; break;
                            case CENTER:
                            case NONE:    minPt = BLM2; break;
                        }
                        break;
                    }
                    case START_2: {
                        switch (minPos) {
                            case LEFT:    minPt = BRM3; break;
                            case RIGHT:   minPt = BRM1; break;
                            case CENTER:
                            case NONE:    minPt = BRM2; break;
                        }
                        break;
                    }
                }
                break;
            }
        }
        return minPt;
    }

    void setImageNames()
    {
        trackableNames.add("BluePerimeter_Rover");
        trackableNames.add("RedPerimeter_Footprint");
        trackableNames.add("FrontPerimeter_Craters");
        trackableNames.add("BackPerimeter_Space");
    }

    void setImageLocations()
    {
        float[][] TRACKABLE_POS = {
                scaleArr(new float[]{0.0f, N_WALL_Y, IMAGE_Z}),
                scaleArr(new float[]{0.0f, S_WALL_Y, IMAGE_Z}),
                scaleArr(new float[]{E_WALL_X, 0.0f, IMAGE_Z}),
                scaleArr(new float[]{W_WALL_X, 0.0f, IMAGE_Z})
        };

        locationsOnField.add(genMatrix(TRACKABLE_POS[0], new float[]{90.0f, 0.0f, 0.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[1], new float[]{90.0f, 0.0f, 180.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[2], new float[]{90.0f, 0.0f, 90.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[3], new float[]{90.0f, 0.0f, 270.0f}));
    }
}
