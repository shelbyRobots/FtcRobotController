package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.util.Point2d;

@SuppressWarnings({"unused", "WeakerAccess"})
public class UgField extends Field
{
    public UgField()
    {
        super("UltimateGoal");
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
    //Red (Left/Right) Start Points
    public static final Point2d RRS1 = new Point2d("RRS1", -61.5,  -48);
    public static final Point2d RLS1 = new Point2d("RLS1", -61.5,  -24);

    //Red (Left/Right) Scan Pt's (Image Scan Pt)
    public static final Point2d RLSP = new Point2d("RLSP", -55.0,  -24);
    public static final Point2d RRSP = new Point2d("RRSP", -55.0,  -48);

    //Red (Left/Right) Dodge Points
    public static final Point2d RLDP = new Point2d("RLDP", -12.0, -19);
    public static final Point2d RRDP = new Point2d("RRDP", -12.0, -53);


    //Red Right Wobble (A/B/C)
    public static final Point2d RRWA = new Point2d("RRWA", 0.0,-53);
    public static final Point2d RRWB = new Point2d("RRWB",  24.0,-36);
    public static final Point2d RRWC = new Point2d("RRWC",  48.0,-53);

    //Red Left Wobble (A/B/C)
    public static final Point2d RLWA = new Point2d("RLWA", 0.0,-53);
    public static final Point2d RLWB = new Point2d("RLWB",  24.0,-36);
    public static final Point2d RLWC = new Point2d("RLWC",  48.0,-53);
    //includes offset for veh center. actual

    public static final Point2d RRPA = new Point2d("RRPA",  0.0,-53);
    public static final Point2d RRPB = new Point2d("RRPB",  0.0,-48);
    public static final Point2d RRPC = new Point2d("RRPC",  0.0,-53);

    public static final Point2d RLPA = new Point2d("RLPA",  0.0,-53);
    public static final Point2d RLPB = new Point2d("RLPB",  0.0,-24);
    public static final Point2d RLPC = new Point2d("RLPC",  0.0,-24);

    private static final int ALNC_RED = 0;
    private static final int ALNC_BLU = 1;
    private static final int STRT_ONE = 0;
    private static final int STRT_TWO = 1;
    private static final int STN_LEFT = 0;
    private static final int STN_CNTR = 1;
    private static final int STN_RGHT = 2;

    private static final String ASSET_NAME = "UltimateGoal";

    private static final float halfField  = 70.5f;
    private static final float quadField  = 34.5f;
    private static final float mmTargetHeight   = 6.0f;

    void setImageNames()
    {
       trackableNames.add("BlueTowerGoal");
       trackableNames.add("RedTowerGoal");
       trackableNames.add("RedAlliance");
       trackableNames.add("BlueAlliance");
       trackableNames.add("FrontWall");
    }

    void setImageLocations()
    {
        float[][] TRACKABLE_POS = {
                scaleArr(new float[]{halfField,  quadField, IMAGE_Z}),
                scaleArr(new float[]{halfField, -quadField, IMAGE_Z}),
                scaleArr(new float[]{     0.0f, -halfField, IMAGE_Z}),
                scaleArr(new float[]{     0.0f,  halfField, IMAGE_Z}),
                scaleArr(new float[]{-halfField,      0.0f, IMAGE_Z})
        };

        locationsOnField.add(genMatrix(TRACKABLE_POS[0], new float[]{90.0f, 0.0f,  -90.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[1], new float[]{90.0f, 0.0f,  -90.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[2], new float[]{90.0f, 0.0f , 180.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[3], new float[]{90.0f, 0.0f ,   0.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[4], new float[]{90.0f, 0.0f,   90.0f}));
    }
}
