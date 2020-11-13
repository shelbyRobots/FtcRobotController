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
    public static final Point2d ROS1 = new Point2d("ROS1", -61.5,  -47);
    public static final Point2d RIS1 = new Point2d("RIS1", -61.5,  -24);

    //Red (Left/Right) Scan Pt's (Image Scan Pt)
    public static final Point2d ROSP = new Point2d("ROSP", -61.45,  -47);
    public static final Point2d RISP = new Point2d("RISP", -61.45,  -24);

    //Red (Left/Right) Turn Pt's (Image Scan Pt)
    public static final Point2d ROTP = new Point2d("ROTP", -55.0,  -47);
    public static final Point2d RITP = new Point2d("RITP", -55.0,  -24);

    //Red (Left/Right) Dodge Points
    public static final Point2d RODP = new Point2d("RODP", -12.0, -53);
    public static final Point2d RIDP = new Point2d("RIDP", -12.0, -19);
    
    //Red Right Wobble (A/B/C)
    public static final Point2d ROWA = new Point2d("ROWA",   6,-53);
    public static final Point2d ROWB = new Point2d("ROWB",  18.0,-47);
    public static final Point2d ROWC = new Point2d("ROWC",  40.0,-53);

    //Red Left Wobble (A/B/C)
    public static final Point2d RIWA = new Point2d("RIWA",  8,-44);
    public static final Point2d RIWB = new Point2d("RIWB",  8,-26);
    public static final Point2d RIWC = new Point2d("RIWC",  8,-46);
    //includes offset for veh center. actual

    public static final Point2d ROPA = new Point2d("ROPA",  0,-53);
    public static final Point2d ROPB = new Point2d("ROPB",  0,-53);
    public static final Point2d ROPC = new Point2d("ROPC",  0,-53);

    public static final Point2d RIPA = new Point2d("RIPA",  0,-20);
    public static final Point2d RIPB = new Point2d("RIPB",  0,-20);
    public static final Point2d RIPC = new Point2d("RIPC",  0,-20);

    //shoot points, Red Outside Shoot A
    public static final Point2d ROSA = new Point2d("ROSA", -4, -53);
    public static final Point2d RISA = new Point2d("RISA", -4, -20);

    // Red Outside Grab Wobble
    public static final Point2d ROGW = new Point2d("ROGW", -54, -8);

    //Red Outside Turn
    public static final Point2d ROT2 = new Point2d("ROT2", -20, -53);

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
