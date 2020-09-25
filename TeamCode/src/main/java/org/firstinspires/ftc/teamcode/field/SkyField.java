package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.util.Point2d;

@SuppressWarnings({"unused", "WeakerAccess"})
public class SkyField extends Field
{
    public SkyField()
    {
        super("Skystone");
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
    //Red Left Start Point 1
    static final Point2d RLS1 = new Point2d("RLS1", -34.5,  -61.5);

    //Red Scan Pt (Image Scan Pt)
    static final Point2d RIS1 = new Point2d("RIS1", -34.5,  -50.0);
    //includes offset for veh center. actual

    //Red Stone Points (Stone center location on field)
    static final Point2d RSP1 = new Point2d("RSP1", -66.5,   -21.0); //R
    static final Point2d RSP2 = new Point2d("RSP2", -58.5,   -21.0); //R
    static final Point2d RSP3 = new Point2d("RSP3", -50.5,   -21.0); //R
    static final Point2d RSP4 = new Point2d("RSP4", -42.5,   -21.0); //R
    static final Point2d RSP5 = new Point2d("RSP5", -34.5,   -21.0); //R
    static final Point2d RSP6 = new Point2d("RSP6", -26.5,   -21.0); //R

    static final Point2d RGPA = new Point2d("RGPA", -34.52,  -39.0); //R
    static final Point2d RGPB = new Point2d("RGPB", -58.5,   -39.0); //R
    static final Point2d RAPA = new Point2d("RAPA", -34.52,  -39.0); //R
    static final Point2d RAPB = new Point2d("RAPB", -35.0,   -39.0); //R
    static final Point2d RDPA = new Point2d("RDPA",  42.0,   -39.0); //R
    static final Point2d RDPB = new Point2d("RDPB",  32.0,   -39.0); //R
    static final Point2d RDPC = new Point2d("RDPC",  16.0,   -39.0);
    static final Point2d RDPD = new Point2d("RDPD",  35.0,   -39.0);
    static final Point2d RDPF = new Point2d("RDPF",  31.0,   -39.0);
    static final Point2d RDPE = new Point2d("RDPE",  14.0,   -39.0);

    static final Point2d RTP1 = new Point2d("RTP1", -34.5, -39.0); //R
    static final Point2d RPP1 = new Point2d("RPP1",   5.0, -39.0); //R

    //platch
    static final Point2d RBS1 = new Point2d( "RBS1", 36.0, -61.5);
    static final Point2d RBP1 = new Point2d( "RBP1", 36.0, -50.0);
    static final Point2d RBP2 = new Point2d( "RBP2", 48.0, -39.0);
    static final Point2d RBP3 = new Point2d( "RBP3", 48.0, -33.0); //latch
    static final Point2d RBP4 = new Point2d( "RBP4", 44.0, -52.0);
    static final Point2d RBP5 = new Point2d( "RBP5", 42.0, -52.0); //unlatch
    static final Point2d RBP6 = new Point2d( "RBP6",  4.0, -60.0);

    static final Point2d RPPS = new Point2d( "RPPS",  9.0, -61.5);
    static final Point2d RPP6 = new Point2d( "RPP6",  0.0, -61.5);


    private static final int ALNC_RED = 0;
    private static final int ALNC_BLU = 1;
    private static final int STRT_ONE = 0;
    private static final int STRT_TWO = 1;
    private static final int STN_LEFT = 0;
    private static final int STN_CNTR = 1;
    private static final int STN_RGHT = 2;

    private static final String ASSET_NAME = "Skystone";

    private static final float STONEZ = 2.0f;
    private static final float bridgeZ = 6.42f;
    private static final float bridgeY = 23.0f;
    private static final float bridgeX = 5.18f;
    private static final float bridgeRotY = 59.0f;
    private static final float bridgeRotZ = 180.0f;

    private static final float halfField  = 70.5f;
    private static final float quadField  = 34.5f;
    private static final float mmTargetHeight   = 6.0f;

    void setImageNames()
    {
       trackableNames.add("Stone Target");
       trackableNames.add("Blue Rear Bridge");
       trackableNames.add("Red Rear Bridge");
       trackableNames.add("Red Front Bridge");
       trackableNames.add("Blue Front Bridge");
       trackableNames.add("Red Perimeter 1");
       trackableNames.add("Red Perimeter 2");
       trackableNames.add("Front Perimeter 1");
       trackableNames.add("Front Perimeter 2");
       trackableNames.add("Blue Perimeter 1");
       trackableNames.add("Blue Perimeter 2");
       trackableNames.add("Rear Perimeter 1");
       trackableNames.add("Rear Perimeter 2");
    }

    void setImageLocations()
    {
        float[][] TRACKABLE_POS = {

                scaleArr(new float[]{0.0f, 0.0f, STONEZ}),
                scaleArr(new float[]{ bridgeX,  bridgeY, bridgeZ}),
                scaleArr(new float[]{ bridgeX, -bridgeY, bridgeZ}),
                scaleArr(new float[]{-bridgeX, -bridgeY, bridgeZ}),
                scaleArr(new float[]{-bridgeX,  bridgeY, bridgeZ}),
                scaleArr(new float[]{ quadField, -halfField, mmTargetHeight}),
                scaleArr(new float[]{-quadField, -halfField, mmTargetHeight}),
                scaleArr(new float[]{-halfField, -quadField, mmTargetHeight}),
                scaleArr(new float[]{-halfField,  quadField, mmTargetHeight}),
                scaleArr(new float[]{-quadField,  halfField, mmTargetHeight}),
                scaleArr(new float[]{ quadField,  halfField, mmTargetHeight}),
                scaleArr(new float[]{ halfField,  quadField, mmTargetHeight}),
                scaleArr(new float[]{ halfField, -quadField, mmTargetHeight})
        };

        locationsOnField.add(genMatrix(TRACKABLE_POS[0], new float[]{90.0f, 0.0f, -90.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[1], new float[]{0.0f,  bridgeRotY, bridgeRotZ}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[2], new float[]{0.0f, -bridgeRotY, bridgeRotZ}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[3], new float[]{0.0f, -bridgeRotY, 0.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[4], new float[]{0.0f,  bridgeRotY, 0.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[5], new float[]{90.0f, 0.0f, 180.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[6], new float[]{90.0f, 0.0f, 180.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[7], new float[]{90.0f, 0.0f , 90.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[8], new float[]{90.0f, 0.0f , 90.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[9], new float[]{90.0f, 0.0f, 0.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[10], new float[]{90.0f, 0.0f, 0.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[11], new float[]{90.0f, 0.0f , -90.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[12], new float[]{90.0f, 0.0f , -90.0f}));
    }
}
