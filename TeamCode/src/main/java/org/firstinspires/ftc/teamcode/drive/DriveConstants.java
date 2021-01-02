package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

import java.util.Arrays;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {

    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static final double TICKS_PER_REV = RobotConstants.DT_CPMR;
    public static final double MAX_RPM = RobotConstants.DT_MAX_RPM;

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = true;

    public static PIDFCoefficients MOTOR_VELO_PID =
        new PIDFCoefficients(14, 0, 0.3, 12.8);
           // getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));
    //f=voltage compensate kf from dash = 12.5-13.5
    //P and D also from dash tuning

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = RobotConstants.DT_WHEEL_DIAM/2.0;
    public static double GEAR_RATIO = RobotConstants.DT_EXT_GEAR_RATIO; // wheelSpd / motspd
    public static double TRACK_WIDTH = RobotConstants.DT_TRACK_WIDTH;
    public static final double DC_CIRC = RobotConstants.DT_CIRCUM;
    public static final double DC_ECIRC = DC_CIRC * GEAR_RATIO;
    public static final double DC_CPI = TICKS_PER_REV / DC_ECIRC;
    public static final double DC_IPC = 1.0/DC_CPI;
    public static final double DC_RPM2VEL = DC_ECIRC / 60.0;

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    //SBH TODO:  adjust these after initial setup
    public static double MAX_VEL = 59; //RR tune  maxVel 59.96
    public static double MAX_ACCEL = 40;
    public static double MAX_ANG_VEL = Math.toRadians(360);
    public static double MAX_ANG_ACCEL = Math.toRadians(270);

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(7.5, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    public final static TrajectoryVelocityConstraint defVelConstraint =
        new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(MAX_ANG_VEL),
            new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
    public final static TrajectoryAccelerationConstraint defAccelConstraint
        = new ProfileAccelerationConstraint(MAX_ACCEL);

    public static double encoderTicksToInches(double ticks) {
//        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
        return ticks * DC_IPC;
    }

    public static double rpmToVelocity(double rpm) {
//        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
        return rpm * DC_RPM2VEL;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
