package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.odometry.MathFunctions;

/* Main Configuration for Ultimate Goal Challenge game field
 *  The coordinate origin is as per FTC standard in the center of the field
 *  X Axis is parallel to red alliance wall with positive values towards the tower goals and powershots
 *  Y Axis is perpendicular to red alliance wall with positive values towards the blue alliance.
 */
/**
 * Each floor tile is 23.5 inch square (counting tabs on one side and not on the other side)
 * Each floor tile with all side tabs cut off is 22.75 inch square
 * The tabs add 0.75 to tile width on each side.
 * Field width = 23.5 * 6 - 0.75 = 70.25 each side square
 *
 * Robot is 18x18 square. Robot (x,y) position is at the center of the robot.
 */

public class FieldUGoal {

    // FTC Team alliance color, BLUE or RED, the field is mirror images for each side
    // therefore lot of robot movement calculations are affected by which color you are on
    enum AllianceColor { BLUE, RED}
    public static final double  ANGLE_POS_X_AXIS = 0.0;
    public static final double  ANGLE_POS_Y_AXIS = Math.PI / 2;
    public static final double  ANGLE_NEG_X_AXIS = Math.PI;
    public static final double  ANGLE_NEG_Y_AXIS = -ANGLE_POS_Y_AXIS;

    // field distances between objects
    public static final double     TILE_LENGTH                 = 23.5;
    public static final double     TILE_WITHOUT_TABS           = 22.75;
    public static final double     ROBOT_RADIUS                = 8.5;

    public static final double     ORIGIN                      = 0.0;  // applicable for both X-Axis and Y-Axis
    public static final double     TILE_1_FROM_ORIGIN          = ORIGIN + TILE_LENGTH;
    public static final double     TILE_2_FROM_ORIGIN          = TILE_1_FROM_ORIGIN + TILE_LENGTH;
    public static final double     TILE_3_FROM_ORIGIN          = TILE_2_FROM_ORIGIN + TILE_LENGTH;
    public static final double     TILE_1_CENTER               = TILE_LENGTH / 2;
    public static final double     TILE_2_CENTER               = TILE_1_CENTER + TILE_LENGTH;
    public static final double     TILE_3_CENTER               = TILE_2_CENTER + TILE_LENGTH;

    public static final double     TARGET_ZONE_A_X             = TILE_1_CENTER;
    public static final double     TARGET_ZONE_A_Y             = TILE_3_CENTER;
    public static final double     TARGET_ZONE_B_X             = TILE_2_CENTER;
    public static final double     TARGET_ZONE_B_Y             = TILE_2_CENTER;
    public static final double     TARGET_ZONE_C_X             = TILE_3_CENTER;
    public static final double     TARGET_ZONE_C_Y             = TILE_3_CENTER;

    // TARGETS
    public static final double HIGH_GOAL_HEIGHT = 35.5;
    public static final double MED_GOAL_HEIGHT = 27;
    public static final double LOW_GOAL_HEIGHT = 17;
    public static final double POWER_SHOT_HEIGHT = 33;

    public static final double GOALX                    = TILE_3_FROM_ORIGIN;
    public static final double GOALY                    = TILE_2_CENTER;
    public static final double POWERSHOTX               = TILE_3_FROM_ORIGIN;
    //power shot 1 is furthest from center
    public static final double POWERSHOT_1_Y            = 19;
    public static final double POWERSHOT_2_Y            = POWERSHOT_1_Y-7.5;
    public static final double POWERSHOT_3_Y            = POWERSHOT_2_Y-7.5;
    public static final double DISTANCE_BETWEEN_POWERSHOT  = -7.5;

    // Robot function or game play specific values, maybe these need to do into a different file
    public static final long        RING_SHOOTING_INTERVAL      = 1000; // milliseconds
    public static final double      ROBOT_TURN_SMALL            = 0.05; // 0.05 Radians = 2.86 degrees
    // Tuning Tuning: Compensation for robot behavior, it shoots curved to the left, by few inches
    // Perform calculations as if the Robot center was to the left by few inches and shooting hits target straight ahead
    // Given that the Robot is directly facing the goal line (Heading = 0 (+ve X-axis)), we will also
    // actually position on the field to the right of the intended Target Y coordinate
    public static final double      ROBOT_SHOOTING_Y_OFFSET     = 10.0; // inches

    enum Target { HIGHGOAL, POWERSHOT_1, POWERSHOT_2, POWERSHOT_3}

    // record position that we need to return to repeatedly
    // Robot starting position is at the audience wall farthest from the Goals, on the start line
    public static final double     poseStartX = -TILE_3_FROM_ORIGIN + ROBOT_RADIUS;
    public static final double     poseStartY = TILE_1_FROM_ORIGIN + ROBOT_RADIUS;
    public static Pose2d poseStart = new Pose2d(poseStartX, poseStartY, ANGLE_POS_X_AXIS);
    public static Pose2d poseHighGoal = new Pose2d(ORIGIN - 6.0, flip4Red(GOALY - ROBOT_SHOOTING_Y_OFFSET), ANGLE_POS_X_AXIS);
    public static Pose2d posePowerShot1 = new Pose2d(ORIGIN - 6.0, flip4Red(POWERSHOT_1_Y - ROBOT_SHOOTING_Y_OFFSET), ANGLE_POS_X_AXIS);
    public static Pose2d posePowerShot2 = new Pose2d(ORIGIN - 6.0, flip4Red(POWERSHOT_2_Y - ROBOT_SHOOTING_Y_OFFSET), ANGLE_POS_X_AXIS);
    public static Pose2d posePowerShot3 = new Pose2d(ORIGIN - 6.0, flip4Red(POWERSHOT_3_Y - ROBOT_SHOOTING_Y_OFFSET), ANGLE_POS_X_AXIS);
    public static Pose2d posePark = new Pose2d(TILE_1_CENTER, flip4Red(TILE_1_FROM_ORIGIN), ANGLE_NEG_X_AXIS);
    // Robot positioned touching side wall (left side on BLUE field) with front of robot touching Y-Axis line
    public static Pose2d poseResetOdometry = new Pose2d(ORIGIN - ROBOT_RADIUS, TILE_3_FROM_ORIGIN - ROBOT_RADIUS, ANGLE_POS_X_AXIS);

    public static AllianceColor aColor = AllianceColor.BLUE;

    public static double flip4Red(double value) {
        if (aColor == AllianceColor.RED) {
            value = -value;
        }
        return value;
    }

    public static double flipAngle4Red(double value) {
        if (aColor == AllianceColor.RED) {
            value = MathFunctions.angleWrap(180 - value);
        }
        return value;
    }

}
