package org.firstinspires.ftc.teamcode.ultimategoal;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

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
    public static final double POWER_SHOT_HEIGHT = 31.5;    // this value is tuned by field experimentation, to get correct tilt angle of shooting platform

    public static final double GOALX                    = TILE_3_FROM_ORIGIN;
    public static final double GOALY                    = TILE_2_CENTER;
    public static final double POWERSHOTX               = TILE_3_FROM_ORIGIN;
    //power shot 1 is furthest from center
    public static final double POWERSHOT_1_Y            = 19;
    public static final double POWERSHOT_2_Y            = POWERSHOT_1_Y-7.5;
    public static final double POWERSHOT_3_Y            = POWERSHOT_2_Y-7.5;
    public static final double DISTANCE_BETWEEN_POWERSHOT  = -7.5;

    // Robot function or game play specific values, maybe these need to do into a different file
    public static final long        RING_SHOOTING_INTERVAL      = 600; // milliseconds
    public static final double      ROBOT_ROTATE_POWERSHOT        = 3.6; // degrees by empirical measurement
    // Tuning Tuning: Compensation for robot behavior, it shoots curved to the left, by few inches
    // Perform calculations as if the Robot center was to the left by few inches and shooting hits target straight ahead
    // Given that the Robot is directly facing the goal line (Heading = 0 (+ve X-axis)), we will also
    // actually position on the field to the right of the intended Target Y coordinate
    public static final double      ROBOT_SHOOTING_Y_OFFSET     = 8.0; // inches

    enum Target { HIGHGOAL, POWERSHOT_1, POWERSHOT_2, POWERSHOT_3, WOBBLE_LANDING_1, WOBBLE_LANDING_2}

    // record position that we need to return to repeatedly
    // Robot starting position is at the audience wall farthest from the Goals, on the start line
    public static Pose2d poseStart = new Pose2d(-TILE_3_FROM_ORIGIN + ROBOT_RADIUS, TILE_1_FROM_ORIGIN + ROBOT_RADIUS, ANGLE_POS_X_AXIS);
    // Ideal heading angle is ANGLE_POS_X_AXIS, but we need to Field Tuning compensation for Robot driving error
    public static Pose2d poseHighGoal = new Pose2d(ORIGIN - 6.0, flip4Red(GOALY - ROBOT_SHOOTING_Y_OFFSET), ANGLE_POS_X_AXIS);
    public static Pose2d poseHighGoal2 = new Pose2d(poseHighGoal.vec(), Angle.norm(ANGLE_POS_X_AXIS - Math.toRadians(4)));
    public static Pose2d posePowerShot1 = new Pose2d(ORIGIN - 6.0, flip4Red(POWERSHOT_1_Y - ROBOT_SHOOTING_Y_OFFSET), ANGLE_POS_X_AXIS);
    public static Pose2d posePowerShot2 = new Pose2d(ORIGIN - 6.0, flip4Red(POWERSHOT_2_Y - ROBOT_SHOOTING_Y_OFFSET), ANGLE_POS_X_AXIS);
    public static Pose2d posePowerShot3 = new Pose2d(ORIGIN - 6.0, flip4Red(POWERSHOT_3_Y - ROBOT_SHOOTING_Y_OFFSET), ANGLE_POS_X_AXIS);
    public static Pose2d posePark = new Pose2d(TILE_1_CENTER, flip4Red(TILE_1_CENTER), ANGLE_NEG_X_AXIS);
    public static Pose2d poseWobblePickup = new Pose2d(-TILE_2_FROM_ORIGIN + 2.0, TILE_2_CENTER + 2.0, ANGLE_POS_X_AXIS);
    public static Vector2d vecRingStack = new Vector2d(-(TILE_1_FROM_ORIGIN - 2.0), TILE_2_CENTER);

    // Ring stack is picked up at an angle heading. Robot rotates to the angle and drives reverse at that angle to pickup rings from stack
    // theta = Math.atan( 10 / (-15.5)) = -0.572966 Radians or -32.828 degrees  (when poseHighGoal = (-6.0, +27.5) coordinates)
    public static final double ANGLE_RINGSTACK_PICKUP = Math.atan((vecRingStack.getY() - poseHighGoal.getY() + 2.5) / (vecRingStack.getX() - poseHighGoal.getX())); // constant is tuning adjustment
    public static final Pose2d poseRingPickupStart = poseHighGoal.plus(new Pose2d(0,0, ANGLE_RINGSTACK_PICKUP)); // We expect Robot turns to ANGLE_RINGSTACK_PICKUP after shooting rings

    // we want to drive DIST_PAST_RINGSTACK inches through the ring stack, calculate the x,y coordinate values and end vector
    static final double DIST_PAST_RINGSTACK = 7.0; // inches
    static final double xDistance = DIST_PAST_RINGSTACK * Math.cos(ANGLE_RINGSTACK_PICKUP + Math.PI);    // Add 180 deg (PI radians) for reverse direction of robot heading
    static final double yDistance = DIST_PAST_RINGSTACK * Math.sin(ANGLE_RINGSTACK_PICKUP + Math.PI);
    public static final Vector2d vecRingPickupEnd = vecRingStack.plus(new Vector2d(xDistance, yDistance));

    // Robot positioned next to the Audience wall, ready to drop the wobble over the perimeter wall into the landing zone
    public static Vector2d poseWobbleLanding1 = new Vector2d(-TILE_3_CENTER, flip4Red(TILE_2_CENTER));
    public static Vector2d poseWobbleLanding2 = new Vector2d(-TILE_3_CENTER, flip4Red(TILE_1_CENTER));
    // Robot positioned touching side wall (left side on BLUE field) with front of robot touching Y-Axis line
    public static Pose2d poseOdoLeft = new Pose2d(ORIGIN - ROBOT_RADIUS, TILE_3_FROM_ORIGIN - ROBOT_RADIUS, ANGLE_POS_X_AXIS);
    // Robot positioned touching the side wall (right side on BLUE HALF field) with front of robot touching launch line
    public static Pose2d poseOdoRight = new Pose2d(TILE_1_CENTER - ROBOT_RADIUS - 1, -TILE_1_FROM_ORIGIN + ROBOT_RADIUS, ANGLE_POS_X_AXIS);

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
