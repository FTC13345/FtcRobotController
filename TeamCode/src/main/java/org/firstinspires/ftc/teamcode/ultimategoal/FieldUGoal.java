package org.firstinspires.ftc.teamcode.ultimategoal;


import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.odometry.MathFunctions;

import static org.firstinspires.ftc.teamcode.ultimategoal.UGoalRobot.ROBOT_SHOOTING_Y_OFFSET;

/** Main Configuration for Ultimate Goal Challenge game field
 *  The coordinate origin is as per FTC standard in the center of the field
 *  X Axis is parallel to red alliance wall with positive values towards the tower goals and powershots
 *  Y Axis is perpendicular to red alliance wall with positive values towards the blue alliance.
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

    public static final double     poseStartX = -TILE_3_FROM_ORIGIN + ROBOT_RADIUS;
    public static final double     poseStartY = TILE_1_FROM_ORIGIN + ROBOT_RADIUS;
    public static final double     poseStartH = ANGLE_POS_X_AXIS;

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
    public static final double POWER_SHOT_HEIGHT = 30;

    public static final double GOALX                    = TILE_3_FROM_ORIGIN;
    public static final double GOALY                    = TILE_2_CENTER;
    public static final double POWERSHOTX               = TILE_3_FROM_ORIGIN;
    //power shot 1 is furthest from center
    public static final double POWERSHOT_1_Y            = 19;
    public static final double POWERSHOT_2_Y            = POWERSHOT_1_Y-7.5;
    public static final double POWERSHOT_3_Y            = POWERSHOT_2_Y-7.5;
    static final double     DISTANCE_BETWEEN_POWERSHOT  = -7.5;

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

    enum Target { HIGHGOAL, POWERSHOT_1, POWERSHOT_2, POWERSHOT_3}

    // record position that we need to return to repeatedly
    public static Pose2d poseHighGoal = new Pose2d(ORIGIN - 6.0, flip4Red(GOALY - ROBOT_SHOOTING_Y_OFFSET), ANGLE_POS_X_AXIS);
    public static Pose2d posePowerShot1 = new Pose2d(ORIGIN - 4.0, flip4Red(POWERSHOT_1_Y - ROBOT_SHOOTING_Y_OFFSET), ANGLE_POS_X_AXIS);
    public static Pose2d posePowerShot2 = new Pose2d(ORIGIN - 4.0, flip4Red(POWERSHOT_2_Y - ROBOT_SHOOTING_Y_OFFSET), ANGLE_POS_X_AXIS);
    public static Pose2d posePowerShot3 = new Pose2d(ORIGIN - 4.0, flip4Red(POWERSHOT_3_Y - ROBOT_SHOOTING_Y_OFFSET), ANGLE_POS_X_AXIS);

}
