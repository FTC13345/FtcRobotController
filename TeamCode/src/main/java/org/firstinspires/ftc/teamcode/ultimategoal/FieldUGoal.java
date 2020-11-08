package org.firstinspires.ftc.teamcode.ultimategoal;


import com.qualcomm.robotcore.hardware.Servo;

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
    public static final double  ANGLE_POS_Y_AXIS = 90.0;
    public static final double  ANGLE_NEG_X_AXIS = 180.0;
    public static final double  ANGLE_NEG_Y_AXIS = -90.0;



    // field distances between objects
    static final double     HALF_TAB                    = 0.75 / 2;
    static final double     TILE_LENGTH                 = 23.5;
    static final double     TILE_WITHOUT_TABS           = 22.75;
    static final double     ROBOT_RADIUS                = 9;
    static final double     POWERSHOT_1_Y               = 19;
    static final double     POWERSHOT_2_Y               = POWERSHOT_1_Y-7.5;
    static final double     POWERSHOT_3_Y               = POWERSHOT_2_Y-7.5;
    static final double     DISTANCE_BETWEEN_POWERSHOT  = 7.5;
    static final double     TILE_1_FROM_ORIGIN          = HALF_TAB + TILE_WITHOUT_TABS;
    static final double     TILE_2_FROM_ORIGIN          = TILE_1_FROM_ORIGIN + TILE_LENGTH;
    static final double     TILE_3_FROM_ORIGIN          = TILE_2_FROM_ORIGIN + TILE_LENGTH;
    static final double     TILE_1_CENTER               = HALF_TAB + (TILE_WITHOUT_TABS / 2);
    static final double     TILE_2_CENTER               = TILE_1_CENTER + TILE_LENGTH;
    static final double     TILE_3_CENTER               = TILE_2_CENTER + TILE_LENGTH;
    static final double     BEHIND_LAUNCH_LINE          = TILE_1_CENTER - ROBOT_RADIUS;
    static final double     INTAKE_DOWN_ANGLE           = Servo.MAX_POSITION; //max is 135 degrees, all the way down
    static final double     PUSHER_REST_POSITION        = Servo.MIN_POSITION;
    static final double     WOBBLE_FINGER_CLOSED        = Servo.MAX_POSITION;
    static final double     WOBBLE_FINGER_OPEN          = 0.5; //middle to save time
    static final double     WOBBLE_CLAW_OPEN            = Servo.MAX_POSITION;
    static final double     WOBBLE_CLAW_CLOSED          = 0;//CHANGE WITH TESTING
    static final double     WOBBLE_CLAW_ARM_INSIDE      = Servo.MIN_POSITION;//needs to be 180 degrees
    static final double     WOBBLE_CLAW_ARM_OUTSIDE     = 0;// 0 degrees
    static final int        ENCODER_TICKS_PER_REVOLUTION        = 288;

    static final int        FINGER_ARM_HORIZONTAL       = ENCODER_TICKS_PER_REVOLUTION/2;
    static final int        FINGER_ARM_UP               = ENCODER_TICKS_PER_REVOLUTION/4;
    static final int        FINGER_ARM_DOWN             = 0;

    //*TODO FIND LIFT TOP AND BOTTOM VALUES, AND TEST FOR WOBBLE RINGS DISTANCE
    static final int        LIFT_TOP                    = 0;
    static final int        LIFT_BOTTOM                 = 0;
    static final int        LIFT_UP_RINGS_HEIGHT        = 0;

    static final double     TARGET_ZONE_A_X             = TILE_1_CENTER;
    static final double     TARGET_ZONE_A_Y             = TILE_3_CENTER;
    static final double     TARGET_ZONE_B_X             = TILE_2_CENTER;
    static final double     TARGET_ZONE_B_Y             = TILE_2_CENTER;
    static final double     TARGET_ZONE_C_X             = TILE_3_CENTER;
    static final double     TARGET_ZONE_C_Y             = TILE_3_CENTER;
    // field elements dimensions or distances go here
    // TARGETS
    public static final double HIGH_GOAL = 35.5;
    public static final double MED_GOAL = 27;
    public static final double LOW_GOAL = 17;
    public static final double POWER_SHOT = 26;

}
