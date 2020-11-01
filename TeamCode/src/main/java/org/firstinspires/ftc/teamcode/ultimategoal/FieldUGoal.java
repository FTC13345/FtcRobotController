package org.firstinspires.ftc.teamcode.ultimategoal;


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
    static final double     TILE_1_FROM_ORIGIN          = HALF_TAB + TILE_WITHOUT_TABS;
    static final double     TILE_2_FROM_ORIGIN          = TILE_1_FROM_ORIGIN + TILE_LENGTH;
    static final double     TILE_3_FROM_ORIGIN          = TILE_2_FROM_ORIGIN + TILE_LENGTH;
    static final double     TILE_1_CENTER               = HALF_TAB + (TILE_WITHOUT_TABS / 2);
    static final double     TILE_2_CENTER               = TILE_1_CENTER + TILE_LENGTH;
    static final double     TILE_3_CENTER               = TILE_2_CENTER + TILE_LENGTH;

    // field elements dimensions or distances go here
    // TARGETS
    public static final double HIGH_GOAL = 48;
    public static final double MED_GOAL = 36;
    public static final double LOW_GOAL = 24;
}
