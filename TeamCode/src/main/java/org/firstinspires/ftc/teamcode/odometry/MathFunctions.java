package org.firstinspires.ftc.teamcode.odometry;

public class MathFunctions {

    /**
     * Makes sure any angle is within the range -180 to 180
     * @param angle in degrees
     * @return angle with range -180 to 180
     */
    public static double angleWrap(double angle) {
        while (angle <= -180) {
            angle += 360;
        }

        while (angle > 180) {
            angle -= 360;
        }

        return angle;
    }

    /**
     * Makes sure any angle is within the range -PI to +PI
     * @param angle in radians
     * @return angle with range -PI to +PI
     */
    public static double angleWrapRad(double angle) {
        while (angle <= -Math.PI) {
            angle += (2*Math.PI);
        }

        while (angle > Math.PI) {
            angle -= (2*Math.PI);
        }

        return angle;
    }

}
