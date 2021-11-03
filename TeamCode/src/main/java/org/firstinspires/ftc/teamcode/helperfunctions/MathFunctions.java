package org.firstinspires.ftc.teamcode.helperfunctions;

public class MathFunctions {
    // normalizes the angle to -pi to pi
    public static double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
    // normalizes the angle to -pi/2 to pi/2
    public static double angleWrap180(double radians) {
        double normalized = angleWrap(radians);
        if (Math.abs(normalized) > Math.PI/2)
            normalized -= Math.PI;
        return normalized;
    }
}
