package org.firstinspires.ftc.teamcode.auto.autocontrol;

public class DriveConstants {
    public static final double driveTicksPerRotation = 28.0 / 24.0 * 105.0 / 22.0 * 14.0 * 3.0; // 233.863636364
    public static final double wheelDiameter = 3.07630582136;
    public static final double wheelCircumference = Math.PI * wheelDiameter; // 9.664499768566156
    public static final double driveTicksPerInch = driveTicksPerRotation / wheelCircumference; // 24.19821428571429
    public static final double convergeDistance = 8;
    public static final double admissibleError = 2;
}
