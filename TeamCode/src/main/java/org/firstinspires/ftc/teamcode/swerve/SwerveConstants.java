package org.firstinspires.ftc.teamcode.swerve;

public class SwerveConstants {
    public static final double wheelbase = 2 * Math.sqrt(0.5);
    public static final double trackwidth = 2 * Math.sqrt(0.5);
    public static final double ticksPerRot = 28 * 60 / 30 * 84 / 14; // 28 ticks per rot motor -> 60:30 -> 84:14
    public static final double ticksPerRad = ticksPerRot / (Math.PI * 2);
    public static final double wheelRotPerPodRot = 22.0 / 14.0 / 3.0; // 0.52380952381 per pod rotation
    public static final double driveSpeed = 6000.0 * 24 / 105 * 22 / 14 / 3; // 718.367346939 rpm
    public static final double rotationSpeed = 6000.0 / 2 * 16 / 84; // 571.428571429 rpm
    public static final double driveRotRatio = driveSpeed / rotationSpeed; // 1.25714285714 drive rpm : rot rpm
}
