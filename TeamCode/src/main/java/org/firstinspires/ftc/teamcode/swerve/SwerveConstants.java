package org.firstinspires.ftc.teamcode.swerve;

public class SwerveConstants {
    public static final double wheelbase = 0.0; //vertical distance between center of wheels
    public static final double trackwidth = 0.0; //horizontal distance between center of wheels
    public static final double ticksPerRot = 28 * 60 / 30 * 84 / 14; // 28 ticks per rot motor -> 60:30 -> 84:14
    public static final double ticksPerRad = ticksPerRot / (Math.PI * 2);
}
