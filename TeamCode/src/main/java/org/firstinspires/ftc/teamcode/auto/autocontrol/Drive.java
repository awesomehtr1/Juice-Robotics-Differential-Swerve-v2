package org.firstinspires.ftc.teamcode.auto.autocontrol;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helperfunctions.SanfordGyro;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;

public class Drive {
    SwerveDrive swerveDrive;
    SanfordGyro gyro;

    public Drive(HardwareMap hardwareMap) {
        swerveDrive = new SwerveDrive(hardwareMap);
        gyro = new SanfordGyro(hardwareMap);
    }

    public double getAngle() { return gyro.getAngleCorrected(); }
}
