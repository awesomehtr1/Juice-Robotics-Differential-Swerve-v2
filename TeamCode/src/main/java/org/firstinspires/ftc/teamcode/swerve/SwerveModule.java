package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.helperfunctions.PID.SwerveRotationPID;

public class SwerveModule {
    public DcMotorEx rot;
    public DcMotor drive;
    public SwerveRotationPID pid;

    public SwerveModule(DcMotorEx rot, DcMotor drive, SwerveRotationPID pid) {
        this.rot = rot;
        this.drive = drive;
        this.pid = pid;
    }
}
