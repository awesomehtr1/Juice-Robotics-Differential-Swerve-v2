package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.helperfunctions.PID.SwerveRotationPID;

public class SwerveModule {
    private DcMotorEx rot;
    private DcMotor drive;
    private SwerveRotationPID pid;

    public SwerveModule(DcMotorEx rot, DcMotor drive, SwerveRotationPID pid) {
        this.rot = rot;
        this.drive = drive;
        this.pid = pid;
    }

    public void setAngle(double angle) { pid.setState(angle); }

    public double getAngle() { return rot.getCurrentPosition(); }

    public double updatePID(double pos) { return pid.updatePID(pos); }

    public void setRot(double power) { rot.setPower(power); }

    public void setDrive(double power) { drive.setPower(power); }
}
