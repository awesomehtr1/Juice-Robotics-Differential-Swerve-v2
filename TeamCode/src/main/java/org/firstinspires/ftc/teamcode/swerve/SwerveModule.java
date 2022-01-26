package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.helperfunctions.AS5600;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.SwerveRotationPID;

public class SwerveModule {
    private DcMotor rot;
    private DcMotor drive;
    private SwerveRotationPID pid;
    private AS5600 as5600;
    private boolean reverseDrive;

    public SwerveModule(DcMotor rot, DcMotor drive, SwerveRotationPID pid, AS5600 as5600) {
        this.rot = rot;
        this.drive = drive;
        this.pid = pid;
        this.as5600 = as5600;
    }

    public void setAngle(double angle) { pid.setState(angle); }

    public double getAngle() { return as5600.getAngle(); }

    public double updatePID(double pos) { return pid.updatePID(pos); }

    public void setRot(double power) { rot.setPower(power); }

    public void setDrive(double power) {
        if(reverseDrive)
            drive.setPower(-power);
        else
            drive.setPower(power);
    }

    public void setReverseDrive(boolean bool) { reverseDrive = bool; }
}
