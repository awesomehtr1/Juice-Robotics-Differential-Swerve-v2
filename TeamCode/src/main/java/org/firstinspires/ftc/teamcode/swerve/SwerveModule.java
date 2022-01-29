package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.helperfunctions.AS5600;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.SwerveRotationPID;

public class SwerveModule {
    private DcMotor rot;
    private DcMotor drive;
    private SwerveRotationPID pid;
    private AS5600 as5600;
    private boolean reverseDrive = false;

    private double rotPower;

    // swerve module wrapper for storing drive motor, rotation motor, rotation pid, and analog encoder
    // includes getter and setters
    // handles some low level swerve control
    public SwerveModule(DcMotor rot, DcMotor drive, SwerveRotationPID pid, AS5600 as5600) {
        this.rot = rot;
        this.drive = drive;
        this.pid = pid;
        this.as5600 = as5600;
    }

    // sets PID target angle; takes -pi to pi radian format
    public void setAngle(double angle) { pid.setState(angle); }

    // returns analog encoder angle; returns -pi to pi radian format
    public double getAngle() { return as5600.getAngle(); }

    // updates PID with current angle; returns power
    public double updatePID(double pos) { return pid.updatePID(pos); }

    // sets rot motor power
    public void setRot(double power) {
        rotPower = power;
        rot.setPower(power);
    }

    // sets drive motor power
    public void setDrive(double power) {
        if(reverseDrive)
            drive.setPower(-power);
        else
            drive.setPower(power);
    }

    // reverses drive power direction
    public void setReverseDrive(boolean bool) { reverseDrive = bool; }

    // returns last stored rotation power
    public double getRotPower() { return rotPower; }
}
