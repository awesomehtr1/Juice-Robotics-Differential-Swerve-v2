package org.firstinspires.ftc.teamcode.helperfunctions.PID;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperfunctions.PID.BasicPID;

public class SwerveRotationPID extends BasicPID {
    public SwerveRotationPID(double kP, double kI, double kD, double kS, ElapsedTime time){
        super(kP, kI, kD, kS, time);
    }

    @Override
    public boolean shouldIntegralBeZeroed(double error, double power) {
        return error == 0;
    }
}
