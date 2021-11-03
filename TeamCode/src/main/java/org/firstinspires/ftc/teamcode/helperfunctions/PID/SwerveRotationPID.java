package org.firstinspires.ftc.teamcode.helperfunctions.PID;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SwerveRotationPID extends BasicPID {
    public SwerveRotationPID(double kP, double kI, double kD, double kS, ElapsedTime time){
        super(kP, kI, kD, kS, time);
    }

    @Override
    public boolean shouldIntegralBeZeroed(double error, double desiredState) {
        if (desiredState != lastState) {
            lastState = desiredState;
            return true;
        }
        lastState = desiredState;
        return false;
    }

    @Override
    public boolean incrementIntegral(double power) {
        // prevents integral incrementation if power >= 1 or integral sum > 0.3
        return !(Math.abs(power) >= 1 || Math.abs(integral) > 0.3);
    }
}
