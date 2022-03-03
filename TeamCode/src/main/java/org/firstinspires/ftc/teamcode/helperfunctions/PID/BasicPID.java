package org.firstinspires.ftc.teamcode.helperfunctions.PID;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BasicPID {
    public double desiredState, currentState, lastState; // setState, currentState, stateMemory
    public double kP, kI, kD; // PID coefficients
    public double kS; // static coefficient
    public double integral; // current integral
    public double error;
    public boolean firstSetStateLoop, firstGetOutputLoop;
    public double prevTime, prevError, startTime; // last loop + time
    public ElapsedTime time;
    private double power;

    public BasicPID(double kP, double kI, double kD, double kS, ElapsedTime time){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.time = time;
        integral = 0;
        lastState = 100000;
        firstGetOutputLoop = true;
        firstSetStateLoop = true;
    }

    public double updatePID(double currentState){
        double currentTime = time.milliseconds();
        if(firstGetOutputLoop){
            firstGetOutputLoop = false;
            prevTime = currentTime;
            startTime = prevTime;
            prevError = 0;
        }

        double deltaTime = (currentTime - prevTime)/1000;
        error = calculateError(currentState);
        double deltaError = error - prevError;
        prevTime = currentTime;
        prevError = error;

        double derivative = 0;
        if(deltaTime != 0) {
            derivative = deltaError / deltaTime;
        }
        this.currentState = currentState;
        power = kP * error + Math.abs(kI * integral) * Math.signum(error) + kD * derivative;
        if(shouldIntegralBeZeroed(error, desiredState))
            clearIntegral();
        if (incrementIntegral(power))
            integral += error * deltaTime;
        if (power != 0)
            return power + (Math.signum(power) * kS);
        return power;
    }

    public double getPower() { return power; }

    public double calculateError(double currentState) { return desiredState - currentState; }

    // sets target state of PID
    public void setState(double desiredState){
        this.desiredState = desiredState;
    }

    // to be overridden in child classes (anti-windup method)
    public boolean shouldIntegralBeZeroed(double error, double desiredState){
        return false;
    }

    // to be overridden in child classes (anti-windup method)
    public boolean incrementIntegral(double power){
        return true;
    }

    // clears integral to avoid windup
    public void clearIntegral(){
        integral = 0;
    }
}
