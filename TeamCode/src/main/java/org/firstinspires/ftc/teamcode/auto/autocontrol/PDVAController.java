package org.firstinspires.ftc.teamcode.auto.autocontrol;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PDVAController {
    private double kP, kD, kV, kA;
    private ElapsedTime time;

    private double target, state;
    private double velocity, acceleration;
    private double prevTime;
    private double prevError, error;
    private double power;

    private boolean firstLoop;

    /**
     * default constructor for PDVA controller
     * @param kP P constant
     * @param kD D constant
     * @param kV velocity feedforward constant
     * @param kA acceleration feedforward constant
    */
    public PDVAController(double kP, double kD, double kV, double kA) {
        this.kP = kP;
        this.kD = kD;
        this.kV = kV;
        this.kA = kA;

        time = new ElapsedTime();
        firstLoop = true;
    }

    /**
     * updates PDVA output power and current state
     * @param state current system state
     * @param velocity current system target velocity
     * @param acceleration current system target acceleration
     */
    public void update(double state, double velocity, double acceleration) {
        double currentTime = time.milliseconds();

        if(firstLoop){
            firstLoop = false;
            prevTime = currentTime;
            prevError = 0;
        }

        this.state = state;
        this.velocity = velocity;
        this.acceleration = acceleration;

        error = target - state;

        // compute P
        double P = error * kP;

        // compute D
        double deltaTime = (currentTime - prevTime) / 1000;
        double deltaError = error - prevError;
        double D = deltaError / deltaTime * kD;

        // compute feedforward
        double V = velocity * kV;
        double A = acceleration * kA;

        // sum powers
        power = P + D + V + A;

        // prepare for next loop
        prevError = error;
        prevTime = currentTime;
    }

    public void setTarget(double target) { this.target = target; }

    public double getOutput() { return power; }

    public void setkP(double kP) { this.kP = kP; }
    public void setkD(double kD) { this.kD = kD; }
    public void setkV(double kV) { this.kV = kV; }
    public void setkA(double kA) { this.kA = kA; }
}
