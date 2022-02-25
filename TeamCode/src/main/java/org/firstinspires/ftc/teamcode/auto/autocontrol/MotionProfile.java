package org.firstinspires.ftc.teamcode.auto.autocontrol;

public class MotionProfile {
    private double maxVelo, maxAccel;
    private double targetVelo, targetAccel;
    private double veloMargin;

    private enum ACCELSTATE {
        ACCELERATING,
        STEADY,
        DECELERATING
    }
    private ACCELSTATE accelstate;

    public MotionProfile(double maxVelo, double maxAccel, double veloMargin) {
        this.maxVelo = maxVelo;
        this.maxAccel = maxAccel;
        this.veloMargin = veloMargin;
        accelstate = ACCELSTATE.ACCELERATING;
    }

    public double[] update(double currentVelo, double targetDistance, double endVelo) {
        double[] veloAccelState = new double[2];

        // check if robot has to start decelerating
        double decelTime = (currentVelo - endVelo) / maxAccel;
        double decelDistance = (0.5 * decelTime * decelTime) + (decelTime * endVelo); // integral of velocity to decelerate
        if(decelDistance >= targetDistance)
            accelstate = ACCELSTATE.DECELERATING;

        // check if robot should stay at 0 accel (max velo)
        else if(isVeloAtTarget(currentVelo, maxVelo))
            accelstate = ACCELSTATE.STEADY;

        // else robot should accelerate to max velo
        else
            accelstate = ACCELSTATE.ACCELERATING;

        // switch case to determine target velocity and acceleration value
        switch (accelstate) {
            case ACCELERATING:
                targetAccel = maxAccel;
                targetVelo = Math.sqrt(endVelo * endVelo - (2 * targetAccel * targetDistance));
                break;
            case STEADY:
                targetVelo = maxVelo;
                targetAccel = 0;
                break;
            case DECELERATING:
                targetAccel = -maxAccel;
                targetVelo = Math.sqrt(endVelo * endVelo - (2 * targetAccel * targetDistance));
                break;
        }

        veloAccelState[0] = targetVelo;
        veloAccelState[1] = targetAccel;
        return veloAccelState;
    }

    public boolean isVeloAtTarget(double target, double velo) {
        return Math.abs(target - velo) < veloMargin;
    }
}
