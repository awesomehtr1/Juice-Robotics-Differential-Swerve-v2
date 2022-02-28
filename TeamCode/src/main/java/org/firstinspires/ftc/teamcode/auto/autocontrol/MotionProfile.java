package org.firstinspires.ftc.teamcode.auto.autocontrol;

public class MotionProfile {
    private double maxVelo, maxAccel;
    private double targetVelo, targetAccel;
    private double veloMargin, positionMargin;
    private double prevVelo;

    private enum ACCELSTATE {
        ACCELERATING,
        STEADY,
        DECELERATING
    }
    private ACCELSTATE accelstate;

    /**
     * default constructor for trapezoidal motion profile
     * @param maxVelo tuned max velocity
     * @param maxAccel tuned max acceleration (positive)
     * @param veloMargin margin to compute whether measured velocity == target velocity
     * @param positionMargin margin to compute whether distance to target effectively == 0
     */
    public MotionProfile(double maxVelo, double maxAccel, double veloMargin, double positionMargin) {
        this.maxVelo = maxVelo;
        this.maxAccel = maxAccel;
        this.veloMargin = veloMargin;
        this.positionMargin = positionMargin;
        accelstate = ACCELSTATE.ACCELERATING;
        prevVelo = 0;
    }

    /**
     * update method to compute current target velocity and acceleration
     * @param currentVelo current measured velocity
     * @param targetDistance distance to target pose
     * @param endVelo desired velocity to reach pose at
     * @param deltaTime elapsed time since last update
     * @return double[] velocity, acceleration
     */
    public double[] update(double currentVelo, double targetDistance, double endVelo, double deltaTime) {
        double[] veloAccelState = new double[2];

        // check if robot has to start decelerating
        double veloDiff = currentVelo - endVelo;
        double decelTime = veloDiff / maxAccel;
        // integral of velocity to decelerate
        double decelDistance = (0.5 * veloDiff * decelTime) + (decelTime * endVelo);
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
                targetVelo = prevVelo + (deltaTime * maxAccel);
                break;
            case STEADY:
                targetAccel = 0;
                targetVelo = maxVelo;
                break;
            case DECELERATING:
                targetAccel = -maxAccel;
                targetVelo = Math.sqrt(endVelo * endVelo - (2 * targetAccel * targetDistance));
                break;
        }

        // if effectively at target, set target velo and accel to 0
        if(isRobotAtTarget(targetDistance)) {
            targetVelo = 0;
            targetAccel = 0;
        }

        prevVelo = targetVelo;

        // velocity, acceleration format
        veloAccelState[0] = targetVelo;
        veloAccelState[1] = targetAccel;
        return veloAccelState;
    }

    public boolean isVeloAtTarget(double target, double velo) {
        return Math.abs(target - velo) < veloMargin;
    }

    public boolean isRobotAtTarget(double distance) {
        return Math.abs(distance) < positionMargin;
    }

    public double getTargetVelo() { return targetVelo; }

    public double getTargetAccel() { return targetAccel; }
}