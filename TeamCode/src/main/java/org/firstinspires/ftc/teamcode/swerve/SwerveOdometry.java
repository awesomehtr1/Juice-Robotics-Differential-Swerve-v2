package org.firstinspires.ftc.teamcode.swerve;

import org.ejml.simple.SimpleMatrix;

public class SwerveOdometry {
    // x, y, and heading of the robot (FIELD CENTRIC)
    double x, y, heading;

    // x and y velocities of the robot (ROBOT CENTRIC)
    double vx, vy;
    double prevVX = 0;
    double prevVy = 0;

    // trackwidth: horizontal distance between center of pairs of wheels
    // wheelbase: vertical distance between center of pairs of wheels
    private double trackwidth, wheelbase;

    /**
     * inverse kinematics method to return robot centric x and y velocity vectors based
     * on vectors of individual wheels
     * @param wheelVelocities
     * @param moduleOrientations
     */
    public void inverseKinematics(double[] wheelVelocities, double[] moduleOrientations) {
        double RFx, RFy, LFx, LFy, LBx, LBy, RBx, RBy;
        RFx = wheelVelocities[0] * Math.cos(moduleOrientations[0]);
        RFy = wheelVelocities[0] * Math.sin(moduleOrientations[0]);

        LFx = wheelVelocities[1] * Math.cos(moduleOrientations[1]);
        LFy = wheelVelocities[1] * Math.sin(moduleOrientations[1]);

        LBx = wheelVelocities[2] * Math.cos(moduleOrientations[2]);
        LBy = wheelVelocities[2] * Math.sin(moduleOrientations[2]);

        RBx = wheelVelocities[3] * Math.cos(moduleOrientations[3]);
        RBy = wheelVelocities[3] * Math.sin(moduleOrientations[3]);

        vx = (RFx + LFx + LBx + RBx) / 4.0;
        vy = (RFy + LFy + LBy + RBy) / 4.0;
    }

    /**
     * method to update pose; takes current heading, wheel vectors, and elapsed time
     * to compute distance travelled and add that to the previous pose estimate
     * @param wheelVelocities
     * @param moduleOrientations
     * @param heading
     * @param elapsedTime
     */
    public void updatePose(double[] wheelVelocities, double[] moduleOrientations, double heading, double elapsedTime) {
        inverseKinematics(wheelVelocities, moduleOrientations);
        this.heading = heading;
        // matrix square rotation equations
        double deltax = (vx * Math.cos(heading)) + (vy * Math.sin(heading));
        double deltay = (vx * Math.sin(heading) + (vy * Math.cos(heading)));
        deltax *= elapsedTime;
        deltay *= elapsedTime;
        x += deltax;
        y += deltay;
    }

    /**
     * method to update pose using pose exponential vector math; takes current heading,
     * wheel vectors, and elapsed time to compute distance travelled and add that to the
     * previous pose estimate
     * @param wheelVelocities
     * @param moduleOrientations
     * @param heading
     * @param elapsedTime
     */
    public void updatePoseExponential(double[] wheelVelocities, double[] moduleOrientations, double heading, double elapsedTime) {
        inverseKinematics(wheelVelocities, moduleOrientations);

        // ROBOT CENTRIC deltas
        double deltax = (vx + prevVX) * 0.5 * elapsedTime;
        double deltay = (vy + prevVy) * 0.5 * elapsedTime;
        double deltaheading = heading - this.heading;

        // pose exponential matrix math
        SimpleMatrix deltas = new SimpleMatrix(3, 1);
        deltas.set(1, 1, deltax);
        deltas.set(2, 1, deltay);
        deltas.set(3, 1, deltaheading);

        SimpleMatrix poseExponential = new SimpleMatrix(3, 3);
        poseExponential.set(1, 1, Math.sin(deltaheading) / deltaheading);
        poseExponential.set(1, 2, -(1 - Math.cos(deltaheading) / deltaheading));
        poseExponential.set(1, 3, 0.0);

        poseExponential.set(2, 1, 1 - Math.cos(deltaheading) / deltaheading);
        poseExponential.set(2, 2, Math.sin(deltaheading) / deltaheading);
        poseExponential.set(2, 3, 0.0);

        poseExponential.set(3, 1, 0.0);
        poseExponential.set(3, 2, 0.0);
        poseExponential.set(3, 3, 1.0);

        SimpleMatrix rotation = new SimpleMatrix(3, 3);
        rotation.set(1, 1, Math.cos(heading));
        rotation.set(1, 2, -Math.sin(heading));
        rotation.set(1, 3, 0.0);

        rotation.set(2, 1, Math.sin(heading));
        rotation.set(2, 2, Math.cos(heading));
        rotation.set(2, 3, 0.0);

        rotation.set(3, 1, 0.0);
        rotation.set(3, 2, 0.0);
        rotation.set(3, 3, 1.0);

        // matrix multiplication
        SimpleMatrix poseDelta = rotation.mult(poseExponential).mult(deltas);

        x += poseDelta.get(1, 1);
        y += poseDelta.get(2, 1);
        this.heading = heading;
    }

    // get current pose broken down into x, y, and heading
    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    // set/reset pose
    public void setPose(double x, double y) {
        this.x = x;
        this.y = y;
    }

    // functions to set wheelbase and trackwidth measurements
    public void setWheelbase(double wheelbase) { this.wheelbase = wheelbase; }
    public void setTrackwidth(double trackwidth) { this.trackwidth = trackwidth; }
}