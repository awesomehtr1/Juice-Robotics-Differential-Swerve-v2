package org.firstinspires.ftc.teamcode.swerve;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.auto.autocontrol.DriveConstants;

public class SwerveOdometry {
    // x, y, and heading of the robot (FIELD CENTRIC)
    double x, y, heading;

    // x and y velocities of the robot (ROBOT CENTRIC)
    double vx, vy;
    double prevVx = 0;
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
        double robotCentricDeltaX = (vx + prevVx) * 0.5 * elapsedTime;
        double robotCentricDeltaY = (vy + prevVy) * 0.5 * elapsedTime;
        double deltax = (robotCentricDeltaX * Math.cos(heading)) - (robotCentricDeltaY * Math.sin(heading));
        double deltay = (robotCentricDeltaX * Math.sin(heading) + (robotCentricDeltaY * Math.cos(heading)));
        deltay /= DriveConstants.driveTicksPerInch;
        deltay /= DriveConstants.driveTicksPerInch;
        prevVx = vx;
        prevVy = vy;
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
        double deltax = (vx + prevVx) * 0.5 * elapsedTime;
        double deltay = (vy + prevVy) * 0.5 * elapsedTime;
        double deltaheading = heading - this.heading;

        // pose exponential matrix math
        SimpleMatrix deltas = new SimpleMatrix(3, 1);
        deltas.set(0, 0, deltax);
        deltas.set(1, 0, deltay);
        deltas.set(2, 0, deltaheading);

        SimpleMatrix poseExponential = new SimpleMatrix(3, 3);
        poseExponential.set(0, 0, Math.sin(deltaheading) / deltaheading);
        poseExponential.set(0, 1, -(1 - Math.cos(deltaheading) / deltaheading));
        poseExponential.set(0, 2, 0.0);

        poseExponential.set(1, 0, 1 - Math.cos(deltaheading) / deltaheading);
        poseExponential.set(1, 1, Math.sin(deltaheading) / deltaheading);
        poseExponential.set(1, 2, 0.0);

        poseExponential.set(2, 0, 0.0);
        poseExponential.set(2, 1, 0.0);
        poseExponential.set(2, 2, 1.0);

        SimpleMatrix rotation = new SimpleMatrix(3, 3);
        rotation.set(0, 0, Math.cos(heading));
        rotation.set(0, 1, -Math.sin(heading));
        rotation.set(0, 2, 0.0);

        rotation.set(1, 0, Math.sin(heading));
        rotation.set(1, 1, Math.cos(heading));
        rotation.set(1, 2, 0.0);

        rotation.set(2, 0, 0.0);
        rotation.set(2, 1, 0.0);
        rotation.set(2, 2, 1.0);

        // matrix multiplication
        SimpleMatrix poseDelta = rotation.mult(poseExponential).mult(deltas);

        double deltaX = (double) poseDelta.get(0, 0);
        double deltaY = (double) poseDelta.get(1, 0);
        deltaX /= DriveConstants.driveTicksPerInch;
        deltaY /= DriveConstants.driveTicksPerInch;
        x += deltaX;
        y += deltaY;
        this.heading = heading;
        prevVx = vx;
        prevVy = vy;
    }

    // get current pose broken down into x, y, and heading
    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    // set/reset pose
    public void setPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    // get current velocity
    public double getVx() { return vx; }
    public double getVy() { return vy; }
    public double getV() { return Math.sqrt((vx*vx)+(vy*vy)); }

    // functions to set wheelbase and trackwidth measurements
    public void setWheelbase(double wheelbase) { this.wheelbase = wheelbase; }
    public void setTrackwidth(double trackwidth) { this.trackwidth = trackwidth; }
}