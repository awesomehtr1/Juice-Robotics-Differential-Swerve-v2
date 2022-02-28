package org.firstinspires.ftc.teamcode.swerve;

import org.ejml.simple.SimpleMatrix;

public class SwerveOdometry {
    // x, y, and heading of the robot
    double x, y, heading;

    // x and y velocities of the robot
    double vx, vy;

    // trackwidth: horizontal distance between center of pairs of wheels
    // wheelbase: vertical distance between center of pairs of wheels
    private double trackwidth, wheelbase;

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

    public void updatePose(double[] wheelVelocities, double[] moduleOrientations, double heading, double elapsedTime) {
        inverseKinematics(wheelVelocities, moduleOrientations);
        this.heading = heading;
        x = (vx * Math.cos(heading)) + (vy * Math.sin(heading));
        y = (vx * Math.sin(heading) + (vy * Math.cos(heading)));
        x *= elapsedTime;
        y *= elapsedTime;
    }

    public void updatePoseExponential(double[] wheelVelocities, double[] moduleOrientations, double heading, double elapsedTime) {

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