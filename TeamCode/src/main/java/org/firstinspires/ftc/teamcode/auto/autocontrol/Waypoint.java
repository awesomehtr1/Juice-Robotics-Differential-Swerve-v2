package org.firstinspires.ftc.teamcode.auto.autocontrol;

public class Waypoint {
    public double x, y, heading, endVelocity;

    public Waypoint(double x, double y, double heading, double endVelocity) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.endVelocity = endVelocity;
    }
}
