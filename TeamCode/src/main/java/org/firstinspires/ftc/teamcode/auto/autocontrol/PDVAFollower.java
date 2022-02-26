package org.firstinspires.ftc.teamcode.auto.autocontrol;

public class PDVAFollower {
    public PDVAController pdvaController;
    private double kP, kD, kV, kA;

    public PDVAFollower(double kP, double kD, double kV, double kA) {
        pdvaController = new PDVAController(kP, kD, kV, kA);
    }
}