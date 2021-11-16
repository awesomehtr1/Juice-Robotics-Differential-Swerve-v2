package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Robot;

public class Spinner implements Subsystem{
    Robot robot;
    CRServo wheelServo;
    public static double power = 1;

    // stores current state of spinner
    enum State {
        OFF,
        ON,
    }
    Spinner.State state;

    public Spinner(Robot robot) {
        this.robot = robot;
        wheelServo = robot.hardwareMap.get(CRServo.class, ""); // TODO: update device name
        state = State.OFF;
    }

    @Override
    public void update() {
        if(state == State.OFF)
            wheelServo.setPower(0);
        if(state == State.ON)
            wheelServo.setPower(power);
    }

    // updates state of spinner wheel
    public void off() { state = State.OFF; }
    public void on() { state = State.ON; }

    // use this method to switch to blue/red
    public void setPower(double power) { this.power = power; }
}
