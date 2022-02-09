package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class Arm implements Subsystem{
    Robot robot;
    Servo armServo1, armServo2;
    static double intakePos = 0.0; // TODO: tune values
    static double depositPos = 0.0;

    // stores current state of arm
    public enum State{
        INTAKE,
        DEPOSIT
    }
    Arm.State state;

    public Arm(Robot robot) {
        this.robot = robot;
        armServo1 = robot.hardwareMap.get(Servo.class, "ArmR"); // TODO: update device name
        armServo2 = robot.hardwareMap.get(Servo.class, "ArmL");
        state = State.INTAKE;
    }

    @Override
    public void update() {
        if(state == State.INTAKE)
            setPos(intakePos);
        if(state == State.DEPOSIT)
            setPos(depositPos);
    }

    // use these methods to change state of arm
    public void intake() { state = State.INTAKE; }
    public void deposit() { state = State.DEPOSIT;}

    public void setPos(double pos) {
        armServo1.setPosition(pos);
        armServo2.setPosition(1 - pos);
    }
}
