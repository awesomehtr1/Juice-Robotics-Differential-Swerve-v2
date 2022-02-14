package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class Arm implements Subsystem{
    Robot robot;
    Servo armServo1, armServo2;
    final double intakePos = 0.0; // TODO: tune values
    final double depositLowPos = 0.0;
    final double depositMidPos = 0.0;
    final double depositHighPos = 0.0;

    // stores current state of arm
    public enum State{
        INTAKE,
        DEPOSITLOW,
        DEPOSITMID,
        DEPOSITHIGH
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
        if(state == State.DEPOSITLOW)
            setPos(depositLowPos);
        if(state == State.DEPOSITMID)
            setPos(depositLowPos);
        if(state == State.DEPOSITHIGH)
            setPos(depositLowPos);
    }

    // use these methods to change state of arm
    public void intake() { state = State.INTAKE; }
    public void depositLow() { state = State.DEPOSITLOW;}
    public void depositMid() { state = State.DEPOSITMID;}
    public void depositHigh() { state = State.DEPOSITHIGH;}

    public void setPos(double pos) {
        armServo1.setPosition(pos);
        armServo2.setPosition(1 - pos);
    }
}
