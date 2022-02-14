package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class Intake implements Subsystem{
    Robot robot;
    CRServo servoR1, servoR2, servoL1, servoL2;

    // stores current state of intake
    enum State {
        OFF,
        ON,
    }
    Intake.State state;

    public Intake(Robot robot){
        this.robot = robot;
        servoR1 = robot.hardwareMap.get(CRServo.class, ""); // TODO: update device name
        servoR2 = robot.hardwareMap.get(CRServo.class, "");
        servoL1 = robot.hardwareMap.get(CRServo.class, "");
        servoL2 = robot.hardwareMap.get(CRServo.class, "");
        state = State.OFF;
    }

    @Override
    public void update() {
        if(state == State.OFF)
            setPower(0);
        if(state == State.ON)
            setPower(1);
    }

    // use these methods to set the state of the intake
    public void off(){
        state = State.OFF;
    }
    public void on(){ state = State.ON; }

    public void setPower(double power){
        // TODO: change power direction if needed
        servoL1.setPower(power);
        servoL2.setPower(-power);
        servoR1.setPower(-power);
        servoR2.setPower(power);
    }
}
