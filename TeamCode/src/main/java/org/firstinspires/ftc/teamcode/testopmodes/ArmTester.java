package org.firstinspires.ftc.teamcode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Arm Tester", group = "TestOpModes")
public class ArmTester extends LinearOpMode {
    Servo servo1, servo2;
    double setPosition;
    public static double offset;
    boolean track = false;
    boolean toggleSides = false;

    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(Servo.class, "ArmR");
        servo2 = hardwareMap.get(Servo.class, "ArmL");
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a)
                track = true;
            if(gamepad1.b)
                track = false;
            if(gamepad1.x)
                toggleSides = true;
            if(gamepad1.y)
                toggleSides = false;
            setPosition = gamepad1.left_stick_y;
            if(track) {
                if(!toggleSides) {
                    servo1.setPosition(setPosition + offset);
                    servo2.setPosition(1 - setPosition);
                }
                else{
                    servo1.setPosition(1 - setPosition);
                    servo2.setPosition(setPosition + offset);
                }
            }
            else {
                servo1.setPosition(offset);
                servo2.setPosition(1);
            }
        }
    }
}
