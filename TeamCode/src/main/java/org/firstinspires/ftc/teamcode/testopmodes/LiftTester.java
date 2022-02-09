package org.firstinspires.ftc.teamcode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Lift Tester", group = "TestOpModes")
public class LiftTester extends LinearOpMode {
    Servo l1, l2, r1, r2;

    @Override
    public void runOpMode() throws InterruptedException {
        l1 = hardwareMap.get(Servo.class, "l1");
        l2 = hardwareMap.get(Servo.class, "l2");
        r1 = hardwareMap.get(Servo.class, "r1");
        r2 = hardwareMap.get(Servo.class, "r2");
        waitForStart();
        while (opModeIsActive()) {
            l1.setPosition(0);
            l2.setPosition(0);
            r1.setPosition(1);
            r2.setPosition(1);
        }
    }
}
