package org.firstinspires.ftc.teamcode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Lift Tester", group = "TestOpModes")
public class LiftTester extends LinearOpMode {
    Servo l1, l2, r1, r2;
    public static double pos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        l1 = hardwareMap.get(Servo.class, "liftL1");
        l2 = hardwareMap.get(Servo.class, "liftL2");
        r1 = hardwareMap.get(Servo.class, "liftR1");
        r2 = hardwareMap.get(Servo.class, "liftR2");
        waitForStart();
        while (opModeIsActive()) {
            l1.setPosition(1 - pos);
            l2.setPosition(1 - pos);
            r1.setPosition(pos);
            r2.setPosition(pos);
        }
    }
}
