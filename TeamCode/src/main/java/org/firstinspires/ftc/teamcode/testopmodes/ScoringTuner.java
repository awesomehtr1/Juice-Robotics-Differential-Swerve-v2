package org.firstinspires.ftc.teamcode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Scoring Tuner", group = "TestOpModes")
public class ScoringTuner extends LinearOpMode {
    public static double liftPos = 0.5;
    public static double armPos = 0.5;
    Servo l1, l2, r1, r2;
    Servo armL, armR;

    public void runOpMode() throws InterruptedException {
        l1 = hardwareMap.get(Servo.class, "liftL1");
        l2 = hardwareMap.get(Servo.class, "liftL2");
        r1 = hardwareMap.get(Servo.class,"liftR1");
        r2 = hardwareMap.get(Servo.class, "liftR2");

        armL = hardwareMap.get(Servo.class,"armL");
        armR = hardwareMap.get(Servo.class, "armR");
        waitForStart();
        while (opModeIsActive()) {
            l1.setPosition(1 -liftPos);
            l2.setPosition(1- liftPos);
            r1.setPosition(liftPos);
            r2.setPosition(liftPos);

            armR.setPosition(armPos);
            armL.setPosition(1 - armPos);
        }
    }
}
