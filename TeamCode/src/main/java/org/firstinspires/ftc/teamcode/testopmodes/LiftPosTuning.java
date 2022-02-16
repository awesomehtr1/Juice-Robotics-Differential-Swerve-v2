package org.firstinspires.ftc.teamcode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Lift Pos Tuning", group = "TestOpModes")
public class LiftPosTuning extends LinearOpMode {
    public static double pos = 0.5;
    Servo l1, l2, r1, r2;

    public void runOpMode() throws InterruptedException {
        l1 = hardwareMap.get(Servo.class, "l1");
        l2 = hardwareMap.get(Servo.class, "l2");
        r1 = hardwareMap.get(Servo.class, "r1");
        r2 = hardwareMap.get(Servo.class, "r2");
        waitForStart();
        while (opModeIsActive()) {
            // r and left need to swapped for testing (right 0/1; left 2/3)
            l1.setPosition(pos);
            l2.setPosition(pos);
            r1.setPosition(1 - pos);
            r2.setPosition(1 - pos);
        }
    }
}
