package org.firstinspires.ftc.teamcode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Claw Test", group = "TestOpModes")
public class ClawTuner extends LinearOpMode {
    Servo claw;
    public static double pos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class, "claw");
        waitForStart();
        while(opModeIsActive()) {
            claw.setPosition(pos);
        }
    }
}
