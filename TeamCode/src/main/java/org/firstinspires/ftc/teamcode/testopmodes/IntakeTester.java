package org.firstinspires.ftc.teamcode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Intake Test", group = "TestOpModes")
public class IntakeTester extends LinearOpMode {
    public static double power = 1.0;
    CRServo l1, l2, r1, r2;

    @Override
    public void runOpMode() throws InterruptedException {
        l1 = hardwareMap.get(CRServo.class, "intakeL1");
        l2 = hardwareMap.get(CRServo.class, "intakeL2");
        r1 = hardwareMap.get(CRServo.class, "intakeR1");
        r2 = hardwareMap.get(CRServo.class, "intakeR2");
        waitForStart();
        while(opModeIsActive()) {
            l1.setPower(gamepad1.left_stick_y);
            l2.setPower(-gamepad1.left_stick_y);
            r1.setPower(-gamepad1.left_stick_y);
            r2.setPower(gamepad1.left_stick_y);
        }
    }
}
