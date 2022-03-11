package org.firstinspires.ftc.teamcode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Gamepad Lift Tuner", group = "TestOpModes")
public class GamepadLiftTuner extends LinearOpMode {
    double pos = 0.5;
    public boolean track = false;
    Servo L1, L2, R1, R2;

    public void runOpMode() throws InterruptedException {
        R1 = hardwareMap.get(Servo.class, "liftR1");
        R2 = hardwareMap.get(Servo.class, "liftR2");
        L1 = hardwareMap.get(Servo.class, "liftL1");
        L2 = hardwareMap.get(Servo.class, "liftL2");

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a)
                track = true;
            if(gamepad1.b)
                track = false;

            if(gamepad1.dpad_up)
                pos += 0.001;
            if(gamepad1.dpad_down)
                pos -= 0.001;

            L1.setPosition(pos);
            L2.setPosition(pos);
            R1.setPosition(1 - pos);
            R2.setPosition(1 - pos);

            telemetry.addData("tracking: ", track);
            telemetry.addData("position: ", pos);
            telemetry.update();
        }
    }
}
