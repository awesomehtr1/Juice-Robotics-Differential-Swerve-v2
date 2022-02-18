package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.teleopmanager.TeleOpManager;
import org.firstinspires.ftc.teamcode.teleopmanager.TeleOpManagerBuilder;

@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends LinearOpMode {
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2); // creates new robot

        SwerveDrive drive = new SwerveDrive(hardwareMap);

        // teleop managers
        // DRIVER 1
        robot.createTeleOpManager(new TeleOpManagerBuilder() // intake
                .typeToggle(()-> gamepad1.right_bumper)
                .addAction(()-> robot.intake.on())
                .addAction(()-> robot.intake.off())
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder() // slowmode
                .typeToggle(()-> gamepad1.left_trigger > 0.1)
                .addAction(()-> drive.setSlowmode(true))
                .addAction(()-> drive.setSlowmode(false))
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder() // scoring high
                .typeTrigger(()-> gamepad1.left_bumper)
                .addAction(()-> robot.claw.grip())
                .addAction(()-> robot.lift.high())
                .addAction(()-> robot.lift.delayAction(300))
                .addAction(()-> robot.arm.high())
                .addAction(()-> robot.arm.delayAction(300))
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder() // scoring mid
                .typeTrigger(()-> gamepad1.dpad_left)
                .addAction(()-> robot.claw.grip())
                .addAction(()-> robot.lift.mid())
                .addAction(()-> robot.lift.delayAction(300))
                .addAction(()-> robot.arm.mid())
                .addAction(()-> robot.arm.delayAction(300))
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder() // scoring low
                .typeTrigger(()-> gamepad1.dpad_right)
                .addAction(()-> robot.claw.grip())
                .addAction(()-> robot.lift.rest())
                .addAction(()-> robot.lift.delayAction(300))
                .addAction(()-> robot.arm.low())
                .addAction(()-> robot.arm.delayAction(300))
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder() // release/grip cargo
                .typeTrigger(()-> gamepad1.a)
                .addAction(()-> robot.claw.toggleGrip())
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder() // reset for intaking
                .typeTrigger(()-> gamepad1.dpad_down)
                .addAction(()-> robot.claw.timedRetract())
                .addAction(()-> robot.arm.intake())
                .addAction(()-> robot.lift.rest())
                .build());
        robot.createTeleOpManager(new TeleOpManagerBuilder() // duck spinner
                .typeToggle(()-> gamepad1.x)
                .addAction(()-> robot.spinner.on())
                .addAction(()-> robot.spinner.off())
                .build());

        // reset robot
        robot.lift.rest();
        robot.arm.intake();
        robot.claw.grip();
        robot.update();

        waitForStart();
        robot.claw.intake();
        if(isStopRequested()) return;
        while(opModeIsActive()) {
            robot.update();
            double rotation = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            drive.setMotorPowers(rotation, strafe, forward);
        }
    }
}