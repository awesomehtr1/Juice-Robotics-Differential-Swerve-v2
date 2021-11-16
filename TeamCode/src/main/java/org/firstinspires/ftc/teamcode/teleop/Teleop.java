package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.teleopmanager.RobotAction;
import org.firstinspires.ftc.teamcode.teleopmanager.TeleOpManager;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends LinearOpMode {
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2); // creates new robot

        // teleop managers
        // DRIVER 1
        robot.createTeleOpManager(new TeleOpManager(
                ()-> robot.gamepad1.right_trigger > 0.05,
                ()-> robot.intake.on(),
                ()-> robot.intake.off())); // intake toggle
        robot.createTeleOpManager(new TeleOpManager(
                ()-> robot.gamepad1.dpad_up,
                new ArrayList<RobotAction>(Arrays.asList(
                        ()-> robot.lift.high(),
                        ()-> robot.arm.deposit())))); // lift to high and arm to deposit
        robot.createTeleOpManager(new TeleOpManager(
                ()-> robot.gamepad1.dpad_left,
                new ArrayList<RobotAction>(Arrays.asList(
                        ()-> robot.lift.mid(),
                        ()-> robot.arm.deposit())))); // lift to mid  and arm to deposit
        robot.createTeleOpManager(new TeleOpManager(
                ()-> robot.gamepad1.dpad_right,
                new ArrayList<RobotAction>(Arrays.asList(
                        ()-> robot.lift.low(),
                        ()-> robot.arm.deposit())))); // lift to low and arm to deposit
        robot.createTeleOpManager(new TeleOpManager(
                ()-> robot.gamepad1.dpad_down,
                new ArrayList<RobotAction>(Arrays.asList(
                        ()-> robot.lift.rest(),
                        ()-> robot.arm.intake(),
                        ()-> robot.claw.intake())))); // lift to rest and arm/claw to intake
        robot.createTeleOpManager(new TeleOpManager(
                ()-> robot.gamepad1.x,
                ()-> robot.claw.release())); // release cargo
        robot.createTeleOpManager(new TeleOpManager(
                ()-> robot.gamepad1.a,
                ()-> robot.claw.grip())); // grip cargo

        // DRIVER 2
        robot.createTeleOpManager(new TeleOpManager(
                ()-> robot.gamepad2.x,
                ()-> robot.spinner.on(),
                ()-> robot.spinner.off())); // duck spinner toggle

        // reset robot
        robot.lift.rest();
        robot.arm.intake();
        robot.claw.rest();
        robot.update();

        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()){
            robot.drive.drive.setWeightedDrivePower(new Pose2d(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            gamepad1.right_stick_x));
            robot.update();
        }
    }
}