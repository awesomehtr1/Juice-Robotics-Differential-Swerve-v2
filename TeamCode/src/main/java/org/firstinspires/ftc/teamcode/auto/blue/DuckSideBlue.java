package org.firstinspires.ftc.teamcode.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.RemoteAuto;
import org.firstinspires.ftc.teamcode.auto.autocontrol.DriveConstants;
import org.firstinspires.ftc.teamcode.auto.autocontrol.RunToPosition;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline;

@Autonomous(name = "Blue Duck Side", group = "Auto")
public class DuckSideBlue extends LinearOpMode {
    Robot robot;
    RunToPosition drive;
    Vision vision;

    ElapsedTime currentTime;
    double time;

    public enum LEVEL {
        HIGH,
        MID,
        LOW
    }
    RemoteAuto.LEVEL level;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, gamepad1, gamepad2);
        drive = new RunToPosition(
                hardwareMap,
                DriveConstants.drivePIDConstants,
                DriveConstants.rotationPIDconstants);
        drive.setStartPose(-36, 64, Math.toRadians(90));
        drive.setTargetHeading(Math.toRadians(90));

        vision = new Vision(hardwareMap, telemetry);
        vision.setPipeline();
        vision.startStreaming();
        currentTime = new ElapsedTime();

        // run vision and update enum
        while(!opModeIsActive() && !isStopRequested()) {
            VisionPipeline.POS pos = vision.getPosition();
            if(pos == VisionPipeline.POS.RIGHT)
                level = RemoteAuto.LEVEL.HIGH;
            else if(pos == VisionPipeline.POS.CENTER)
                level = RemoteAuto.LEVEL.MID;
            else if(pos == VisionPipeline.POS.LEFT)
                level = RemoteAuto.LEVEL.LOW;
        }
        if(isStopRequested()) return;
        waitForStart();
        vision.closeCamera();
        currentTime.reset();
        drive.resetTimers();

        // AUTO CODE
        robot.claw.grip();
        robot.lift.rest();
        robot.arm.intake();
        robot.update();

        // SCORE PRELOAD
        timeout(0.25);

        // RANDOMIZATION SWITCH CASE
        switch (level) {
            case LOW:
                robot.arm.low();
                robot.lift.rest();
                drive.setTargetPoint(-34, 24);
                break;
            case MID:
                robot.arm.mid();
                robot.lift.mid();
                drive.setTargetPoint(-34, 24);
                break;
            case HIGH:
                robot.arm.high();
                robot.lift.high();
                drive.setTargetPoint(-33, 24);
                break;
        }
        runDrive();
        timeout(0.5);
        drive.setTargetHeading(Math.toRadians(0));
        runDrive();
        robot.claw.deposit();
        timeout(0.75);

        // RESET
        drive.setTargetPoint(-38, 23);
        runDrive();
        robot.claw.timedRetract();
        robot.arm.intake();
        robot.lift.rest();
        timeout(0.5);

        // SPIN DUCK
        drive.setTargetHeading(Math.toRadians(30));
        runDrive();
        drive.setTargetPoint(-55, 55);
        runDrive();
        drive.forwardByTime(0.15, 4);
        robot.spinner.on();
        timeout(4);
        robot.spinner.off();

        // INTAKE DUCK
        robot.intake.on();
        drive.setTargetPoint(-52, 52);
        runDrive();
        drive.setTargetHeading(Math.toRadians(90));
        runDrive();
        drive.setTargetPoint(-52, 62);
        runDrive();
        drive.strafeByTime(0.25, 2);
        runDrive();
        robot.intake.off();
        robot.claw.grip();

        // SCORE DUCK
        drive.setTargetPoint(-34, 23);
        runDrive();
        drive.setTargetHeading(Math.toRadians(0));
        runDrive();
        robot.lift.high();
        robot.arm.high();
        timeout(0.75);
        robot.claw.deposit();
        timeout(0.5);
        robot.claw.timedRetract();
        robot.arm.intake();
        robot.lift.rest();

        // PARK
        drive.setTargetPoint(-60, 35);
        runDrive();
        timeout(1);
    }

    public void runDrive() {
        drive.update();
        while(!drive.isReachedTarget() && opModeIsActive()) {
            drive.update();
            robot.update();
        }
    }

    public void timeout(double seconds) {
        currentTime.reset();
        this.time = seconds;
        while(isTimedOut() && opModeIsActive()){
            robot.update();
            drive.update();
        }
    }

    public boolean isTimedOut() { return currentTime.seconds() < time; }
}