package org.firstinspires.ftc.teamcode.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.RemoteAuto;
import org.firstinspires.ftc.teamcode.auto.autocontrol.DriveConstants;
import org.firstinspires.ftc.teamcode.auto.autocontrol.RunToPosition;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline;

@Autonomous(name = "Red Duck Side", group = "Auto")
public class DuckSideRed extends LinearOpMode {
    // Robot robot;
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
//        robot = new Robot(hardwareMap, gamepad1, gamepad2);
        drive = new RunToPosition(
                hardwareMap,
                DriveConstants.drivePIDConstants,
                DriveConstants.rotationPIDconstants);
        drive.setStartPose(-36, -64, Math.toRadians(-90));
        drive.setTargetHeading(Math.toRadians(-90));

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
//        robot.claw.grip();
//        robot.lift.rest();
//        robot.arm.intake();
//        robot.update();

        // SCORE PRELOAD
        drive.setTargetPoint(-33, -23);
        runDrive();
        drive.setTargetPoint(-36, -60);
        runDrive();
//        drive.setTargetHeading(Math.toRadians(-180));
//        runDrive();
//
//        // SPIN DUCK
//        drive.setTargetHeading(Math.toRadians(-125));
//        runDrive();
//        drive.setTargetPoint(-55, -55);
//        runDrive();
//
//        // INTAKE DUCK
//        drive.setTargetPoint(-52, -52);
//        runDrive();
//        drive.setTargetHeading(Math.toRadians(-90));
//        runDrive();
//        drive.setTargetPoint(-52, -60);
//        runDrive();
//        drive.setTargetPoint(-40, -60);
//        runDrive();
//
//        // SCORE DUCK
//        drive.setTargetPoint(-33, -23);
//        runDrive();
//        drive.setTargetHeading(Math.toRadians(-180));
//        runDrive();
//
//        // PARK
//        drive.setTargetPoint(-60, -35);
//        runDrive();
    }

    public void runDrive() {
        drive.update();
        while(!drive.isReachedTarget() && opModeIsActive()) {
            drive.update();
//            robot.update();
        }
    }

    public void timeout(double seconds) {
        currentTime.reset();
        this.time = seconds;
        while(isTimedOut() && opModeIsActive()){
//            robot.update();
            drive.update();
        }
    }

    public boolean isTimedOut() { return currentTime.seconds() < time; }
}
