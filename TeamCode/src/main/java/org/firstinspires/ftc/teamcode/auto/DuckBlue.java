package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.teleop.Teleop;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous
public class DuckBlue extends LinearOpMode {
    public Robot robot;
    public static Pose2d startPose = new Pose2d(0,0,0);

    public enum State {
        STARTED,
        LIFTLOW,
        LIFTMID,
        LIFTHIGH,
        LIFTREST,
        DEPOSIT,
        SPINNERON,
        SPINNEROFF,
        INTAKEON,
        INTAKEOFF,
        STOPPED
    }
    public State state;
    OpenCvCamera camera;
    Vision vision;

    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, null, null);
        state = State.STARTED;
        robot.drive.drive.setPoseEstimate(startPose);

        // opening camera + starting vision pipeline
        vision = new Vision(hardwareMap, telemetry);
        vision.setPipeline();
        vision.startStreaming();

        // low path
        TrajectorySequence low = robot.drive.drive.trajectorySequenceBuilder(startPose)
                .build();

        // mid path
        TrajectorySequence mid = robot.drive.drive.trajectorySequenceBuilder(startPose)
                .build();

        // high path
        TrajectorySequence high = robot.drive.drive.trajectorySequenceBuilder(startPose)
                .build();

        waitForStart();
        VisionPipeline.POS position = vision.detector.getPosition();
        switch(position) {
            case LEFT:
                robot.drive.drive.followTrajectorySequenceAsync(low);
                break;
            case CENTER:
                robot.drive.drive.followTrajectorySequenceAsync(mid);
                break;
            case RIGHT:
                robot.drive.drive.followTrajectorySequenceAsync(high);
                break;
        }

        while (opModeIsActive()) {
            robot.update();
            switch (state) {
                case STARTED:
                    break;
                case LIFTLOW:
                    robot.lift.low();
                    break;
                case LIFTMID:
                    robot.lift.mid();
                    break;
                case LIFTHIGH:
                    robot.lift.high();
                    break;
                case LIFTREST:
                    robot.lift.rest();
                    robot.claw.intake();
                    robot.arm.intake();
                    break;
                case DEPOSIT:
                    robot.claw.release();
                    break;
                case SPINNERON:
                    robot.spinner.on();
                    break;
                case SPINNEROFF:
                    robot.spinner.off();
                    break;
                case INTAKEON:
                    robot.intake.on();
                    break;
                case INTAKEOFF:
                    robot.intake.off();
                    break;
            }
        }
    }
}
