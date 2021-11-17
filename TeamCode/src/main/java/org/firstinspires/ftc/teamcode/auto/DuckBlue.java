package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

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

    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, null, null);
        state = State.STARTED;

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
        if(isStopRequested()) return;

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
