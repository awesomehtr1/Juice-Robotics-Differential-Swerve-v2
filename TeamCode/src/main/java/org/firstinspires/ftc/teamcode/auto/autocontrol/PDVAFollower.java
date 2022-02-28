package org.firstinspires.ftc.teamcode.auto.autocontrol;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperfunctions.PID.BasicPID;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;

public class PDVAFollower {
    private SwerveDrive swerveDrive;
    private PDVAController pdvaController;
    private MotionProfile motionProfile;
    private BasicPID rotationPID;
    private ElapsedTime time;

    private Path path;

    private double lookahead;

    public PDVAFollower(
            HardwareMap hardwareMap,
            double[] PDVAconstants,
            double[] motionProfileConstants,
            double[] rotationPIDconstants)
    {
        swerveDrive = new SwerveDrive(hardwareMap);
        pdvaController = new PDVAController(
                PDVAconstants[0],
                PDVAconstants[1],
                PDVAconstants[2],
                PDVAconstants[3]
        );
        motionProfile = new MotionProfile(
                motionProfileConstants[0],
                motionProfileConstants[1],
                motionProfileConstants[2],
                motionProfileConstants[3]
        );
        rotationPID = new BasicPID(
                rotationPIDconstants[0],
                rotationPIDconstants[1],
                rotationPIDconstants[2],
                rotationPIDconstants[3],
                time);
    }

    public double getErrorToPath() {
        double rise = path.getSegmentEnd().y - path.getSegmentStart().y;
        double run = path.getSegmentEnd().x - path.getSegmentEnd().x;
        double slope = rise / run;

    }

    public void setLookahead(double lookahead) {
        this.lookahead = lookahead;
    }

    public void setStartPose(double x, double y) {
        swerveDrive.setPose(x, y);
    }

    public void setPath(Path path) {
        this.path = path;
    }
}