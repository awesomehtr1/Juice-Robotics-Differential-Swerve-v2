package org.firstinspires.ftc.teamcode.testopmodes.swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperfunctions.AS5600;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.SwerveRotationPID;

@TeleOp(name = "Swerve Homing Test", group = "TestOpModes")
public class SwerveHomingTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        AS5600 as5600 = new AS5600(hardwareMap, "analogtest", 2.523);
        DcMotor rot = hardwareMap.get(DcMotor.class, "rot");
        ElapsedTime time = new ElapsedTime();
        SwerveRotationPID pid = new SwerveRotationPID(5, 0.1, 1, 0, time);
        waitForStart();
        while(opModeIsActive()){
            pid.setState(0);
            double power = pid.updatePID(as5600.getAngle());
            rot.setPower(power);
        }
    }
}
