package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.SwerveRotationPID;

import java.util.Dictionary;
import java.util.HashMap;

public class SwerveDrive {
    SwerveKinematics swerveKinematics;
    private final double wheelbase = 0.0; //get from cad
    private final double trackwidth = 0.0; //get from cad

    HardwareMap hardwareMap;

    DcMotor LFdrive, RFdrive, RBdrive, LBdrive;
    DcMotorEx LFrot, RFrot, RBrot, LBrot;

    ElapsedTime time = new ElapsedTime();
    // TODO: PID
    public final double kP = 0.0;
    public final double kI = 0.0;
    public final double kD = 0.0;
    public final double kS = 0.0;
    SwerveRotationPID LFPID, RFPID, RBPID, LBPID = new SwerveRotationPID(kP, kI, kD, kS, time);

    SwerveModule LF, RF, RB, LB;

    public HashMap<String, SwerveModule> moduleHashMap = new HashMap<>();

    private final double ticksPerRot = 28 * 60 / 30 * 84 / 14; // 28 ticks per rot motor -> 60:30 -> 84:14
    private final double ticksPerRad = ticksPerRot / (Math.PI * 2);

    public SwerveDrive(HardwareMap hardwareMap) {
        swerveKinematics = new SwerveKinematics();
        swerveKinematics.setTrackwidth(trackwidth);
        swerveKinematics.setWheelbase(wheelbase);

        this.hardwareMap = hardwareMap;

        LFdrive = hardwareMap.get(DcMotor.class, "LF");
        RFdrive = hardwareMap.get(DcMotor.class, "RF");
        RBdrive = hardwareMap.get(DcMotor.class, "RB");
        LBdrive = hardwareMap.get(DcMotor.class, "LB");

        LFrot = hardwareMap.get(DcMotorEx.class, "LFrot");
        RFrot = hardwareMap.get(DcMotorEx.class, "RFrot");
        RBrot = hardwareMap.get(DcMotorEx.class, "RBrot");
        LBrot = hardwareMap.get(DcMotorEx.class, "LBrot");

        LF = new SwerveModule(LFrot, LFdrive, LFPID);
        RF = new SwerveModule(RFrot, RFdrive, RFPID);
        RB = new SwerveModule(RBrot, RBdrive, RBPID);
        LB = new SwerveModule(LBrot, LBdrive, LBPID);
    }

    public void setMotorPowers(double rotation, double strafe, double forward) {
        swerveKinematics.calculateKinematics(rotation, strafe, forward);
    }

    public void setRotationPower(SwerveModule module, double angle) {
        module.pid.setState(angle);
        double power = module.pid.updatePID(module.rot.getCurrentPosition());
        module.rot.setPower(power);
    }

    public void setDrivePower(SwerveModule module, double power) {
        
    }
}
