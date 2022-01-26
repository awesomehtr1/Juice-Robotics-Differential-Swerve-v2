package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperfunctions.AS5600;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.SwerveRotationPID;

// high level swerve drive class which utilizes gamepad controls to control the swerve modules
// wraps SwerveKinematics controls and implements SwerveModule
public class SwerveDrive {
    SwerveKinematics swerveKinematics;

    HardwareMap hardwareMap;

    DcMotor LFdrive, RFdrive, RBdrive, LBdrive;
    DcMotor LFrot, RFrot, RBrot, LBrot;

    ElapsedTime time = new ElapsedTime();
    // TODO: PID Constants
    public final double kP = 0.0;
    public final double kI = 0.0;
    public final double kD = 0.0;
    public final double kS = 0.0;
    SwerveRotationPID LFPID, RFPID, RBPID, LBPID = new SwerveRotationPID(kP, kI, kD, kS, time);

    AS5600 LFas5600, RFas5600, RBas5600, LBas5600;

    // array of swerve module objects
    SwerveModule LF, RF, RB, LB;
    SwerveModule[] swerveModules = new SwerveModule[4];

    public SwerveDrive(HardwareMap hardwareMap) {
        swerveKinematics = new SwerveKinematics();
        swerveKinematics.setTrackwidth(SwerveConstants.trackwidth);
        swerveKinematics.setWheelbase(SwerveConstants.wheelbase);

        this.hardwareMap = hardwareMap;

        LFdrive = hardwareMap.get(DcMotor.class, "LF");
        RFdrive = hardwareMap.get(DcMotor.class, "RF");
        RBdrive = hardwareMap.get(DcMotor.class, "RB");
        LBdrive = hardwareMap.get(DcMotor.class, "LB");

        //TODO: reverse drive motors if needed
//        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
//        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        LFrot = hardwareMap.get(DcMotor.class, "LFrot");
        RFrot = hardwareMap.get(DcMotor.class, "RFrot");
        RBrot = hardwareMap.get(DcMotor.class, "RBrot");
        LBrot = hardwareMap.get(DcMotor.class, "LBrot");

        //TODO: reverse rot motors if needed
//        LFrot.setDirection(DcMotorSimple.Direction.REVERSE);
//        LFrot.setDirection(DcMotorSimple.Direction.REVERSE);
//        LFrot.setDirection(DcMotorSimple.Direction.REVERSE);
//        LFrot.setDirection(DcMotorSimple.Direction.REVERSE);

        LFas5600 = new AS5600(hardwareMap, "LFas5600");
        RFas5600 = new AS5600(hardwareMap, "RFas5600");
        RBas5600 = new AS5600(hardwareMap, "RBas5600");
        LBas5600 = new AS5600(hardwareMap, "LBas5600");

        LF = new SwerveModule(LFrot, LFdrive, LFPID, LFas5600);
        RF = new SwerveModule(RFrot, RFdrive, RFPID, RFas5600);
        RB = new SwerveModule(RBrot, RBdrive, RBPID, RBas5600);
        LB = new SwerveModule(LBrot, LBdrive, LBPID, LBas5600);

        // add swerve modules to array
        swerveModules[0] = LF;
        swerveModules[1] = RF;
        swerveModules[2] = RB;
        swerveModules[3] = LB;
    }

    // sets rotation and drive powers using inputs from gamepad
    public void setMotorPowers(double rotation, double strafe, double forward) {
        swerveKinematics.calculateKinematics(rotation, strafe, forward);
        double[] rotAngleArray = swerveKinematics.getWheelAngles();
        double[] drivePowerArray = swerveKinematics.getWheelVelocities();
        setRotPowerArray(rotAngleArray);
        setDrivePowerArray(drivePowerArray);
    }

    // method for setting pid target for individual swerve module with angle optimization
    public void setRotationPower(SwerveModule module, double angle) {
        if(Math.abs(angle - module.getAngle()) > Math.PI/2) {
            angle = 180 + angle;
            module.setReverseDrive(true);
        }
        else
            module.setReverseDrive(false);
        module.setAngle(angle);
        double power = module.updatePID(module.getAngle());
        module.setRot(power);
    }

    public void setDrivePower(SwerveModule module, double power) {
        module.setDrive(power);
//        TODO: module.setDrive(power - compensateRotation(module));
    }

    // compensate wheel rotation when pod rotates
    public double compensateRotation(SwerveModule module) {
        double power = module.getRotPower();
        power *= SwerveConstants.wheelRotPerPodRot;
        power /= SwerveConstants.driveRotRatio;
        return power;
    }

    // sets rotation power for all modules in the array
    // takes 1 dimensional array of rotation angles from SwerveKinematics
    public void setRotPowerArray(double[] rotAngleArray) {
        for (int i = 0; i < 4; i++)
            setRotationPower(swerveModules[i], rotAngleArray[i]);
    }

    // sets drive power for all modules in the array
    // takes 1 dimensional array of drive powers from SwerveKinematics
    public void setDrivePowerArray(double[] drivePowerArray) {
        for (int i = 0; i < 4; i++)
            setDrivePower(swerveModules[i], drivePowerArray[i]);
    }
}