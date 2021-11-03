package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.teleopmanager.TeleOpManager;

import java.util.ArrayList;

public class Robot {
    public ArrayList<Subsystem> Subsystems; // list of subsystems
    public ArrayList<TeleOpManager> TeleOpManagers; // list of teleopmanagers
    public HardwareMap hardwareMap; // stores hardware map

    // robot subsystems
    public Drive drive;
    public Lift lift;
    public Intake intake;

    public Gamepad gamepad1, gamepad2; // gamepads

    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap; // init passed-in hardwaremap

        // init subsystems
        drive = new Drive(this);
        lift = new Lift(this);
        intake = new Intake(this);

        Subsystems = new ArrayList<>();
        TeleOpManagers = new ArrayList<>();

        // add subsystems to list
        Subsystems.add(drive);
        Subsystems.add(lift);
        Subsystems.add(intake);

        // init gamepads
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    // updates robot
    public void update() {
        for(Subsystem subsystem : Subsystems)
            subsystem.update(); // calls update method of every subsystem
        for(TeleOpManager manager: TeleOpManagers)
            manager.update(); // calls update method of every teleopmanager
    }

    // creates teleopmanager and adds it to list of teleopmanagers
    public void createTeleOpManager(TeleOpManager teleOpManager) {
        TeleOpManagers.add(teleOpManager);
    }
}