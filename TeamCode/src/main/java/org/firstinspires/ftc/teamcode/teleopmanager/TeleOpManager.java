package org.firstinspires.ftc.teamcode.teleopmanager;

import android.app.job.JobInfo;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleOpManager {
    public BooleanSupplier trigger; // gamepad boolean input
    public DoubleSupplier pos; // gamepad double input
    public RobotAction action, action2; // passed in robot actions

    public boolean prevState, state = false; // booleans for toggle/trigger
    public enum MODE {
        TRIGGER,
        TOGGLE,
        DOUBLE
    };
    public MODE mode;

    // basic constructor (1 boolean trigger, 1 action)
    public TeleOpManager(BooleanSupplier trigger, RobotAction action) {
        this.trigger = trigger;
        this.action = action;
        mode = MODE.TRIGGER;
    }

    // toggle constructor (1 boolean toggle, 2 actions)
    public TeleOpManager(BooleanSupplier trigger, RobotAction action, RobotAction action2) {
        this.trigger = trigger;
        this.action = action;
        this.action2 = action2;
        mode = MODE.TOGGLE;
    }

    // double constructor (1 double, 1 action)
    public TeleOpManager(DoubleSupplier pos, RobotAction action){
        this.pos = pos;
        this.action = action;
        mode = MODE.DOUBLE;
    }

    // updates state and calls run()
    public void update() {
        // trigger mode
        if (mode == MODE.TRIGGER) {
            if (trigger.getAsBoolean())
                action.run();
        }

        // toggle mode
        else if (mode == MODE.TOGGLE){
            if (trigger.getAsBoolean()) {
                if (!prevState)
                    state = !state;
                prevState = true;
            }
            else {
                if (prevState)
                    prevState = false;
            }
            if(state)
                action.run();
            else
                action2.run();
        }
        else if (mode == MODE.DOUBLE){
            action.run();
        }
    }
}