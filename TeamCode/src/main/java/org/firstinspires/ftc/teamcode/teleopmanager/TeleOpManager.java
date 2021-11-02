package org.firstinspires.ftc.teamcode.teleopmanager;

import java.util.function.BooleanSupplier;

public class TeleOpManager {
    public BooleanSupplier trigger; // gamepad boolean input
    public RobotAction action, action2; // passed in robot actions

    public boolean prevState, state, toggle = false; // booleans for toggle/trigger

    // basic constructor (1 boolean trigger, 1 action)
    public TeleOpManager(BooleanSupplier trigger, RobotAction action) {
        this.trigger = trigger;
        this.action = action;
    }

    // toggle constructor (1 boolean toggle, 2 actions)
    public TeleOpManager(BooleanSupplier trigger, RobotAction action, RobotAction action2) {
        this.trigger = trigger;
        this.action = action;
        this.action2 = action2;
        toggle = true;
    }

    // updates state and calls run()
    public void update() {
        // trigger case
        if (!toggle) {
            if (trigger.getAsBoolean())
                action.run();
        }

        // toggle case
        else {
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
    }
}