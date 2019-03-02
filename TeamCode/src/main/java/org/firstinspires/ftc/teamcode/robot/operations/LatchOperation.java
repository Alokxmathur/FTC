package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.robot.components.Latch;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class LatchOperation extends Operation {
    public enum LatchOperationType {
        Level, Raise, Lock, Unlock, Release, Engage, Lift
    }

    public LatchOperationType getLatchOperationType() {
        return latchOperationType;
    }

    LatchOperationType latchOperationType;
    public LatchOperation(LatchOperationType latchOperationType, String title) {
        this.type = TYPE.LATCH;
        this.latchOperationType = latchOperationType;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Latch: %s --%s",
                this.latchOperationType,
                this.title);
    }


    public boolean isComplete(Latch latch) {
        switch (getLatchOperationType()) {
            case Lock: {
            }
            case Unlock: {
                return true;
            }
            case Release:
            case Raise:
            case Level:
            case Engage:
            case Lift:
            {
                return latch.isWithinReach();
            }
            default:
                return true;
            }
        }
}

