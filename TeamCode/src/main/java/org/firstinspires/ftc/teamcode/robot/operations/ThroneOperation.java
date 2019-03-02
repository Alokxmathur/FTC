package org.firstinspires.ftc.teamcode.robot.operations;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class ThroneOperation extends Operation {
    public enum ThroneOperationType {
        Lower, Raise
    }

    public ThroneOperationType getThroneOperationType() {
        return throneOperationType;
    }

    ThroneOperationType throneOperationType;
    public ThroneOperation(ThroneOperationType throneOperationType, String title) {
        this.type = TYPE.THRONE;
        this.throneOperationType = throneOperationType;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "TiaraThrone: %s --%s",
                this.throneOperationType,
                this.title);
    }


    public boolean isComplete() {
        return (new Date().getTime() - getStartTime().getTime() > 500);
    }
}

