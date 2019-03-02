package org.firstinspires.ftc.teamcode.robot.operations;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class FrontTrapOperation extends Operation {
    public enum TrapOperationType {
        Close, Open, Level
    }

    public TrapOperationType getTrapOperationType() {
        return trapOperationType;
    }

    TrapOperationType trapOperationType;
    public FrontTrapOperation(TrapOperationType trapOperationType, String title) {
        this.type = TYPE.FRONT_TRAP;
        this.trapOperationType = trapOperationType;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Trap: %s --%s",
                this.trapOperationType,
                this.title);
    }


    public boolean isComplete()
    {
        return (new Date().getTime() - getStartTime().getTime() > 400);
    }
}

