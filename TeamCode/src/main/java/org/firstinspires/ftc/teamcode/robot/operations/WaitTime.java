package org.firstinspires.ftc.teamcode.robot.operations;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class WaitTime extends Operation {
    public float getTime() {
        return time;
    }

    private long time;
    private Date timeStarted;

    public WaitTime(long time, String title) {
        this.type = TYPE.WAIT_TIME;
        this.time = time;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(),"WaitTime: for %d msecs --%s",
                this.time, this.title);
    }

    public void setStart() {
        this.timeStarted = new Date();
    }
    public boolean isComplete() {
        return new Date().getTime() - timeStarted.getTime() > time;
    }
}
