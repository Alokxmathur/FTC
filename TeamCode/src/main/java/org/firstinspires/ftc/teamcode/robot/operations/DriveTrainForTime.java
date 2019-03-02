package org.firstinspires.ftc.teamcode.robot.operations;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class DriveTrainForTime extends Operation {
    private double leftPower;
    private double rightPower;

    public float getTime() {
        return time;
    }

    private long time;
    private Date timeStarted;

    public DriveTrainForTime(double leftPower, double rightPower, long time, String title) {
        this.type = TYPE.DRIVE_FOR_TIME;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        this.time = time;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(),"DriveForTime: L:%.2f, R:%.2f, for %d msecs --%s",
                this.leftPower, this.rightPower, this.time, this.title);
    }

    public void setStart() {
        this.timeStarted = new Date();
    }
    public boolean isComplete() {
        return new Date().getTime() - timeStarted.getTime() > time;
    }
    public double getLeftPower() {
        return leftPower;
    }
    public double getRightPower() {
        return rightPower;
    }
}
