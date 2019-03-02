package org.firstinspires.ftc.teamcode.robot.operations;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class DriveTrainUntilSeeingVuMark extends Operation {
    private double leftPower;
    private double rightPower;

    public DriveTrainUntilSeeingVuMark(double leftPower, double rightPower, String title) {
        this.type = TYPE.DRIVE_UNTIL_VUMARK;
        this.title = title;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
    }

    public String toString() {
        return String.format(Locale.getDefault(),"DriveUntilVuMark: (%.2f, %.2f) --%s",
                this.leftPower,  this.rightPower, this.title);
    }

    public double getLeftPower() {
        return leftPower;
    }

    public void setLeftPower(double leftPower) {
        this.leftPower = leftPower;
    }

    public double getRightPower() {
        return rightPower;
    }

    public void setRightPower(double rightPower) {
        this.rightPower = rightPower;
    }
}
