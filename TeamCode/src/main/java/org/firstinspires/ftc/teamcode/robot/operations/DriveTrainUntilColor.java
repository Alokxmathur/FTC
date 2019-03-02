package org.firstinspires.ftc.teamcode.robot.operations;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.ColorAndDistanceSensor;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class DriveTrainUntilColor extends Operation {
    private double leftPower;
    private double rightPower;
    private Match.Alliance color;
    private ColorAndDistanceSensor colorAndDistanceSensor;

    private Date timeStarted;

    public DriveTrainUntilColor(double leftPower, double rightPower, Match.Alliance color, ColorAndDistanceSensor colorAndDistanceSensor,
                                String title) {
        this.type = TYPE.DRIVE_UNTIL_COLOR;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        this.color = color;
        this.colorAndDistanceSensor = colorAndDistanceSensor;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(),"DriveForTime: L:%.2f, R:%.2f, until %s --%s",
                this.leftPower, this.rightPower, this.color.toString(), this.title);
    }

    public boolean isComplete() {
        return colorAndDistanceSensor.isSeeing(color);
    }
    public double getLeftPower() {
        return leftPower;
    }
    public double getRightPower() {
        return rightPower;
    }
}
