package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class DriveTrainOperation extends Operation {
    private double leftTravel;
    private double rightTravel;
    private double speed;
    private int initialEncoderValueLeft;
    private int desiredEncoderValueLeft;
    private int initialEncoderValueRight;
    private int desiredEncoderValueRight;


    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public int getInitialEncoderValueLeft() {
        return initialEncoderValueLeft;
    }

    public void setInitialEncoderValueLeft(int initialEncoderValueLeft) {
        this.initialEncoderValueLeft = initialEncoderValueLeft;
    }

    public int getDesiredEncoderValueLeft() {
        return desiredEncoderValueLeft;
    }

    public void setDesiredEncoderValueLeft(int desiredEncoderValueLeft) {
        this.desiredEncoderValueLeft = desiredEncoderValueLeft;
    }

    public int getInitialEncoderValueRight() {
        return initialEncoderValueRight;
    }

    public void setInitialEncoderValueRight(int getInitialEncoderValueRight) {
        this.initialEncoderValueRight = getInitialEncoderValueRight;
    }

    public int getDesiredEncoderValueRight() {
        return desiredEncoderValueRight;
    }

    public void setDesiredEncoderValueRight(int desiredEncoderValueRight) {
        this.desiredEncoderValueRight = desiredEncoderValueRight;
    }

    public double getLeftTravel() {
        return leftTravel;
    }

    public void setLeftTravel(double leftDestination) {
        this.leftTravel = leftDestination;
    }

    public double getRightTravel() {
        return rightTravel;
    }

    public void setRightTravel(double rightDestination) {
        this.rightTravel = rightDestination;
    }

    public DriveTrainOperation(double leftTravel, double rightTravel, double speed, String title) {
        this.type = TYPE.DRIVE_TRAIN;
        this.leftTravel = leftTravel;
        this.rightTravel = rightTravel;
        this.title = title;
        this.speed = speed;
    }

    public DriveTrainOperation(double leftTravel, double rightTravel, String title) {
        this(leftTravel, rightTravel, DriveTrain.DRIVE_SPEED, title);
    }

    public String toString() {
        return String.format(Locale.getDefault(), "DriveTrain: %.2f(%.2f\"), %.2f(%.2f\") --%s",
                this.leftTravel, (this.leftTravel / Field.MM_PER_INCH),
                this.rightTravel, (this.rightTravel / Field.MM_PER_INCH),
                this.title);
    }

    public double getSpeed() {
        return this.speed;
    }

    public boolean isComplete(DriveTrain driveTrain) {
        if (driveTrain.driveTrainWithinRange()) {
            driveTrain.stop();
            return true;
        } else {
            //set ramped speed values
            double speed = driveTrain.getRampedSpeed(this);
            driveTrain.setLeftSpeed(speed);
            driveTrain.setRightSpeed(speed);
            return false;
        }
    }
}

