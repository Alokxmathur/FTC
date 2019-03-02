package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class StraightLineIMUOperation extends DriveTrainOperation {
    private double travelDistance;
    private double desiredHeading;
    private DriveTrain driveTrain;

    public double getTravelDistance() {
        return travelDistance;
    }

    public StraightLineIMUOperation(double travelDistance, double heading,
                                    DriveTrain driveTrain, String title) {
        super(travelDistance, travelDistance, title);
        this.travelDistance = travelDistance;
        this.type = TYPE.GYROSCOPIC_DRIVE;
        this.desiredHeading = heading;
        this.driveTrain = driveTrain;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "StraightLine: %.2f(%.2f\")@%.2f --%s",
                this.travelDistance, (this.travelDistance / Field.MM_PER_INCH), this.desiredHeading,
                this.title);
    }


    public boolean isComplete(DriveTrain driveTrain) {
        if (driveTrain.driveTrainWithinRange()) {
            driveTrain.stop();
            return true;
        } else {
            // adjust relative speed based on desiredHeading error.
            double bearingError = AngleUnit.normalizeDegrees(desiredHeading - driveTrain.getIMU().getBearing());
            double steer = DriveTrain.getSteer(bearingError, DriveTrain.P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (travelDistance < 0)
                steer *= -1.0;
            //double speed = this.getSpeed();
            double speed = driveTrain.getRampedSpeed(this);
            double leftSpeed = speed - steer;
            double rightSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0) {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            driveTrain.setLeftSpeed(leftSpeed);
            driveTrain.setRightSpeed(rightSpeed);
            return false;
        }
    }
}

