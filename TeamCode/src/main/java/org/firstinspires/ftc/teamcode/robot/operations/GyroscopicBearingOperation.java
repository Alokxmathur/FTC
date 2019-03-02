package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class GyroscopicBearingOperation extends Operation {
    protected double desiredBearing;
    protected DriveTrain driveTrain;
    boolean rawBearing = false;
    double minimumPower = 0.0;

    public double getMinimumPower() {
        return minimumPower;
    }

    public void setMinimumPower(double minimumPower) {
        this.minimumPower = minimumPower;
    }

    public GyroscopicBearingOperation(double desiredBearing, String title, DriveTrain driveTrain) {
        this.type = TYPE.GYROSCOPIC_BEARING;
        this.driveTrain = driveTrain;
        this.title = title;
        this.desiredBearing = desiredBearing;
    }

    public GyroscopicBearingOperation(double desiredBearing, String title, DriveTrain driveTrain, boolean rawBearing) {
        this.type = TYPE.GYROSCOPIC_BEARING;
        this.driveTrain = driveTrain;
        this.title = title;
        this.desiredBearing = desiredBearing;
        this.rawBearing = rawBearing;
    }

    public String toString() {
        return String.format(Locale.getDefault(),"GyroscopicBearing: %.2f --%s",
                this.desiredBearing, this.title);
    }

    @Override
    public void setOperationBeingProcessed() {
        super.setOperationBeingProcessed();
        //Match.log("Desired heading of " + desiredBearing);
    }

    public boolean isComplete() {
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        double currentBearing = rawBearing ? driveTrain.getIMU().getRawBearing() : driveTrain.getIMU().getBearing();
        double error = AngleUnit.normalizeDegrees(desiredBearing - currentBearing);
        boolean isEarly = false;//new java.util.Date().getTime() - this.getStartTime().getTime() < 500;

        if (Math.abs(error) <= DriveTrain.HEADING_THRESHOLD) {
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
            //Match.log("Now bearing " + desiredBearing);
        }
        else {
            steer = DriveTrain.getSteer(error, DriveTrain.P_TURN_COEFF);
            boolean isNegative = steer < 0;
            double absoluteSpeed = Math.max(Math.abs((isEarly ? 0.05 : DriveTrain.TURN_SPEED) * steer), minimumPower);
            leftSpeed = rightSpeed = absoluteSpeed * (isNegative ? -1 : 1);
            //Match.log("Gyroscopic bearing error: " + error + ", power: " + leftSpeed + ", mode: " + driveTrain.getLeftRunMode());
        }

        // Send desired speeds to motors.
        driveTrain.setLeftSpeed(leftSpeed);
        driveTrain.setRightSpeed(rightSpeed);

        return onTarget;
    }

}
