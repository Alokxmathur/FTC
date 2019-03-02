package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.PIDController;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class PIDGyroscopicBearingOperation extends GyroscopicBearingOperation {
    private PIDController pidController;


    public PIDGyroscopicBearingOperation(double desiredBearing, String title, DriveTrain driveTrain) {
        super(desiredBearing, title, driveTrain);
        this.type = TYPE.PID_GYROSCOPIC_BEARING;
        this.pidController = new PIDController(0.01, 0.00005, -0.005);
        this.pidController.setInputRange(0, 360);
        this.pidController.setOutputRange(0.01, DriveTrain.TURN_SPEED);
        this.pidController.setSetpoint(this.desiredBearing);
        this.pidController.enable();
    }

    public String toString() {
        return String.format(Locale.getDefault(),"PIDGyroscopicBearing: %.2f --%s",
                this.desiredBearing, this.title);
    }

    @Override
    public void setOperationBeingProcessed() {
        super.setOperationBeingProcessed();
        Match.log("Desired heading of " + desiredBearing);
    }

    public boolean isComplete() {
        // determine turn power based on +/- error
        double currentBearing = driveTrain.getIMU().getBearing();
        if (currentBearing < 0) {
            currentBearing += 360;
        }

        double turnSpeed = this.pidController.performPID(currentBearing);
        if (this.pidController.onTarget()) {
            driveTrain.setLeftSpeed(0);
            driveTrain.setRightSpeed(0);
            Match.log("Now bearing " + currentBearing + ", desired=" + desiredBearing);
            return true;
        }
        else {
            driveTrain.setLeftSpeed(turnSpeed);
            driveTrain.setRightSpeed(turnSpeed);
            Match.log("Applying power of " + turnSpeed + " for error of "
                    + (currentBearing - desiredBearing) + ", current=" + currentBearing);
            return false;
        }
    }

}
