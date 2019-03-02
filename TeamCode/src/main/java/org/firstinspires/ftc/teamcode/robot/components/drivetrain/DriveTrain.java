package org.firstinspires.ftc.teamcode.robot.components.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.imu.IMU;
import org.firstinspires.ftc.teamcode.robot.operations.DriveTrainOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveTrainUntilColor;
import org.firstinspires.ftc.teamcode.robot.operations.DriveTrainUntilSeeingVuMark;
import org.firstinspires.ftc.teamcode.robot.operations.GyroscopicBearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.MineralGrabOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StraightLineIMUOperation;

/**
 * Created by Silver Titans on 10/26/17.
 */

public class DriveTrain {
    //Define constants that help us move appropriate inches based on our drive configuration
    public static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: Rev HD Motor Encoder
    //public static final double     COUNTS_PER_MOTOR_REV    = 280 ;    // eg: Rev Hex Motor Encoder
    private static final double     WHEEL_RADIUS   = 45;     // For figuring circumference (mm)
    private static  final double     COUNTS_PER_MM = COUNTS_PER_MOTOR_REV  /
            (WHEEL_RADIUS  * Math.PI * 2);
    public static final int DRIVE_TRAIN_WITHIN_RANGE = 5;
    //our drive train width is 14 7/8 inches
    public static final double DRIVE_TRAIN_WIDTH = 290;
    public static final double MINIMUM_POWER = 0.1;

    public static final double RAMP_UP_INTERVAL=0.15;
    public static final double RAMP_DOWN_INTERVAL=0.15;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    public static double     DRIVE_SPEED             = 1.0;     // Nominal speed for better accuracy.
    public static final double     TURN_SPEED              = 0.7;     // Nominal half speed for better accuracy.

    public static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it
    public static final double     P_TURN_COEFF            = 0.0075;     // Larger is more responsive, but also less stable
    public static final double     P_DRIVE_COEFF           = 0.01;     // Larger is more responsive, but also less stable

    //our left and right motors
    DcMotor leftDrive   = null;
    DcMotor rightDrive  = null;
    // The IMU sensor object
    IMU imu;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public DriveTrain(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        // Define and Initialize Motors
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        // Set all dc motors to run with encoders.
        this.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //setup our IMU
        this.imu = new IMU(hardwareMap);
    }

    /** Set power of left motor
     *
     * @param power
     *
     */
    public void setLeftPower(double power) {
        Match.log("Set left power = " + power + ", mode=" + this.leftDrive.getMode());
        this.leftDrive.setPower(power);
    }

    /**
     * Set power of right motor
     * @param power
     */
    public void setRightPower(double power) {
        Match.log("Set right power = " + power + ", mode=" + this.rightDrive.getMode());
        this.rightDrive.setPower(power);
    }

    /** Set speed of left motor
     *
     * @param speed
     *
     */
    public void setLeftSpeed(double speed) {
        this.leftDrive.setPower(speed);
    }

    /**
     * Set speed of right motor
     * @param speed
     */
    public void setRightSpeed(double speed) {
        this.rightDrive.setPower(speed);
    }

    public void handleOperation(DriveTrainOperation operation) {
        stop();
        // Determine new target position, and pass to motor controller
        int initialLeftEncoderValue = leftDrive.getCurrentPosition();
        int initialRightEncoderValue = rightDrive.getCurrentPosition();
        int newLeftTarget = initialLeftEncoderValue - (int)(operation.getLeftTravel() * COUNTS_PER_MM);
        int newRightTarget = initialRightEncoderValue + (int)(operation.getRightTravel() * COUNTS_PER_MM);

        operation.setInitialEncoderValueLeft(initialLeftEncoderValue);
        operation.setInitialEncoderValueRight(initialRightEncoderValue);
        operation.setDesiredEncoderValueLeft(newLeftTarget);
        operation.setDesiredEncoderValueRight(newRightTarget);

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        setMode(leftDrive, DcMotor.RunMode.RUN_TO_POSITION);
        setMode(rightDrive, DcMotor.RunMode.RUN_TO_POSITION);

        this.leftDrive.setPower(Math.max(operation.getSpeed()/7, MINIMUM_POWER));
        this.rightDrive.setPower(Math.max(operation.getSpeed()/7, MINIMUM_POWER));
    }

    public void handleOperation(DriveTrainUntilSeeingVuMark operation) {
        stop();
        leftDrive.setPower(-operation.getLeftPower());
        rightDrive.setPower(operation.getRightPower());
    }

    public void handleOperation(DriveTrainUntilColor operation) {
        stop();
        leftDrive.setPower(-operation.getLeftPower());
        rightDrive.setPower(operation.getRightPower());
    }

    public void handleOperation(StraightLineIMUOperation operation) {
        handleOperation((DriveTrainOperation) operation);
    }

    public void handleOperation(GyroscopicBearingOperation operation) {
        stop();
    }

    private boolean withinRange()  {
        return !leftDrive.isBusy() || !rightDrive.isBusy();
    }

    /**
     * Check if the drive train is within the specified encoder count
     * @return
     */
    public boolean driveTrainWithinRange() {
        if (withinRange())
        {
            stop();
            return true;
        }
        else {
            return false;
        }
    }

    public void stop() {
        //Stop our motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        this.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public String getStatus() {
        return String.format("L:%.2f(%d>%d),R:%.2f(%d>%d)",
            this.leftDrive.getPower(), this.leftDrive.getCurrentPosition(), this.leftDrive.getTargetPosition(),
            this.rightDrive.getPower(), this.rightDrive.getCurrentPosition(), this.rightDrive.getTargetPosition());

    }

    public static void setMode(DcMotor motor, DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public IMU getIMU() {
        return this.imu;
    }



    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public static double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public double getRampedSpeed(DriveTrainOperation operation) {
        boolean isEarly = new java.util.Date().getTime() - operation.getStartTime().getTime() < 500;
        int delta = Math.abs(operation.getDesiredEncoderValueRight() - operation.getInitialEncoderValueRight());
        int completed = Math.abs(rightDrive.getCurrentPosition() - operation.getInitialEncoderValueRight());
        float completionRatio = (float) completed / (float) delta;
        double speed = 0;
        if (completionRatio > RAMP_DOWN_INTERVAL) {
            double slope = operation.getSpeed() / (1.0 - RAMP_UP_INTERVAL);
            speed = Math.max(completionRatio * slope * -1 + Math.abs(slope), MINIMUM_POWER);
        } else if (completionRatio < RAMP_UP_INTERVAL) {
            double slope = operation.getSpeed() / RAMP_UP_INTERVAL;
            speed = Math.max(completionRatio * slope, MINIMUM_POWER);
        } else {
            speed = operation.getSpeed();
        }
        if (isEarly) {
            speed = Math.min(speed, 0.05);
        }

/*
        Match.log("Ramped speed: " + speed + ", completion: " + completionRatio + ", completed: " + completed
                + ",ld: " + leftDrive.getCurrentPosition() + "->" + leftDrive.getTargetPosition() + ":" + leftDrive.getMode()
                + ",rd: " + rightDrive.getCurrentPosition() + "->" + rightDrive.getTargetPosition() + ":" + rightDrive.getMode());
*/

        return speed;
    }

    public DcMotor.RunMode getLeftRunMode() {
        return this.leftDrive.getMode();
    }
    public DcMotor.RunMode getRightRunMode() {
        return this.rightDrive.getMode();
    }

    public void abortOperation() {
        stop();
    }
}
