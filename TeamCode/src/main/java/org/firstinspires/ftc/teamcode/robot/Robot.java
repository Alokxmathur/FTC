package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.game.Mineral;
import org.firstinspires.ftc.teamcode.robot.components.ColorAndDistanceSensor;
import org.firstinspires.ftc.teamcode.robot.components.TiaraThrone;
import org.firstinspires.ftc.teamcode.robot.components.Latch;
import org.firstinspires.ftc.teamcode.robot.components.PickerArm;
import org.firstinspires.ftc.teamcode.robot.components.FrontTrap;
import org.firstinspires.ftc.teamcode.robot.components.camera.Camera;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.modes.autonomous.AutonomousHelper;
import org.firstinspires.ftc.teamcode.robot.operations.CameraOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveTrainForTime;
import org.firstinspires.ftc.teamcode.robot.operations.DriveTrainOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveTrainUntilColor;
import org.firstinspires.ftc.teamcode.robot.operations.DriveTrainUntilSeeingVuMark;
import org.firstinspires.ftc.teamcode.robot.operations.FrontTrapOperation;
import org.firstinspires.ftc.teamcode.robot.operations.GyroscopicBearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.LatchOperation;
import org.firstinspires.ftc.teamcode.robot.operations.ThroneOperation;
import org.firstinspires.ftc.teamcode.robot.operations.MineralGrabOperation;
import org.firstinspires.ftc.teamcode.robot.operations.Operation;
import org.firstinspires.ftc.teamcode.robot.operations.OperationThread;
import org.firstinspires.ftc.teamcode.robot.operations.PIDGyroscopicBearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StraightLineIMUOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitTime;
import org.opencv.core.Rect;

import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.teamcode.robot.components.drivetrain.DriveTrain.DRIVE_TRAIN_WIDTH;

/**
 * This class represents our robot.
 * The config on the robot needs to have the following entries defined:
 * *
 * rightDrive: the right motor of the drive train
 * leftDrive: the left motor of the drive tpenrain
 */

public class Robot {

    public static final int CAMERA_FORWARD_DISPLACEMENT_FROM_CENTER_OF_ROBOT = 185;
    public static final int CAMERA_LEFT_DISPLACEMENT_FROM_CENTER_OF_ROBOT = -70;
    public static final int CAMERA_VERTICAL_DISPLACEMENT_FROM_BOTTOM_OF_ROBOT = 180;
    public static final int WHEEL_OFFSET_FROM_FRONT = 220;
    public static final int WHEEL_OFFSET_FROM_LATCH = 240;

    //distance we travel backwards to harvest
    public static final double DISTANCE_TO_TRAVEL_TO_HARVEST = 18 * Field.MM_PER_INCH;
    public static final double MINIMUM_DISTANCE_FROM_WALL_TO_SEE_VUMARK = 12 * Field.MM_PER_INCH;
    public static int NUM_CHANNELS = 5;
    public static final double MIN_AWAY_FROM_CENTER_FOR_AUTO_HARVEST = 52.0 * Field.MM_PER_INCH;
    public static final double MAX_AWAY_FROM_CENTER_FOR_AUTO_HARVEST = Field.FIELD_WIDTH / 2
            - Math.sqrt(Robot.WHEEL_OFFSET_FROM_FRONT * Robot.WHEEL_OFFSET_FROM_FRONT +
            DriveTrain.DRIVE_TRAIN_WIDTH * DriveTrain.DRIVE_TRAIN_WIDTH);
    public static final double HARVEST_CHANNEL_WIDTH =
            (MAX_AWAY_FROM_CENTER_FOR_AUTO_HARVEST - MIN_AWAY_FROM_CENTER_FOR_AUTO_HARVEST) / NUM_CHANNELS;


    private double randomDistanceFromCenter;
    private double travelToCraterWall;
    private AutonomousHelper autonomousHelper;

    public void openDistalGripper() {
        this.pickerArm.openDistalGripper();
    }

    public int getShoulderTarget() {
        return this.pickerArm.getShoulderTarget();
    }

    public boolean haveDistalMineral() {
        return this.pickerArm.haveColorBasedDistalMineral();
    }
    public boolean haveProximalMineral() {
        return this.pickerArm.haveColorBasedProximalMineral();
    }

    public String getDistalMineralColor() {
        return this.pickerArm.getDistalMineralColor();
    }
    public String getProximalMineralColor() {
        return this.pickerArm.getProximalMineralColor();
    }

    public boolean haveAnyMineral() {
        return this.pickerArm.haveAnyMineral();
    }
    public boolean haveBothMinerals() {
        return this.pickerArm.haveBothMinerals();
    }

    Telemetry telemetry;
    private HardwareMap hardwareMap;
    Match match;

    OperationThread operationThreadPrimary;
    OperationThread operationThreadSecondary;

    DriveTrain driveTrain = null;
    FrontTrap frontTrap = null;
    TiaraThrone tiaraThrone = null;
    Latch latch = null;
    PickerArm pickerArm = null;
    ColorAndDistanceSensor colorSensor;

    private float lastPower;
    private float lastTurn;
    private float lastLeftPower;
    private float lastRightPower;
    private float lastLatchPower;
    private float lastShoulderPower;
    private float lastWinchPower;

    boolean isInitialized = false;

    boolean tankDrive = false;

    //Our sensors etc.

    Camera camera;
    OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
            .translation(CAMERA_FORWARD_DISPLACEMENT_FROM_CENTER_OF_ROBOT,
                    CAMERA_LEFT_DISPLACEMENT_FROM_CENTER_OF_ROBOT,
                    CAMERA_VERTICAL_DISPLACEMENT_FROM_BOTTOM_OF_ROBOT)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.XYZ,
                    DEGREES, 180, 90, 0));

    //our state
    String state = "pre-initialized";
    private Rect goldRectangle;
    private double bearingToHarvest;
    private double bearingToSeeVuMark;
    //private double bearingToDeliverSilver;
    private double distanceToCraterWall;

    public Robot() {
        Log.d("SilverTitans", "Robot: got created");
    }

    public String findVuMark() {
        String foundTarget = this.camera.findTarget();
        if (foundTarget != null && !this.driveTrain.getIMU().isCalibratedWithKnownLocation()) {
            this.foundInitialVuMark(foundTarget);
        }
        return foundTarget;
    }

    /**
     * Initialize our robot
     * We set our alliance and our starting position based on finding a VuMark
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Match match) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.match = match;

        this.autonomousHelper = new AutonomousHelper(this);

        telemetry.addData("Status", "Creating operations thread, please wait");
        telemetry.update();
        this.operationThreadPrimary = new OperationThread(this, "Primary");
        operationThreadPrimary.start();
        this.operationThreadSecondary = new OperationThread(this, "Secondary");
        operationThreadSecondary.start();
        Match.log("Started operations threads");

        //initialize camera
        telemetry.addData("Status", "Initializing Camera, please wait");
        this.initVuforia();
        telemetry.update();
        Match.log("Initialized camera");

        //initialize our motors
        telemetry.addData("Status", "Initializing components, please wait");
        telemetry.update();
        initComponents(hardwareMap, telemetry);
        Match.log("Initialized components");

        telemetry.update();
        Match.log("Finished initialization of robot ");
        telemetry.addData("Status", "Initialization finished");
        telemetry.update();

        this.state = "Initialized";
        this.isInitialized = true;
    }

    public void calibrateIMU_Vuforia() {
        //this.camera.findTarget();
        this.driveTrain.getIMU().calibrateWithKnownLocation(this.camera.getLastKnownPosition());
    }

    public void initVuforia() {
        this.camera = new Camera();
        this.camera.init(hardwareMap, telemetry, this.phoneLocationOnRobot);
    }


    private void initComponents(HardwareMap hardwareMap, Telemetry telemetry) {
        //Create our drive train
        this.driveTrain = new DriveTrain(hardwareMap, telemetry);
        this.frontTrap = new FrontTrap(hardwareMap, telemetry);
        this.tiaraThrone = new TiaraThrone(hardwareMap, telemetry);
        this.latch = new Latch(hardwareMap, telemetry);
        this.pickerArm = new PickerArm(hardwareMap, telemetry);
        this.colorSensor = new ColorAndDistanceSensor(hardwareMap, telemetry);
    }

    private void setLeftDrivePower(double power) {
        this.driveTrain.setLeftPower(power);
    }

    private void setRightDrivePower(double power) {
        this.driveTrain.setRightPower(power);
    }

    /**
     * Stop the robot
     */
    public void stop() {
        //Stop all of our motors
        Match.log("Stopping robot");
        this.operationThreadPrimary.abort();
        this.operationThreadSecondary.abort();
        this.driveTrain.stop();
        this.pickerArm.openProximalGripper();
        this.pickerArm.openDistalGripper();
        Match.log(("Robot stopped"));
    }

    /**
     * Returns a string representing the status of the motors of the robot
     *
     * @return Motor Status
     */
    public String getMotorStatus() {
        return String.format("Driver:%s,%s", tankDrive ? "Tank" : "Arcade",
                this.driveTrain.getStatus());
    }

    /**
     * Returns a string representing the status of the sensor of the robot
     *
     * @return
     */
    public String getSensorStatus() {
        StringBuffer sensorStatus = new StringBuffer("nothing yet");

        return sensorStatus.toString();
    }

    public void queueUpWait(long msecs, String title, boolean primary) {
        if (primary) {
            this.operationThreadPrimary.queueUpOperation(new WaitTime(msecs, title));
        } else {
            this.operationThreadPrimary.queueUpOperation(new WaitTime(msecs, title));
        }
    }

    public void queueUpWait(long mSecs, String title) {
        this.queueUpWait(mSecs, title, true);
    }

    /**
     * Check if an operation has been completed
     *
     * @param operation - the operation to see if it is completed
     * @return
     */
    public boolean operationCompleted(Operation operation) {
        switch (operation.getType()) {
            case DRIVE_TRAIN: {
                DriveTrainOperation driveTrainOperation = (DriveTrainOperation) operation;
                return driveTrainOperation.isComplete(driveTrain);
            }
            case MINERAL_GRAB: {
                MineralGrabOperation mineralGrabOperation = (MineralGrabOperation) operation;
                if (!((MineralGrabOperation) operation).areBothMineralsPickedUp()) {
                    if (!mineralGrabOperation.isDistalMineralPickedUp() && this.pickerArm.seeingDistalMineral()) {
                        this.driveTrain.setLeftSpeed(0);
                        this.driveTrain.setRightSpeed(0);
                        this.pickerArm.closeDistalGripper();
                        sleep(500);
                        this.driveTrain.setLeftSpeed(mineralGrabOperation.getSpeed());
                        this.driveTrain.setRightSpeed(mineralGrabOperation.getSpeed());
                        mineralGrabOperation.setDistalMineralPickedUp(true);
                    }
                    if (mineralGrabOperation.isDistalMineralPickedUp() &&
                            !mineralGrabOperation.isProximalMineralPickedUp() && this.pickerArm.seeingProximalMineral()) {
                        this.driveTrain.setLeftSpeed(0);
                        this.driveTrain.setRightSpeed(0);
                        this.pickerArm.closeProximalGripper();
                        sleep(500);
                        this.driveTrain.setLeftSpeed(mineralGrabOperation.getSpeed());
                        this.driveTrain.setRightSpeed(mineralGrabOperation.getSpeed());
                        mineralGrabOperation.setProximalMineralPickedUp(true);
                    }
                }
                else {
                    this.pickerArm.setShoulderPosition(PickerArm.SHOULDER_VERTICAL_POSITION);
                    sleep(1000);
                    this.pickerArm.setWinchPosition(PickerArm.WINCH_VERTICAL_POSITION);
                }
                return this.driveTrain.driveTrainWithinRange();
            }
            case GYROSCOPIC_DRIVE: {
                StraightLineIMUOperation straightLineIMUOperation = (StraightLineIMUOperation) operation;
                return straightLineIMUOperation.isComplete(driveTrain);
            }
            case DRIVE_FOR_TIME: {
                DriveTrainForTime driveForTimeOperation = (DriveTrainForTime) operation;
                boolean complete = driveForTimeOperation.isComplete();
                if (complete) {
                    this.driveTrain.stop();
                }
                return complete;
            }
            case DRIVE_UNTIL_VUMARK: {
                boolean complete = this.findVuMark() != null;
                if (complete) {
                    this.driveTrain.stop();
                }
                return complete;
            }
            case DRIVE_UNTIL_COLOR: {
                DriveTrainUntilColor driveTrainUntilColor = (DriveTrainUntilColor) operation;
                boolean complete = driveTrainUntilColor.isComplete();
                if (complete) {
                    this.driveTrain.stop();
                }
                return complete;
            }
            case GYROSCOPIC_BEARING: {
                GyroscopicBearingOperation gyroscopicBearingOperation = (GyroscopicBearingOperation) operation;
                return gyroscopicBearingOperation.isComplete();
            }
            case PID_GYROSCOPIC_BEARING: {
                PIDGyroscopicBearingOperation pidGyroscopicBearingOperation = (PIDGyroscopicBearingOperation) operation;
                return pidGyroscopicBearingOperation.isComplete();
            }
            case LATCH: {
                LatchOperation latchOperation = (LatchOperation) operation;
                return latchOperation.isComplete(this.latch);
            }
            case PICKER_OPERATION: {
                PickerOperation pickerOperation = (PickerOperation) operation;
                return pickerOperation.isComplete(this.pickerArm);
            }
            case WAIT_TIME: {
                WaitTime waitTimeOperation = (WaitTime) operation;
                return waitTimeOperation.isComplete();
            }
            case FRONT_TRAP: {
                FrontTrapOperation frontTrapOperation = (FrontTrapOperation) operation;
                return frontTrapOperation.isComplete();
            }
            case THRONE: {
                ThroneOperation throneOperation = (ThroneOperation) operation;
                return throneOperation.isComplete();
            }
            case CAMERA: {
                CameraOperation cameraOperation = (CameraOperation) operation;
                return cameraOperation.isComplete();
            }
        }
        return false;
    }

    /**
     * execute an operation
     *
     * @param operation - the operation to execute
     */
    public void executeOperation(Operation operation) {
        switch (operation.getType()) {
            case DRIVE_TRAIN: {
                this.driveTrain.handleOperation((DriveTrainOperation) operation);
                break;
            }
            case MINERAL_GRAB: {
                this.pickerArm.openDistalGripper();
                this.pickerArm.openProximalGripper();
                this.driveTrain.handleOperation((MineralGrabOperation) operation);
                break;
            }
            case GYROSCOPIC_DRIVE: {
                this.driveTrain.handleOperation((StraightLineIMUOperation) operation);
                break;
            }
            case GYROSCOPIC_BEARING: {
                this.driveTrain.handleOperation((GyroscopicBearingOperation) operation);
                break;
            }
            case PID_GYROSCOPIC_BEARING: {
                this.driveTrain.handleOperation((PIDGyroscopicBearingOperation) operation);
                break;
            }
            case LATCH: {
                this.latch.handleOperation((LatchOperation) operation);
                break;
            }
            case PICKER_OPERATION: {
                this.pickerArm.handleOperation((PickerOperation) operation);
                break;
            }
            case DRIVE_FOR_TIME: {
                DriveTrainForTime driveForTimeOperation = (DriveTrainForTime) operation;
                driveForTimeOperation.setStart();
                this.driveTrain.setLeftPower(driveForTimeOperation.getLeftPower());
                this.driveTrain.setRightPower(driveForTimeOperation.getRightPower());
                break;
            }
            case DRIVE_UNTIL_COLOR: {
                DriveTrainUntilColor driveUntilColorOperation = (DriveTrainUntilColor) operation;
                this.driveTrain.handleOperation(driveUntilColorOperation);
                break;
            }
            case WAIT_TIME: {
                WaitTime waitTime = (WaitTime) operation;
                waitTime.setStart();
                break;
            }
            case DRIVE_UNTIL_VUMARK: {
                DriveTrainUntilSeeingVuMark driveUntilVuMarkOperation = (DriveTrainUntilSeeingVuMark) operation;
                this.driveTrain.handleOperation(driveUntilVuMarkOperation);
                break;
            }
            case FRONT_TRAP: {
                FrontTrapOperation frontTrapOperation = (FrontTrapOperation) operation;
                this.frontTrap.handleOperation(frontTrapOperation);
                break;
            }
            case THRONE: {
                ThroneOperation throneOperation = (ThroneOperation) operation;
                this.tiaraThrone.handleOperation(throneOperation);
                break;
            }
            case CAMERA: {
                CameraOperation cameraOperation = (CameraOperation) operation;
                this.camera.handleOperation(cameraOperation);
                break;
            }
        }
    }

    public void abortOperationCompleted(Operation operation) {
        switch (operation.getType()) {
            case DRIVE_TRAIN: {
                this.driveTrain.abortOperation();
                break;
            }
            case MINERAL_GRAB: {
                this.driveTrain.abortOperation();
                break;
            }
            case GYROSCOPIC_DRIVE: {
                this.driveTrain.abortOperation();
                break;
            }
            case GYROSCOPIC_BEARING: {
                this.driveTrain.abortOperation();
                break;
            }
            case PID_GYROSCOPIC_BEARING: {
                this.driveTrain.abortOperation();
                break;
            }
            case LATCH: {
                this.latch.stop();
                break;
            }
            case PICKER_OPERATION: {
                this.pickerArm.stop();
                break;
            }
            case DRIVE_FOR_TIME: {
                this.driveTrain.abortOperation();
                break;
            }
            case DRIVE_UNTIL_COLOR: {
                this.driveTrain.abortOperation();
                break;
            }
            case WAIT_TIME: {
                break;
            }
            case DRIVE_UNTIL_VUMARK: {
                this.driveTrain.abortOperation();
                break;
            }
            case FRONT_TRAP: {
                break;
            }
            case THRONE: {
                break;
            }
            case CAMERA: {
                break;
            }
        }
    }

    public void queueUpRotateClockwise(double degrees, String title) {
        queueUpRotateClockwise(degrees, title, true);
    }

    /**
     * Rotate clockwise
     *
     * @param degrees - number of degrees to rotate clockwise
     */
    public void queueUpRotateClockwise(double degrees, String title, boolean primary) {
        degrees = degrees % 360;
        if (degrees > 180) {
            degrees = -(degrees - 360);
        } else if (degrees < -180) {
            degrees = degrees + 360;
        }
        if (degrees != 0) {
            //Match.log("Queuing up rotation of " + degrees);
            double radians = (degrees % 360) * Math.PI / 180;
            double arcLength = radians * DRIVE_TRAIN_WIDTH / 2;
            this.queueUpTravel(arcLength, -arcLength, title + "(" + degrees + " rotation)", primary);
        } else {
            Match.log("Skipping 0 degree rotation");
        }
    }

    public void queueUpBearing(double bearing, String title) {
        queueUpBearing(bearing, title, true);
    }

    /**
     * Head to bearing using the IMU
     *
     * @param bearing - the bearing to head to
     */
    public void queueUpBearing(double bearing, String title, boolean primary) {
        if (primary) {
            this.operationThreadPrimary.queueUpOperation
                    (new GyroscopicBearingOperation(bearing, title, this.driveTrain));
        } else {
            this.operationThreadSecondary.queueUpOperation
                    (new GyroscopicBearingOperation(bearing, title, this.driveTrain));
        }
    }

    public void queueUpRawBearing(double bearing, String title) {
        queueUpRawBearing(bearing, title, true);
    }

    /**
     * Head to raw bearing using the IMU
     *
     * @param bearing - the bearing to head to
     */
    public void queueUpRawBearing(double bearing, String title, boolean primary) {
        if (primary) {
            this.operationThreadPrimary.queueUpOperation
                    (new GyroscopicBearingOperation(bearing, title, this.driveTrain, true));
        } else {
            this.operationThreadSecondary.queueUpOperation
                    (new GyroscopicBearingOperation(bearing, title, this.driveTrain, true));
        }
    }


    public void queueUpBearingPID(double bearing, String title) {
        queueUpBearingPID(bearing, title, true);
    }

    /**
     * Head to bearing using the IMU
     *
     * @param bearing - the bearing to head to
     */
    public void queueUpBearingPID(double bearing, String title, boolean primary) {
        if (primary) {
            this.operationThreadPrimary.queueUpOperation
                    (new PIDGyroscopicBearingOperation(bearing, title, this.driveTrain));
        } else {
            this.operationThreadSecondary.queueUpOperation
                    (new PIDGyroscopicBearingOperation(bearing, title, this.driveTrain));
        }
    }

    //turn left

    public void queueUpTurnLeft(double degrees, String title) {
        if (degrees != 0) {
            //Match.log("Queuing up left turn of " + degrees);
            double radians = (degrees % 360) * Math.PI / 180;
            double arcLength = radians * DRIVE_TRAIN_WIDTH;
            this.queueUpTravel(0, arcLength, title + "(" + degrees + " turn left)");
        } else {
            //Match.log("Skipping 0 degree left turn");
        }
    }

    //turn right

    public void queueUpTurnRight(double degrees, String title) {
        if (degrees != 0) {
            //Match.log("Queuing up right turn of " + degrees);
            double radians = (degrees % 360) * Math.PI / 180;
            double arcLength = radians * DRIVE_TRAIN_WIDTH;
            this.queueUpTravel(0, arcLength, title + "(" + degrees + " turn right)");
        } else {
            //Match.log("Skipping 0 degree right turn");
        }
    }

    public void queueUpTravel(double leftMMs, double rightMMs, String title, boolean primary) {
        if (leftMMs != 0 || rightMMs != 0) {
            if (primary) {
                this.operationThreadPrimary.queueUpOperation(new DriveTrainOperation(leftMMs, rightMMs, title));
            } else {
                this.operationThreadSecondary.queueUpOperation(new DriveTrainOperation(leftMMs, rightMMs, title));
            }
        } else {
            //Match.log("Skipping 0 left and right travel");
        }

    }

    /**
     * Move the specified number of mms left and right
     *
     * @param leftMMs
     * @param rightMMs
     */
    public void queueUpTravel(double leftMMs, double rightMMs, String title) {
        this.queueUpTravel(leftMMs, rightMMs, title, true);
    }

    public void queueUpTravel(double mms, String title, boolean primary) {

        this.queueUpTravel(mms, mms, title, primary);
    }

    public void queueUpTravelIMU(double mms, double degrees, String title) {
        this.queueUpTravelIMU(mms, degrees, title, true);
    }

    public void queueUpTravelIMU(double mms, double degrees, String title, boolean primary) {

        if (mms != 0) {
            if (primary) {
                this.operationThreadPrimary.queueUpOperation(
                        new StraightLineIMUOperation(mms, degrees, driveTrain, title));
            } else {
                this.operationThreadSecondary.queueUpOperation(
                        new StraightLineIMUOperation(mms, degrees, driveTrain, title));
            }
        }
    }

    /**
     * Move specified number of mms in a straight line
     *
     * @param mms
     */
    public void queueUpTravel(double mms, String title) {

        this.queueUpTravel(mms, mms, title, true);
    }

    public void queueUpTravelForTime(double leftPower, double rightPower, long time, String title, boolean primary) {
        this.queueOperation(new DriveTrainForTime(leftPower, rightPower, time, title), primary);
    }

    /**
     * Move the specified number of mms left and right
     *
     * @param leftPower
     * @param rightPower
     */
    public void queueUpTravelForTime(double leftPower, double rightPower, long time, String title) {
        this.queueUpTravelForTime(leftPower, rightPower, time, title, true);
    }

    public void queueUpTravelUntilVuMark(double leftPower, double rightPower, String title, boolean primary) {
        this.queueOperation(new DriveTrainUntilSeeingVuMark(leftPower, rightPower, title), primary);
    }

    public void queueOperation(Operation operation) {
        queueOperation(operation, true);
    }

    public void queueOperation(Operation operation, boolean primary) {
        if (primary) {
            this.operationThreadPrimary.queueUpOperation(operation);
        } else {
            this.operationThreadSecondary.queueUpOperation(operation);
        }

    }

    /**
     * Move the specified number of mms left and right
     *
     * @param leftPower
     * @param rightPower
     */
    public void queueUpTravelUntilVuMark(double leftPower, double rightPower, String title) {
        this.queueUpTravelUntilVuMark(leftPower, rightPower, title, true);
    }

    public void queueUpGoToPositionWithBearing(String title, float newX, float newY, float newBearing) {
        float currentX = this.camera.getCurrentX();
        float currentY = this.camera.getCurrentY();
        float currentBearing = this.camera.getCurrentBearing();
        float xMovement = newX - currentX;
        float yMovement = newY - currentY;

        Match.log(String.format(Locale.getDefault(),
                "Going from %.0f(%.2f\"),%.0f(%.2f\"):%.0f to %.0f(%.2f\"),%.0f(%.2f\"):%.0f",
                currentX, currentX / Field.MM_PER_INCH, currentY, currentY / Field.MM_PER_INCH, currentBearing,
                newX, newX / Field.MM_PER_INCH, newY, newY / Field.MM_PER_INCH, newBearing));

        //queueUpRotateClockwise to be parallel to the y axis - make bearing 90 or 270
        float bearingOffFrom90 = currentBearing - 90;
        float bearingOffFrom270 = currentBearing - 270;
        boolean rotateTo90 = Math.abs(bearingOffFrom90) < Math.abs(bearingOffFrom270);
        if (rotateTo90) {
            queueUpRotateClockwise(bearingOffFrom90, title + " (rot90)");
        } else {
            queueUpRotateClockwise(bearingOffFrom270, title + " (rot270)");
        }
        if (rotateTo90) {
            this.queueUpTravel(yMovement, title + " (travel Y)");
            if (xMovement > 0) {
                queueUpRotateClockwise(90, title + " (rot +90)");
                this.queueUpTravel(xMovement, title + " (travel +X)");
            } else {
                queueUpRotateClockwise(-90, title + " (rot -90)");
                this.queueUpTravel(-xMovement, title + " (travel -X)");
            }
        } else {
            this.queueUpTravel(-yMovement, title);
            if (xMovement > 0) {
                queueUpRotateClockwise(-90, title + " (rot -90)");
                this.queueUpTravel(xMovement, title + " (travel X)");
            } else {
                queueUpRotateClockwise(90, title + " (rot +90)");
                this.queueUpTravel(-xMovement, title + " (travel -X)");
            }
        }
        //now we queueUpRotateClockwise to point to the desired bearing
        if (xMovement > 0) {
            //our current bearing must be 0 degrees if we were advancing on the x axis
            if (newBearing > 180) {
                queueUpRotateClockwise(360 - newBearing, title + " (bearing 360-)");
            } else {
                queueUpRotateClockwise(-newBearing, title + " (bearing)");
            }
        } else {
            //our current bearing must be 180 degrees if we were advancing on the negative x axis
            if (newBearing > 180) {
                queueUpRotateClockwise(-(newBearing - 180), title + " (bearing)");
            } else {
                queueUpRotateClockwise(newBearing - 180, title + " (bearing)");
            }
        }
    }

    public float getCurrentX() {
        return this.camera.getCurrentX();
    }

    public float getCurrentY() {
        return this.camera.getCurrentY();
    }

    public double getBearing() {
        //return this.camera.getCurrentBearing();
        return this.driveTrain.getIMU().getBearing();
    }

    public void clearQueuedOperations() {
        this.operationThreadPrimary.abort();
        this.operationThreadSecondary.abort();
    }

    public boolean operationsCompleted() {
        return !(this.operationThreadPrimary.hasEntries() || this.operationThreadSecondary.hasEntries());
    }

    public String getPositionStatus() {
        return this.camera.getPosition();
    }

    public String getVuforiaBearingStatus() {
        return this.camera.getBearing();
    }

    public String getIMUBearingStatus() {
        return this.driveTrain.getIMU().getStatus();
    }

    public boolean sampledGold() {
        this.goldRectangle = camera.sampleGold();
        this.state = "Finding gold";
        if (this.goldRectangle != null) {
            this.state = "Found gold";
            return true;
        }
        return false;
    }

    public String getGoldSampleStatus() {
        if (this.goldRectangle == null) {
            return "No gold found";
        } else {
            return this.goldRectangle.toString();
        }
    }

    public Rect getGoldRectangle() {
        return this.goldRectangle;
    }

    /**
     * Set things up based on the initial VuMark
     *
     * @return
     */

    private String foundInitialVuMark(String target) {
        if (target != null) {
            if (target.equals(Camera.BACK_SPACE)) {
                match.setAlliance(Match.Alliance.Red);
                match.setStartingLaunchedFacing(Field.LaunchedFacing.Depot);
            } else if (target.equals(Camera.BLUE_ROVER)) {
                match.setAlliance(Match.Alliance.Blue);
                match.setStartingLaunchedFacing(Field.LaunchedFacing.Crater);
            } else if (target.equals(Camera.FRONT_CRATERS)) {
                match.setAlliance(Match.Alliance.Blue);
                match.setStartingLaunchedFacing(Field.LaunchedFacing.Depot);
            } else if (target.equals(Camera.RED_FOOTPRINT)) {
                match.setAlliance(Match.Alliance.Red);

                match.setStartingLaunchedFacing(Field.LaunchedFacing.Crater);
            }
            this.calibrateIMU_Vuforia();
            this.state = "Found " + target;
            Match.log("Found target: " + target + ", alliance=" + match.getAlliance());
            return target;
        }
        return null;
    }

    public String getState() {
        return this.state;
    }

    public void setState(String state) {
        this.state = state;
    }

    public boolean isInitialized() {
        return this.isInitialized;
    }

    public void closeTrap() {
        this.frontTrap.lower();
    }

    public void openTrap() {
        this.frontTrap.raise();
    }

    public void lowerThrone() {
        this.tiaraThrone.lowerServo();
    }

    public void raiseThrone() {
        this.tiaraThrone.raiseServo();
    }

    public boolean isGyroCalibrated() {
        return this.driveTrain.getIMU().isCalibrated();
    }

    public void setLatchPower(float power) {
        this.latch.setLatchPower(power);
    }

    public void unlockLatch() {
        this.latch.unlock();
    }

    public void lockLatch() {
        this.latch.lock();
    }

    public String getLatchStatus() {
        return this.latch.getStatus();
    }

    public String getPickerArmStatus() {
        return this.pickerArm.getStatus();
    }

    public boolean handleDriveTrain(Gamepad gamePad1, Gamepad gamePad2) {
        boolean didSomething = false;

        if (gamePad1.left_bumper && gamePad1.right_bumper) {
            Match.log("Going to bracket");
            queueGetToBracket();
            didSomething = true;
        }
        if (gamePad2.left_bumper && gamePad2.right_bumper) {
            Match.log("Auto harvest");
            autonomousHelper.queueAutoHarvest();
            didSomething = true;
        }
/*
        if (gamePad2.left_bumper) {
            Match.log("Shift left");
            this.shiftRightGoingForward(Field.MM_PER_INCH);
            didSomething = true;
        } else if (gamePad2.right_bumper) {
            Match.log("Shift right");
            this.shiftLeftGoingForward(Field.MM_PER_INCH);
            didSomething = true;
        }
*/
        if (gamePad1.y) {
            tankDrive = !tankDrive;
            didSomething = true;
        }
        if (tankDrive) {
            //tank drive
            float left = -gamePad1.right_stick_y * 0.5f; // Get left joystick's y-axis value.
            float right = gamePad1.left_stick_y * 0.5f; // Get right joystick's y-axis value.
            this.driveTrain.setLeftSpeed(left);
            this.driveTrain.setRightSpeed(right);
            //if we changed any motors, update telemetry
            if (lastLeftPower != left || lastRightPower != right) {
                lastLeftPower = left;
                lastRightPower = right;
                didSomething = true;
            }
        } else {
            //Arcade drive - gamepad 1 is driver
            float turn = gamePad1.right_stick_x * 0.25f; // Get right joystick's x-axis value.
            float power = -gamePad1.right_stick_y * 0.5f; // Get right joystick's y-axis value.
            this.setLeftDrivePower((power - turn));
            this.setRightDrivePower(-(power + turn));
            //if we changed any motors, update telemetry
            if (lastPower != power || lastTurn != turn) {
                lastPower = power;
                lastTurn = turn;
                didSomething = true;
            }
        }
        return didSomething;
    }

    public boolean handleLatch(Gamepad gamePad1, Gamepad gamePad2) {
        boolean didSomething = false;
/*        if (!tankDrive) {
            float power = -gamePad1.left_stick_y; // Get left joystick's y-axis value.
            this.setLatchPower(power);
            if (lastLatchPower != power) {
                didSomething = true;
                lastLatchPower = power;
            }
        }*/
        if (gamePad1.a || gamePad2.a) {
            this.unlockLatch();
            didSomething = true;
        } else if (gamePad1.b || gamePad2.b) {
            this.lockLatch();
            didSomething = true;
        }
        if (gamePad1.right_stick_button) {
            this.queueLatchToLander();
            didSomething = true;
        }
        if (gamePad1.left_stick_button) {
            this.queueFoldArm();
            didSomething = true;
        }
        return didSomething;
    }

    private void queueFoldArm() {
        this.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.INITIAL, "Fold arm"));
    }

    public boolean handleTrapAndThrone(Gamepad gamePad1, Gamepad gamePad2) {
        if (gamePad1.dpad_up) {
            this.openTrap();
            return true;
        }
        if (gamePad1.dpad_down) {
            this.closeTrap();
            return true;
        }
        if (gamePad1.dpad_right) {
            this.lowerThrone();
            return true;
        }
        if (gamePad1.dpad_left) {
            this.raiseThrone();
            return true;
        }
        return false;
    }

    public boolean handlePicker(Gamepad gamePad1, Gamepad gamePad2) {
        boolean didSomething = false;

        float shoulderMovement = gamePad2.left_stick_y; // Get left joystick's y-axis value.
        if (shoulderMovement > 0) {
            this.pickerArm.incrementShoulderPosition();
            didSomething = true;
        } else if (shoulderMovement < 0) {
            this.pickerArm.decrementShoulderPosition();
            didSomething = true;
        }

        float winchMovement = gamePad2.right_stick_y; // Get right joystick's y-axis value.
        this.pickerArm.setWinchPower(winchMovement);
        if (lastWinchPower != winchMovement) {
            didSomething = true;
            lastWinchPower = winchMovement;
        }

        if (gamePad2.left_trigger > 0) {
            this.pickerArm.closeDistalGripper();
            didSomething = true;
        }
        if (gamePad2.right_trigger > 0) {
            this.pickerArm.openDistalGripper();
            didSomething = true;
        }
        if (gamePad1.left_trigger > 0) {
            this.pickerArm.closeProximalGripper();
            didSomething = true;
        }
        if (gamePad1.right_trigger > 0) {
            this.pickerArm.openProximalGripper();
            didSomething = true;
        }
        if (gamePad2.dpad_up) {
            //queue delivery position
            queueDeliveryPosition(Mineral.Color.Silver);
            didSomething = true;
        }
        if (gamePad2.dpad_down) {
            queueHarvestPosition();
            didSomething = true;
        }

        return didSomething;
    }

    public String getColorStatus() {
        return this.colorSensor.getStatus();
    }


    public boolean handleGameControllers(Gamepad gamePad1, Gamepad gamePad2) {
        boolean somethingChanged = false;
        //abort mechanism in case getting to bracket is compromised
        if (gamePad1.x || gamePad2.x) {
            this.operationThreadPrimary.abort();
            this.operationThreadSecondary.abort();
            autonomousHelper.abort();
            somethingChanged = true;
        }

        autonomousHelper.checkAutoHarvesting();

        if (operationsCompleted()) {
            somethingChanged = this.handleTrapAndThrone(gamePad1, gamePad2);
            somethingChanged = this.handleDriveTrain(gamePad1, gamePad2) || somethingChanged;
            somethingChanged = this.handleLatch(gamePad1, gamePad2) || somethingChanged;
            somethingChanged = this.handlePicker(gamePad1, gamePad2) || somethingChanged;
        }
        //update telemetry if the robot changed anything
        if (somethingChanged) {
            this.setState("DC: Running");
            match.updateTelemetry(telemetry, "DC running");
        }
        return somethingChanged;
    }

    /**
     * Get to the lander bracket
     * We do this by first finding the vumark we are facing to get our coordinates and then
     * getting to the 45 degree line, rotating to face away from lander and backing up to the lander
     */

    public void queueGetToBracket() {
        autonomousHelper.abort();
        String vuMark = this.findVuMark();
        double d1 = 0, d2 = 0;
        double currentX = getCurrentX();
        double currentY = getCurrentY();
        double cos45 = Math.cos(Math.toRadians(45));
        double bracketDistanceFromCenter = Math.sqrt(2) * Field.BracketCoordinates.getX();

        double parallelToLanderBearing = 0;
        if (Camera.BLUE_ROVER.equals(vuMark)) {
            parallelToLanderBearing = -45;
            d1 = Math.abs(currentY - currentX) * cos45;
            d2 = d1 + Math.sqrt(2.0) * currentX
                    - bracketDistanceFromCenter
                    - Robot.WHEEL_OFFSET_FROM_LATCH;
        } else if (Camera.BACK_SPACE.equals(vuMark)) {
            parallelToLanderBearing = -135;
            d1 = Math.abs(currentX + currentY) * cos45;
            d2 = d1 - Math.sqrt(2.0) * currentY
                    - bracketDistanceFromCenter
                    - Robot.WHEEL_OFFSET_FROM_LATCH;
        } else if (Camera.RED_FOOTPRINT.equals(vuMark)) {
            parallelToLanderBearing = 135;
            d1 = Math.abs(currentY - currentX) * cos45;
            d2 = d1 - Math.sqrt(2.0) * currentX
                    - bracketDistanceFromCenter
                    - Robot.WHEEL_OFFSET_FROM_LATCH;
        } else if (Camera.FRONT_CRATERS.equals(vuMark)) {
            parallelToLanderBearing = 45;
            d1 = Math.abs(-currentX - currentY) * cos45;
            d2 = d1 - Math.sqrt(2.0) * currentY
                    - bracketDistanceFromCenter
                    - Robot.WHEEL_OFFSET_FROM_LATCH;
        } else {
            Match.getInstance().updateTelemetry(telemetry, "Didn't find a vumark, cant get to lander");
            return;
        }
        Match.log("Getting to bracket, d1=" + d1 + ", d2=" + d2);
        this.queueOperation(
                new LatchOperation(LatchOperation.LatchOperationType.Level, "Level latch"), false);
        this.queueOperation(
                new PickerOperation(PickerOperation.PickerOperationType.CLOSE_DISTAL, "Close gripper"), false);
        this.queueOperation(
                new PickerOperation(PickerOperation.PickerOperationType.INITIAL, "Fold arm"), false);
        this.queueUpBearing(parallelToLanderBearing,
                "Get parallel to lander: " + parallelToLanderBearing + " degrees");
        this.queueUpTravel(d1, "Get to 45 line");
        this.queueUpBearing(parallelToLanderBearing + 90, "Turn back to lander");
        this.queueUpBearing(parallelToLanderBearing + 90, "Turn back to lander ensured");
        this.queueOperation(new DriveTrainOperation(-d2, -d2, 0.5, "Get to bracket"));
    }

    private void queueDeliveryPosition(Mineral.Color color) {
        this.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.CLOSE_DISTAL,
                "Close picker"));
        this.queueOperation(new WaitTime(300, "Wait"));
        this.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_CRATER,
                "Clear crater"));
        this.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.Vertical,
                "Vertical"));

        double currentRawBearing = this.driveTrain.getIMU().getRawBearing();
        double currentX = 0;
        double currentY = 0;
        if (color == Mineral.Color.Silver) {
            this.queueUpRawBearing(0, "Pirouette");
        } else {

        }
        this.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.Deliver,
                "Delivery position"));
    }

    private void queueHarvestPosition() {
        this.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_DISTAL,
                "Open picker"), false);
        this.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.Harvest,
                "Harvest position"));
    }

    public void shiftLeftGoingBack(double mms) {
        shiftRightGoingBack(-mms);
    }

    public void shiftRightGoingBack(double mms) {
        this.queueUpTravel(-100, "Move back");
        this.queueUpRotateClockwise(90, "Rotate to face right");
        this.queueUpTravel(mms, "Move right");
        this.queueUpRotateClockwise(-90, "Rotate 90");
        this.queueUpTravel(100, "Move forward");
    }

    public void shiftLeftGoingForward(double mms) {
        shiftRightGoingForward(-mms);
    }

    public void shiftRightGoingForward(double mms) {
        this.queueUpTravel(100, "Move forward");
        this.queueUpRotateClockwise(90, "Rotate to face right");
        this.queueUpTravel(mms, "Move right");
        this.queueUpRotateClockwise(-90, "Rotate 90");
        this.queueUpTravel(-100, "Move back");
    }

    public void queueLatchToLander() {
        this.queueOperation(new LatchOperation(LatchOperation.LatchOperationType.Unlock, "Unlock latch"));
        this.queueOperation(new LatchOperation(LatchOperation.LatchOperationType.Engage, "Engage latch"));
        this.queueOperation(new LatchOperation(LatchOperation.LatchOperationType.Lift, "Lift Robot"));
        this.queueOperation(new LatchOperation(LatchOperation.LatchOperationType.Lock, "Lock latch"));
    }

    private void sleep(int mSecs) {
        try {
            Thread.sleep(mSecs);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

    public void queueInitialAutoHarvest() {
        this.autonomousHelper.queueInitialAutoHarvest();
    }
}
