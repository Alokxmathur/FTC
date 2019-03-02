package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/26/17.
 */

public class PickerArm {
    public static double OFFSET_FROM_BASE = 2.5*Field.MM_PER_INCH;
    public static double GRIPPER_HORIZONTAL_EXTENSION_AT_DELIVERY = 36*Field.MM_PER_INCH;

    public static final int BLUE_THRESHOLD_FOR_SILVER = 40;
    public static final int DISTANCE_WHEN_SEEING_MINERAL = 15;
    public static final String SHOULDER_MOTOR = "shoulderMotor";
    public static final String WINCH_MOTOR = "winchMotor";
    public static final String DISTAL_GRIPPER_SERVO = "distalGripperServo";
    public static final String PROXIMAL_GRIPPER_SERVO = "proximalGripperServo";
    public static final String DISTAL_GRIPPER_SENSOR = "distalGripperSensor";
    public static final String PROXIMAL_GRIPPER_SENSOR = "proximalGripperSensor";
    public static final String INTAKE_SERVO = "intakeServo";
    //our shoulder motor
    DcMotor shoulderMotor= null;
    //and the winch
    DcMotor winchMotor= null;
    //and the gripper servo
    Servo distalGripperServo = null;
    Servo proximalGripperServo = null;
    CRServo intakeServo = null;
    //the color and distance sensor
    ColorSensor distalColorSensor;
    DistanceSensor distalDistanceSensor;
    ColorSensor proximalColorSensor;
    DistanceSensor proximalDistanceSensor;

    public static final double  GRIPPER_CLOSED_POSITION= 0.3;
    public static final double  GRIPPER_OPEN_POSITION= 0.05;
    public static final double  GRIPPER_CHANNEL_POSITION= 0.0;
    public static final int SHOULDER_INCREMENT = 30;
    public static final double SHOULDER_POWER = 0.9;
    public static final int WINCH_INCREMENT = 8;
    public static final double WINCH_SPEED = 1.0;
    public static final double SLOW_WINCH_SPEED = 0.5;

    public static final int SHOULDER_INITIAL_POSITION = 300;
    public static final int SHOULDER_DELIVERY_POSITION = 1260;
    public static final int SHOULDER_DELIVERY_POSITION_AUTO = 1360;
    public static final int SHOULDER_CRATER_POSITION = 1700;
    public static final int SHOULDER_HARVEST_POSITION = 1855;
    public static final int SHOULDER_VERTICAL_POSITION = 1020;

    public static final int WINCH_INITIAL_POSITION = -100;
    public static final int WINCH_DELIVERY_POSITION = -4050;
    public static final int WINCH_DELIVERY_POSITION_AUTO = -4100;
    public static final int WINCH_CRATER_POSITION = -2290;
    public static final int WINCH_HARVEST_POSITION = -4100;
    public static final int WINCH_VERTICAL_POSITION = -400;
    public static final int WINCH_MAX = 0;
    public static final int WINCH_MIN = WINCH_DELIVERY_POSITION;

    private int desiredShoulderPosition;
    private int desiredWinchPosition;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};
    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;
    boolean distalGripperIsOpen = false;
    boolean proximalGripperIsOpen = false;
    private int RED_THRESHOLD_TO_DETERMINE_POSSESSION = 20;

    public PickerArm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        // Define and Initialize Motors
        shoulderMotor = hardwareMap.get(DcMotor.class, SHOULDER_MOTOR);
        winchMotor = hardwareMap.get(DcMotor.class, WINCH_MOTOR);
        distalGripperServo = hardwareMap.get(Servo.class, DISTAL_GRIPPER_SERVO);
        proximalGripperServo = hardwareMap.get(Servo.class, PROXIMAL_GRIPPER_SERVO);
        intakeServo = hardwareMap.get(CRServo.class, INTAKE_SERVO);
        // Define and Initialize sensors
        this.distalColorSensor = hardwareMap.get(ColorSensor.class, DISTAL_GRIPPER_SENSOR);
        this.distalDistanceSensor = hardwareMap.get(DistanceSensor.class, DISTAL_GRIPPER_SENSOR);
        this.proximalColorSensor = hardwareMap.get(ColorSensor.class, PROXIMAL_GRIPPER_SENSOR);
        this.proximalDistanceSensor = hardwareMap.get(DistanceSensor.class, PROXIMAL_GRIPPER_SENSOR);

        // Set shoulder motor to run with encoders.
        //this.shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set winch motor to run with encoders.
        //this.winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        closeDistalGripper();
        closeProximalGripper();
    }

    public void stop() {
        //Stop our motors
        shoulderMotor.setPower(0);
        winchMotor.setPower(0);
    }

    public String getStatus() {
        return String.format(Locale.getDefault(),
                "S:%.2f(%d>%d(%d)),W:%.2f(%d>%d(%d)),D:%.2f,R:%d,G:%d,B:%d=%s,%s,P:%.2f,R:%d,G:%d,B:%d=%s,%s",
                this.shoulderMotor.getPower(), this.shoulderMotor.getCurrentPosition(),
                this.shoulderMotor.getTargetPosition(), this.desiredShoulderPosition,
                this.winchMotor.getPower(), this.winchMotor.getCurrentPosition(),
                this.winchMotor.getTargetPosition(), this.desiredWinchPosition,
                distalDistanceSensor.getDistance(DistanceUnit.CM),
                distalColorSensor.red(),
                distalColorSensor.green(),
                distalColorSensor.blue(),
                getDistalMineralColor(),
                distalGripperIsOpen ? "Open" : "Closed",
                proximalDistanceSensor.getDistance(DistanceUnit.CM),
                proximalColorSensor.red(),
                proximalColorSensor.green(),
                proximalColorSensor.blue(),
                getProximalMineralColor(),
                proximalGripperIsOpen ? "Open" : "Closed");
    }

    public void setShoulderPosition(int position) {
        this.desiredShoulderPosition = position;
        this.shoulderMotor.setTargetPosition(position);
        this.shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.shoulderMotor.setPower(SHOULDER_POWER);
    }
    public void incrementShoulderPosition() {
        setShoulderPosition(desiredShoulderPosition + SHOULDER_INCREMENT);
    }
    public void decrementShoulderPosition() {
        setShoulderPosition(desiredShoulderPosition - SHOULDER_INCREMENT);
    }

    public void setWinchPosition(int winchPosition) {
        setWinchPosition(winchPosition, false);
    }

    private void setWinchPosition(int position, boolean slow) {
        this.desiredWinchPosition = position;
        this.winchMotor.setTargetPosition(position);
        this.winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.winchMotor.setPower(slow? SLOW_WINCH_SPEED : WINCH_SPEED);
    }
    public void incrementWinchPosition() {
        setWinchPosition(desiredWinchPosition + WINCH_INCREMENT, false);
    }
    public void decrementWinchPosition() {
        setWinchPosition(desiredWinchPosition - WINCH_INCREMENT, false);
    }

    public void openDistalGripper() {
        this.distalGripperServo.setPosition(GRIPPER_OPEN_POSITION);
        this.distalGripperIsOpen = true;
    }
    public void closeDistalGripper() {
        this.distalGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        this.distalGripperIsOpen = false;
    }
    public void openProximalGripper() {
        this.proximalGripperServo.setPosition(GRIPPER_OPEN_POSITION);
        this.proximalGripperIsOpen = true;
    }
    public void closeProximalGripper() {
        this.proximalGripperServo.setPosition(GRIPPER_CLOSED_POSITION);
        this.proximalGripperIsOpen = false;
    }

    public int getShoulderTarget() {
        return this.shoulderMotor.getTargetPosition();
    }

    public boolean isWithinReach() {
        return !this.shoulderMotor.isBusy() && !this.winchMotor.isBusy();
    }

    public void handleOperation(PickerOperation operation) {
        switch (operation.getPickerOperationType()) {
            case Deliver: {
                this.setShoulderPosition(SHOULDER_DELIVERY_POSITION);
                this.setWinchPosition(WINCH_DELIVERY_POSITION);
                break;
            }
            case DeliverAuto: {
                this.setShoulderPosition(SHOULDER_DELIVERY_POSITION_AUTO);
                this.setWinchPosition(WINCH_DELIVERY_POSITION_AUTO);
                break;
            }
            case Vertical: {
                this.setShoulderPosition(SHOULDER_VERTICAL_POSITION);
                this.setWinchPosition(WINCH_VERTICAL_POSITION);
                break;
            }
            case SHOULDER_VERTICAL: {
                this.setShoulderPosition(SHOULDER_VERTICAL_POSITION);
                break;
            }
            case Harvest: {
                this.setShoulderPosition(SHOULDER_HARVEST_POSITION);
                this.setWinchPosition(WINCH_HARVEST_POSITION);
                break;
            }
            case Crater: {
                this.setShoulderPosition(SHOULDER_CRATER_POSITION);
                this.setWinchPosition(WINCH_CRATER_POSITION);
                break;
            }
            case INITIAL: {
                this.setShoulderPosition(SHOULDER_INITIAL_POSITION);
                this.setWinchPosition(WINCH_INITIAL_POSITION);
                break;
            }
            case SHOULDER_CRATER: {
                this.setShoulderPosition(SHOULDER_CRATER_POSITION);
                break;
            }
            case SHOULDER_HARVEST: {
                this.setShoulderPosition(SHOULDER_HARVEST_POSITION);
                break;
            }
            case SHOULDER_DELIVERY: {
                this.setShoulderPosition(SHOULDER_DELIVERY_POSITION);
                break;
            }
            case SHOULDER_DELIVERY_AUTO: {
                this.setShoulderPosition(SHOULDER_DELIVERY_POSITION_AUTO);
                break;
            }
            case WINCH_CRATER: {
                this.setWinchPosition(WINCH_CRATER_POSITION);
                break;
            }
            case WINCH_COMPACT: {
                this.setWinchPosition(WINCH_VERTICAL_POSITION);
                break;
            }
            case WINCH_HARVEST: {
                this.setWinchPosition(WINCH_HARVEST_POSITION);
                break;
            }
            case WINCH_DELIVERY: {
                this.setWinchPosition(WINCH_DELIVERY_POSITION);
                break;
            }
            case WINCH_DELIVERY_AUTO: {
                this.setWinchPosition(WINCH_DELIVERY_POSITION_AUTO);
                break;
            }
            case OPEN_DISTAL: {
                this.openDistalGripper();
                break;
            }
            case CLOSE_DISTAL: {
                this.closeDistalGripper();
                break;
            }
            case OPEN_PROXIMAL: {
                this.openProximalGripper();
                break;
            }
            case CLOSE_PROXIMAL: {
                this.closeProximalGripper();
                break;
            }
            case INTAKE_ON: {
                turnIntakeOn();
                break;
            }
            case INTAKE_OFF: {
                turnIntakeOff();
                break;
            }
            case MINERAL_GRAB: {
                this.setShoulderPosition(SHOULDER_HARVEST_POSITION);
                this.openProximalGripper();
                this.openDistalGripper();
                this.turnIntakeOn();
                this.setWinchPower(-0.1f);
                break;
            }
        }
    }

    public void turnIntakeOn() {
        this.intakeServo.setPower(-1);
    }
    public void turnIntakeOff() {
        this.intakeServo.setPower(.1);
    }

    public void setWinchPower(float winchPower) {
        this.winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int currentPosition = this.winchMotor.getCurrentPosition();
        if ((currentPosition > WINCH_MIN && winchPower < 0)
                || (currentPosition < WINCH_MAX && winchPower > 0)) {
            this.winchMotor.setPower(winchPower * WINCH_SPEED);
        }
        else {
            this.winchMotor.setPower(0);
        }
    }
    public void setShoulderPower(float shoulderPower) {
        this.shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shoulderMotor.setPower(shoulderPower);
    }

    public String getDistalMineralColor() {
        int distalBlue = distalColorSensor.blue();
        String color = haveColorBasedDistalMineral() ? (distalBlue > BLUE_THRESHOLD_FOR_SILVER ? "Silver" : "Gold") : "Nothing";
        Match.log(
                "Distal red = " + distalColorSensor.red() + ", blue=" + distalBlue + ", green=" + distalColorSensor.green() + "=" + color);
        return color;
    }

    public boolean seeingDistalMineral() {
        //return this.distalColorSensor.red() < RED_THRESHOLD;
        return distalDistanceSensor.getDistance(DistanceUnit.CM) < DISTANCE_WHEN_SEEING_MINERAL;
    }
    public boolean haveColorBasedDistalMineral() {
        //return this.distalColorSensor.red() < RED_THRESHOLD;
        return distalColorSensor.red() > RED_THRESHOLD_TO_DETERMINE_POSSESSION;
    }
    public boolean haveColorBasedProximalMineral() {
        //return this.distalColorSensor.red() < RED_THRESHOLD;
        return proximalColorSensor.red() > RED_THRESHOLD_TO_DETERMINE_POSSESSION;
    }
    public String getProximalMineralColor() {
        int proximalBlue = proximalColorSensor.blue();
        String color = haveColorBasedProximalMineral() ? (proximalBlue > BLUE_THRESHOLD_FOR_SILVER ? "Silver" : "Gold") : "Nothing";
        Match.log(
                "Proximal red = " + proximalColorSensor.red() + ", blue=" + proximalBlue + ", green="
                        + proximalColorSensor.green() + "=" + color);
        return color;
    }

    public boolean seeingProximalMineral() {
        //return this.proximalColorSensor.red() < RED_THRESHOLD;
        return proximalDistanceSensor.getDistance(DistanceUnit.CM) < DISTANCE_WHEN_SEEING_MINERAL;
    }

    public boolean haveAnyMineral() {
        return haveColorBasedDistalMineral() || haveColorBasedProximalMineral();
    }

    public boolean haveBothMinerals() {
        return haveColorBasedDistalMineral() && haveColorBasedProximalMineral();
    }

    public boolean isFullyExtended() {
        return this.winchMotor.getCurrentPosition() < WINCH_DELIVERY_POSITION;
    }
}
