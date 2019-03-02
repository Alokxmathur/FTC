package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.operations.LatchOperation;

/**
 * Created by Silver Titans on 10/26/17.
 */

public class Latch {
    public static double POWER = 1.0;
    public static int raisedPosition = 720;
    public static int engagedPosition = 600;
    public static int liftedPosition = 200;
    public static int leveledPosition = 200;

    public static double lockPosition = 0;
    public static double unlockPosition = 0.15;

    boolean locked;
    //our latch motor
    DcMotor latchMotor = null;
    Servo lockServo = null;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public Latch(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        // Define and Initialize Motor
        latchMotor = hardwareMap.get(DcMotor.class, "latchMotor");
        lockServo = hardwareMap.get(Servo.class, "lockServo");

        // Set all dc motors to run with encoders.
        this.latchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.latchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lock();
    }

    public void handleOperation(LatchOperation operation) {
        switch (operation.getLatchOperationType()) {
            case Lock: {
                this.lock();
                break;
            }
            case Unlock: {
                this.unlock();
                break;
            }
            case Release: {
                this.unlock();
/*
                this.latchMotor.setTargetPosition(releasePosition);
                this.latchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.latchMotor.setPower(POWER);
*/
                break;
            }
            case Raise: {
                this.latchMotor.setTargetPosition(raisedPosition);
                this.latchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.latchMotor.setPower(POWER);
                break;
            }
            case Level: {
                this.latchMotor.setTargetPosition(leveledPosition);
                this.latchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.latchMotor.setPower(POWER);
                break;
            }
            case Engage: {
                this.latchMotor.setTargetPosition(engagedPosition);
                this.latchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.latchMotor.setPower(POWER);
                break;
            }
            case Lift: {
                this.latchMotor.setTargetPosition(liftedPosition);
                this.latchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.latchMotor.setPower(POWER);
                break;
            }
            default: {
            }
        }
    }

    /** Set power of left motor
     *
     * @param power
     *
     */
    public void setLatchPower(double power) {
        this.latchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.latchMotor.setPower(power);
    }

    /** Set speed of left motor
     *
     * @param speed
     *
     */
    public void setLatchSpeed(double speed) {
        //setMode(latchMotor, DcMotor.RunMode.RUN_USING_ENCODER);
        this.latchMotor.setPower(speed);
    }


    public void stop() {
        //Stop our motors
        latchMotor.setPower(0);
    }

    public void lock() {
        this.lockServo.setPosition(lockPosition);
        //this.latchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.locked = true;
    }

    public void unlock() {
        this.lockServo.setPosition(unlockPosition);
        //this.latchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.locked = false;
    }

    public String getStatus() {
        return String.format("L:%.2f(%d>%d),%s",
                this.latchMotor.getPower(), this.latchMotor.getCurrentPosition(), this.latchMotor.getTargetPosition(),
                this.locked ? "Locked" : "Unlocked");

    }

    public boolean isWithinReach() {
        if (!this.latchMotor.isBusy()) {
            this.latchMotor.setPower(0);
            return true;
        }
        //Match.log("Not yet reached latch position, " + this.latchMotor.getCurrentPosition());
        return false;
/*
        int target = latchMotor.getTargetPosition();
        int current = latchMotor.getCurrentPosition();
        if (Math.abs(target - current) <= 5) {
            //this.latchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.latchMotor.setPower(0);
            //Match.log("Latch: Target=" + target + ", current=" + current + " is within reach");
            return true;
        }
        return false;
*/
    }
}
