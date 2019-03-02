package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.operations.FrontTrapOperation;

public class FrontTrap {
    public static final double ARM_EXTENSION_BEYOND_WHEELS = 250;//600;
    public static final double INCREMENT = 0.05;
    public static final double RAISED_POSITION = 0.15;
    public static final double LOWERED_POSITION = 0.7;
    public static final double INITIAL_POSITION = LOWERED_POSITION;
    public static final double LEVEL_POSITION = 0.41;

    Servo servo;
    public FrontTrap(HardwareMap hardwareMap, Telemetry telemetry) {

        // Define and raise our Servo
        this.servo = hardwareMap.get(Servo.class, "samplingServo");
        this.servo.setPosition(INITIAL_POSITION);
    }

    public void raise() {
        this.servo.setPosition(RAISED_POSITION);
    }
    public void lower() {
        this.servo.setPosition(LOWERED_POSITION);
    }
    public void level() {
        this.servo.setPosition(LEVEL_POSITION);
    }

    public void raiseIncrementally() {
        this.servo.setPosition(this.servo.getPosition() - INCREMENT);
    }
    public void lowerIncrementally() {
        this.servo.setPosition(this.servo.getPosition() + INCREMENT);
    }

    public void handleOperation(FrontTrapOperation frontTrapOperation) {
        switch (frontTrapOperation.getTrapOperationType()) {
            case Close: {
                this.lower();
                break;
            }
            case Level: {
                this.level();
                break;
            }
            case Open: {
                this.raise();
                break;
            }
            default:{}
        }
    }
}
