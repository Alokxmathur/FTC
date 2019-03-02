package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.operations.ThroneOperation;

public class TiaraThrone {
    Servo servo;
    public TiaraThrone(HardwareMap hardwareMap, Telemetry telemetry) {

        // Define and Initialize our Servo
        this.servo = hardwareMap.get(Servo.class, "markerServo");
        raiseServo();
    }

    public void raiseServo() {
        this.servo.setPosition(1);
    }
    public void lowerServo() {
        this.servo.setPosition(0);
    }

    public void handleOperation(ThroneOperation throneOperation) {
        if (throneOperation.getThroneOperationType() == ThroneOperation.ThroneOperationType.Lower) {
            this.lowerServo();
        }
        else {
            this.raiseServo();
        }
    }
}
