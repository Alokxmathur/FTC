package org.firstinspires.ftc.teamcode.robot.operations;

import java.util.Date;

/**
 * Created by Silver Titans on 10/29/17.
 */

public abstract class Operation {

    public enum TYPE {
        DRIVE_TRAIN, DRIVE_FOR_TIME, WAIT_TIME, DRIVE_UNTIL_VUMARK, GYROSCOPIC_ROTATION,
        GYROSCOPIC_DRIVE, GYROSCOPIC_BEARING, PID_GYROSCOPIC_BEARING,
        FRONT_TRAP, THRONE, LATCH, CAMERA, DRIVE_UNTIL_COLOR, PICKER_OPERATION,
        MINERAL_GRAB;
    }
    protected boolean operationIsBeingProcessed = false;

    public Date getStartTime() {
        return startTime;
    }

    private Date startTime;

    TYPE type;
    String title;
    public String getTitle() {
        return title;
    }
    public TYPE getType() {
        return this.type;
    }
    public boolean getOperationIsBeingProcessed() {
        return this.operationIsBeingProcessed;
    }
    public void setOperationBeingProcessed() {
        this.operationIsBeingProcessed = true;
        this.startTime = new Date();
    }
}
