package org.firstinspires.ftc.teamcode.robot.operations;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class CameraOperation extends Operation {
    public enum CameraOperationType {
        FLASH_ON, FLASH_OFF
    }

    public CameraOperationType getCameraOperationType() {
        return cameraOperationType;
    }

    CameraOperationType cameraOperationType;
    public CameraOperation(CameraOperationType armOperationType, String title) {
        this.type = TYPE.CAMERA;
        this.cameraOperationType = armOperationType;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Camera: %s --%s",
                this.cameraOperationType,
                this.title);
    }


    public boolean isComplete() {
        return true;
    }
}

