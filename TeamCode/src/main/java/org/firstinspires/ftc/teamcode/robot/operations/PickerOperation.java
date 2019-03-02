package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.robot.components.PickerArm;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class PickerOperation extends Operation {

    public enum PickerOperationType {
        Deliver, DeliverAuto, Harvest, Crater, INITIAL, Vertical, SHOULDER_CRATER, SHOULDER_HARVEST, SHOULDER_VERTICAL,
        SHOULDER_DELIVERY, SHOULDER_DELIVERY_AUTO, WINCH_COMPACT, WINCH_HARVEST, WINCH_CRATER, WINCH_DELIVERY, WINCH_DELIVERY_AUTO,
        OPEN_DISTAL, CLOSE_DISTAL, OPEN_PROXIMAL, CLOSE_PROXIMAL, INTAKE_ON, INTAKE_OFF, MINERAL_GRAB;
    }

    public PickerOperationType getPickerOperationType() {
        return pickerOperationType;
    }

    PickerOperationType pickerOperationType;

    public boolean isProximalPickedUp() {
        return proximalPickedUp;
    }

    public void setProximalPickedUp(boolean proximalPickedUp) {
        this.proximalPickedUp = proximalPickedUp;
    }

    boolean proximalPickedUp;

    public boolean isDistalPickedUp() {
        return distalPickedUp;
    }

    public void setDistalPickedUp(boolean distalPickedUp) {
        this.distalPickedUp = distalPickedUp;
    }

    boolean distalPickedUp;

    public PickerOperation(PickerOperationType pickerOperationType, String title) {
        this.type = TYPE.PICKER_OPERATION;
        this.pickerOperationType = pickerOperationType;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Picker: %s --%s",
                this.pickerOperationType,
                this.title);
    }


    public boolean isComplete(PickerArm picker) {
        switch (getPickerOperationType()) {
            case Deliver:
            case DeliverAuto:
            case Harvest:
            case Crater:
            case Vertical:
            case INITIAL:
            case SHOULDER_CRATER:
            case SHOULDER_HARVEST:
            case SHOULDER_DELIVERY:
            case SHOULDER_DELIVERY_AUTO:
            case SHOULDER_VERTICAL:
            case WINCH_CRATER:
            case WINCH_COMPACT:
            case WINCH_HARVEST:
            case WINCH_DELIVERY_AUTO:
            case WINCH_DELIVERY: {
                return picker.isWithinReach();
            }
            case CLOSE_PROXIMAL:
            case OPEN_PROXIMAL:
            case CLOSE_DISTAL:
            case OPEN_DISTAL:
            case INTAKE_OFF:
            case INTAKE_ON:
            {
                return true;
            }
            case MINERAL_GRAB: {
                if (!isProximalPickedUp() && picker.seeingProximalMineral()) {
                    picker.closeProximalGripper();
                    setProximalPickedUp(true);
                }
                if (isProximalPickedUp() && !isDistalPickedUp() && picker.seeingDistalMineral()) {
                    picker.closeDistalGripper();
                    setDistalPickedUp(true);
                }
                return picker.seeingDistalMineral() && picker.seeingProximalMineral() || picker.isFullyExtended();
            }
            default:
                //return false for everything else we did not cover
                return false;
        }
    }
}

