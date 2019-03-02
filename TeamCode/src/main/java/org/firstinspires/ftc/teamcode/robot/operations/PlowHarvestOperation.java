package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Field;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class PlowHarvestOperation extends DriveTrainOperation {
    private boolean distalMineralPickedUp;
    private boolean proximalMineralPickedUp;
    public boolean areBothMineralsPickedUp() {
        return distalMineralPickedUp && proximalMineralPickedUp;
    }
    public boolean anyMineralPickedUp() {
        return distalMineralPickedUp || proximalMineralPickedUp;
    }

    public void setDistalMineralPickedUp(boolean distalMineralPickedUp) {
        this.distalMineralPickedUp = distalMineralPickedUp;
    }

    public PlowHarvestOperation(double travel, double speed, String title) {
        super(travel, travel, speed, title);
        this.type = TYPE.MINERAL_GRAB;
    }

    public String toString() {
        return String.format(Locale.getDefault(),"MineralGrab: %.2f(%.2f\"), %.2f(%.2f\") --%s",
                getLeftTravel(),  (super.getLeftTravel()/Field.MM_PER_INCH),
                getRightTravel(), (getRightTravel()/Field.MM_PER_INCH),
                super.title);
    }

    public boolean isDistalMineralPickedUp() {
        return distalMineralPickedUp;
    }

    public boolean isProximalMineralPickedUp() {
        return proximalMineralPickedUp;
    }

    public void setProximalMineralPickedUp(boolean proximalMineralPickedUp) {
        this.proximalMineralPickedUp = proximalMineralPickedUp;
    }
}
