package org.firstinspires.ftc.teamcode.robot.modes.autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Coordinates;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.camera.Camera;
import org.firstinspires.ftc.teamcode.robot.operations.MineralGrabOperation;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;

import java.util.Date;

public class AutonomousHelper {
    private static final double DISTANCE_TO_TRAVEL_TO_HARVEST = 18 * Field.MM_PER_INCH;

    public enum HarvestState {
        DriverControlled, Harvesting, Retrying, Retracting, FindingVuMark, Delivering
    }

    HarvestState harvestState = HarvestState.DriverControlled;
    private int harvestChannel = 1;
    private double randomStartingY;
    private double bearingToHarvest;
    private double bearingToSeeVuMark;
    private double distanceToCraterWall;
    private Robot robot;
    private double channelDistanceFromCenter;

    public static double getCraterToDepotBearing() {
        if (Match.getInstance().getAlliance() == Match.Alliance.Red) {
            //Red alliance
            if (Match.getInstance().getLaunchedFacing() == Field.LaunchedFacing.Crater) {
                //launched from red facing crater
                return 0;
            } else {
                //launched from red facing depot
                return -90;
            }
        } else {
            //Blue alliance
            if (Match.getInstance().getLaunchedFacing() == Field.LaunchedFacing.Crater) {
                //launched from blue facing crater
                return 180;
            } else {
                //launched from blue facing depot
                return 90;
            }
        }
    }

    public static double bearingToSeeVuMark() {
        if (Match.getInstance().getAlliance() == Match.Alliance.Blue) { //blue alliance
            if (Match.getInstance().getLaunchedFacing() == Field.LaunchedFacing.Crater) {
                return 90;
            } else {
                return 180;
            }
        } else { //Red alliance
            if (Match.getInstance().getLaunchedFacing() == Field.LaunchedFacing.Crater) {
                return -90;
            } else {
                return 0;
            }
        }
    }


    public AutonomousHelper(Robot robot) {
        this.robot = robot;
    }

    public void checkAutoHarvesting() {
        if (robot.operationsCompleted()) {
            switch (harvestState) {
                case DriverControlled: {
                    break;
                }
                case Harvesting: {
                    //we were harvesting and operations completed
                    if (robot.haveAnyMineral() || timeToLatch()) {
                        Match.log("Have mineral, retracting to see VUMark");
                        queueAutoRetractToVuMark();
                    } else {
                        Match.log("No mineral, retrying harvest");
                        queueAutoHarvestRetry();
                    }
                    break;
                }
                case Retracting: {
                    //we were retracting and we have completed our operations
                    if (robot.haveAnyMineral() || timeToLatch()) {
                        //if we have mineral, queue finding VuMark before we can deliver
                        Match.log("Have mineral, trying to find VUMark");
                        queueAutoFindVuMark();
                    } else {
                        //no - mineral - retry getting one
                        Match.log("Retract: No mineral to deliver, retrying auto-harvest");
                        queueAutoHarvestRetry();
                    }
                    break;
                }
                case FindingVuMark: {
                    //found vuMark, now try auto delivery
                    if (!timeToLatch()) {
                        queueAutoDelivery();
                    } else {
                        robot.queueGetToBracket();
                    }
                    break;
                }
                case Delivering: {
                    //were able to deliver - queue auto harvest if we still have time
                    if (!timeToLatch()) {
                        queueAutoHarvest();
                    } else {
                        robot.queueGetToBracket();
                    }
                    break;
                }
            }
        }
    }

    public void queueInitialAutoHarvest() {
        harvestState = HarvestState.Harvesting;
        bearingToHarvest = AutonomousHelper.getCraterToDepotBearing();
        bearingToSeeVuMark = AutonomousHelper.bearingToSeeVuMark();
        harvestChannel = Robot.NUM_CHANNELS - 1;
        distanceToCraterWall = 12*Field.MM_PER_INCH;
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.Harvest, "Harvest position"));
        robot.queueOperation(new MineralGrabOperation(DISTANCE_TO_TRAVEL_TO_HARVEST, 0.1, "Grab minerals"));
    }

    public void queueAutoHarvest() {
        String vuMark = robot.findVuMark();
        double currentX = robot.getCurrentX();
        double currentY = robot.getCurrentY();
        //get next channel
        this.harvestChannel = (this.harvestChannel +1) % Robot.NUM_CHANNELS;
        channelDistanceFromCenter = Robot.MIN_AWAY_FROM_CENTER_FOR_AUTO_HARVEST
                + (harvestChannel+1) * Robot.HARVEST_CHANNEL_WIDTH;
        Match.log("Channel = " + harvestChannel + ", distance from center =" + channelDistanceFromCenter);

        if (Camera.BLUE_ROVER.equals(vuMark)) {
            bearingToHarvest = 180;
            bearingToSeeVuMark = 90;
        } else if (Camera.RED_FOOTPRINT.equals(vuMark)) {
            bearingToHarvest = 0;
            bearingToSeeVuMark = -90;
        } else if (Camera.FRONT_CRATERS.equals(vuMark)) {
            bearingToHarvest = 90;
            bearingToSeeVuMark = 180;
        } else if (Camera.BACK_SPACE.equals(vuMark)) {
            bearingToHarvest = -90;
            bearingToSeeVuMark = 0;
        } else {
            harvestState = HarvestState.DriverControlled;
            Match.log("Didn't find a right vumark, can't harvest");
            return;
        }
        //bearingToHarvest = AngleUnit.normalizeDegrees(bearingToHarvest);
        harvestState = HarvestState.Harvesting;
        Match.log("Harvesting from vuMark: " + vuMark);
        //get to a random spot
        robot.queueUpTravel(
                channelDistanceFromCenter
                        - Math.abs(bearingToSeeVuMark == 0 || bearingToSeeVuMark == 180
                        ? currentX : currentY) + 5*Field.MM_PER_INCH, //extra five inches because she doesn't turn well
                "Approach channel distance to vuMark");
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_DISTAL, "Open distal gripper"), false);
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_PROXIMAL, "Open proximal gripper"), false);

        robot.queueUpBearing(bearingToHarvest, "Bearing to harvest");
        distanceToCraterWall = Field.DISTANCE_TO_CRATER_FROM_CHANNEL[harvestChannel];
        double travelToCraterWall = 0;
        if (bearingToHarvest == 0) {
            travelToCraterWall = currentX + distanceToCraterWall;
        } else if (bearingToHarvest == 180) {
            travelToCraterWall = -currentX + distanceToCraterWall;
        } else if (bearingToHarvest == 90) {
            travelToCraterWall = currentY + distanceToCraterWall;
        } else if (bearingToHarvest == -90) {
            travelToCraterWall = -currentY + distanceToCraterWall;
        }
        robot.queueUpTravel(-travelToCraterWall, "Get near crater");
        robot.queueUpBearing(bearingToHarvest, "Bearing to harvest - correction");
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.Harvest, "Harvest position"));
        robot.queueOperation(new MineralGrabOperation(DISTANCE_TO_TRAVEL_TO_HARVEST, 0.1, "Grab minerals"));
    }

    private void queueAutoHarvestRetry() {
        Match.log("Retrying harvest");
        harvestState = HarvestState.Harvesting;
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.Vertical, "Be vertical"));
        robot.queueUpBearing(bearingToHarvest, "Bearing to harvest");
        robot.queueUpTravel(-distanceToCraterWall, "Get back to crater");
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_DISTAL, "Open distal to grab"), false);
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_PROXIMAL, "Open proximal to grab"), false);
        robot.queueUpBearing(bearingToHarvest, "Bearing to harvest");
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.Harvest, "Harvest position"));
        robot.queueOperation(new MineralGrabOperation(DISTANCE_TO_TRAVEL_TO_HARVEST, 0.1, "Grab minerals"));
    }

    public void queueAutoRetractToVuMark() {
        if (robot.haveAnyMineral()) {
            harvestState = HarvestState.Retracting;
            robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.Vertical, "Go compact"));
            robot.queueUpTravel(distanceToCraterWall - Robot.DISTANCE_TO_TRAVEL_TO_HARVEST, "Get back near VuMark");
            robot.queueUpBearing(bearingToSeeVuMark, "Rotate to see VuMark");
        } else {
            Match.log("Retract: No mineral to deliver, retrying auto-harvest");
            queueAutoHarvestRetry();
        }
    }

    private void queueAutoFindVuMark() {
        harvestState = HarvestState.FindingVuMark;
        double distanceFromWall = Field.FIELD_WIDTH / 2 - Math.abs(channelDistanceFromCenter);
        //if needed, add operation to clear enough off the wall so don't waste time inching up
        if (distanceFromWall < Robot.MINIMUM_DISTANCE_FROM_WALL_TO_SEE_VUMARK) {
            robot.queueUpTravel(-(robot.MINIMUM_DISTANCE_FROM_WALL_TO_SEE_VUMARK - distanceFromWall),
                    "Move away enough to see VuMark");
        }
        robot.queueUpTravelUntilVuMark(-0.1, -0.1, "Find vumark");
        Match.log("Finding VuMark before delivery");
    }

    /**
     * Queue auto delivery of mineral
     * <p>
     * If we don't have both minerals, we do nothing, the driver controlled loop will queue auto-harvest
     * <p>
     * First we find our coordinates by finding the vumark
     * We give up if we cant find a vumark
     * <p>
     * We find the mineral drop-off coordinates based on which alliance we are and which mineral we
     * are carrying.
     * <p>
     * We use trigonometry to find which bearing we should assume and how much we should travel to
     * reach the drop-off point
     * <p>
     * We change bearing and travel the distance to reach the drop-off point, drop the mineral and
     * retract to our starting position
     */
    private void queueAutoDelivery() {
        if (robot.haveAnyMineral()) {
            harvestState = HarvestState.Delivering;
            String vuMark = robot.findVuMark();
            if (vuMark == null) {
                //give up if we are not seeing a vumark
                Match.log("Didn't find a right vumark, can't deliver, vuMark=" + vuMark);
                return;
            }
            double currentX = robot.getCurrentX();
            double currentY = robot.getCurrentY();

            //boolean proximalMineralIsSilver = "Silver".equals(robot.getProximalMineralColor());
            Coordinates currentPosition = new Coordinates(robot.getCurrentX(), robot.getCurrentY(), 0);
            Coordinates dropOffPosition = null;
            double bearingToDropSilverMineral = 0, bearingToDropGoldMineral = 0;
            if (bearingToHarvest == 0) {
                //must be red - footprint
                dropOffPosition = Field.DropOffCoordinates_Red_Footprint;
                bearingToDropGoldMineral = Field.BearingToDropRedFootprintGold;
                bearingToDropSilverMineral = Field.BearingToDropRedFootprintSilver;
            } else if (bearingToHarvest == 180) {
                //must be blue - rover
                dropOffPosition = Field.DropOffCoordinates_Blue_Rover;
                bearingToDropGoldMineral = Field.BearingToDropBlueRoverGold;
                bearingToDropSilverMineral = Field.BearingToDropBlueRoverSilver;
            } else if (bearingToHarvest == 90) {
                //must be blue - craters
                dropOffPosition = Field.DropOffCoordinates_Blue_Craters;
                bearingToDropGoldMineral = Field.BearingToDropBlueCratersGold;
                bearingToDropSilverMineral = Field.BearingToDropBlueCratersSilver;
            } else if (bearingToHarvest == AngleUnit.normalizeDegrees(-90)) {
                //must be red - space
                dropOffPosition = Field.DropOffCoordinates_Red_Space;
                bearingToDropGoldMineral = Field.BearingToDropRedSpaceGold;
                bearingToDropSilverMineral = Field.BearingToDropRedSpaceSilver;
            }
            Autonomous.DistanceAndOrientation distanceAndOrientation =
                    Autonomous.getOrientationAndDistance(currentPosition, dropOffPosition);
            robot.queueUpBearing(distanceAndOrientation.orientation + 180, "Point to drop off");
            robot.queueUpTravel(-distanceAndOrientation.distance, "Approach drop-off position");
            boolean bothMineralsAreSameColor = robot.getDistalMineralColor() == robot.getProximalMineralColor();
/*            if (distalMineralIsSilver) {
                robot.queueUpBearing(bearingToDropSilverMineral, "Point to silver bin");
                dropDistalMineral();
            }
            else if (distalMineralIsGold){
                robot.queueUpBearing(bearingToDropGoldMineral, "Point to gold bin");
                dropDistalMineral();
            }*/
            boolean distalMineralIsSilver = "Silver".equals(robot.getDistalMineralColor());
            boolean distalMineralIsGold = "Gold".equals(robot.getDistalMineralColor());
            boolean proximalMineralIsSilver = "Silver".equals(robot.getProximalMineralColor());
            boolean proximalMineralIsGold = "Gold".equals(robot.getProximalMineralColor());
            if (bothMineralsAreSameColor) {
                robot.queueUpBearing(proximalMineralIsSilver ?
                                bearingToDropSilverMineral : bearingToDropGoldMineral,
                        "Orient to drop both " + (proximalMineralIsSilver ? "Silver" : "Gold") + " minerals");
                dropBothMinerals();
            } else {
                if (proximalMineralIsSilver) {
                    robot.queueUpBearing(bearingToDropSilverMineral, "Point to silver bin");
                    dropProximalMineral();
                } else if (proximalMineralIsGold) {
                    robot.queueUpBearing(bearingToDropGoldMineral, "Point to gold bin");
                    dropProximalMineral();
                }
                if (distalMineralIsSilver) {
                    robot.queueUpBearing(bearingToDropSilverMineral, "Point to silver bin");
                    dropDistalMineral();
                } else if (distalMineralIsGold) {
                    robot.queueUpBearing(bearingToDropGoldMineral, "Point to gold bin");
                    dropDistalMineral();
                }
            }
            robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_VERTICAL, "Go shoulder compact"));
            robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.WINCH_COMPACT, "Go winch compact"));
            robot.queueUpBearing(distanceAndOrientation.orientation + 180, "Point to starting point");
            robot.queueUpTravel(distanceAndOrientation.distance, "Back to starting point");
            robot.queueUpBearing(bearingToSeeVuMark, "Get bearing back to see vuMark");
            robot.queueUpTravelUntilVuMark(0.1, 0.1, "Find vuMark");
        } else {
            Match.log("Auto-Delivery: Didn't find any minerals, retrying auto-harvest");
            queueAutoHarvest();
        }

    }

    /**
     * Drop distal mineral by lowering the arm to the level where it is just above the lander, opening the gripper and then
     * making the arm vertical again
     */
    private void dropDistalMineral() {
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.WINCH_DELIVERY_AUTO, "Winch delivery position"));
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_DELIVERY_AUTO, "Shoulder delivery position"));
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_DISTAL, "Drop distal"));
    }

    /**
     * Drop proximal mineral by lowering the arm to the level where it is just above the lander, opening the gripper and then
     * making the arm vertical again
     */
    private void dropProximalMineral() {
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.WINCH_DELIVERY_AUTO, "Winch delivery position"));
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_DELIVERY_AUTO, "Shoulder delivery position"));
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_PROXIMAL, "Drop proximal"));
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_VERTICAL, "Shoulder vertical position"));
    }

    /**
     * Drop both minerals by lowering the arm to the level where it is just above the lander, opening the gripper and then
     * making the arm vertical again
     */
    private void dropBothMinerals() {
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.WINCH_DELIVERY_AUTO, "Winch delivery position"));
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_DELIVERY_AUTO, "Shoulder delivery position"));
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_PROXIMAL, "Drop proximal"));
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_DISTAL, "Drop distal"));
    }

    public boolean timeToLatch() {
/*
        return false;//skipping latching in prototyping mode
*/
        //allow 20 seconds to latch
        if ((new Date().getTime()) - Match.getInstance().getTeleopStartTime().getTime() > 100000) {
            return true;
        }
        else {
            return false;
        }
    }

    public void abort() {
        this.harvestState = HarvestState.DriverControlled;
    }
}
