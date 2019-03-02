package org.firstinspires.ftc.teamcode.robot.modes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.game.Coordinates;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.FrontTrap;
import org.firstinspires.ftc.teamcode.robot.components.PickerArm;
import org.firstinspires.ftc.teamcode.robot.operations.FrontTrapOperation;
import org.firstinspires.ftc.teamcode.robot.operations.LatchOperation;
import org.firstinspires.ftc.teamcode.robot.operations.ThroneOperation;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Created by Silver Titans on 9/19/18.
 * <p>
 * Here's our approach for the autonomous period
 * Steps
 * Steps
 * 1. Find Depot
 * 2. Land
 * 3. Sample
 * 3. Reach VuMark
 * 5. Travel to the depot
 * 6. Deposit marker
 * 7. Travel to our crater and park there.
 */

public class Autonomous extends LinearOpMode {

    public static final float DEGREES_BETWEEN_SAMPLING_MINERALS = 27f;
    public static final double RADIANS_BETWEEN_SAMPLING_MINERALS
            = Math.toRadians(DEGREES_BETWEEN_SAMPLING_MINERALS);
    public static final double RETRACTION_FROM_CENTRAL_MINERAL = 250;
    public static final double DISTANCE_TO_CLEAR_BRACKET = 50; //mms
    public static final double TURN_REQUIRED_TO_SEE_RIGHT_MINERAL = 15;
    public static final double DISTANCE_TO_VUMARKS_FROM_CENTER = 38 * Field.MM_PER_INCH;

    private boolean extraSampling = false;

    public enum GoldMineralLocation {
        Left, Center, Right
    }

    public void setExtraSampling(boolean extraSamplingRequest) {
        extraSampling = extraSamplingRequest;
    }

    @Override
    public void runOpMode() {
        Match match = Match.getNewInstance();
        try {
            match.init();
            Robot robot = match.getRobot();
            robot.init(hardwareMap, this.telemetry, match);
            AutoTransitioner.transitionOnStop(this, "Phoebe: Driver Controlled");
            //wait to begin
            waitForStart();
            match.setStart();

            Match.log("--------> Starting autonomous");

            boolean goldDetermined = false;

            boolean landed = false;
            boolean queuedLanding = false;

            boolean latchLeveled = false;

            boolean sampled = false;
            boolean queuedSampling = false;

            boolean foundVuMark = false;
            boolean queuedFindingVuMark = false;

            boolean claimed = false;
            boolean queuedClaiming = false;

            //presume we have extra sampled if are asked not to do it
            boolean xtraSampled = false;
            boolean queuedXtraSampling = false;

            boolean parked = false;
            boolean queuedParking = false;

            double distanceToKnockOffCentralMineral = Field.DISTANCE_TO_CENTRAL_MINERAL_FROM_BRACKET
                    - Robot.WHEEL_OFFSET_FROM_LATCH
                    - FrontTrap.ARM_EXTENSION_BEYOND_WHEELS
                    - DISTANCE_TO_CLEAR_BRACKET;
            double distanceToKnockOffEdgeMineral = distanceToKnockOffCentralMineral
                    / Math.cos(RADIANS_BETWEEN_SAMPLING_MINERALS);
            double distanceToRetractFromEdgeMineral = RETRACTION_FROM_CENTRAL_MINERAL /
                    Math.cos(Math.toRadians(DEGREES_BETWEEN_SAMPLING_MINERALS));
            double extraDistanceToReachViewMark = (distanceToKnockOffCentralMineral - RETRACTION_FROM_CENTRAL_MINERAL)
                    * Math.tan(RADIANS_BETWEEN_SAMPLING_MINERALS);

            // make sure the imu gyro is calibrated before continuing.
            while (!isStopRequested() && !robot.isGyroCalibrated()) {
                sleep(50);
                idle();
            }

            GoldMineralLocation goldLocation = GoldMineralLocation.Left;
            double bearingToKnockOffMineral = 0;
            double distanceToKnockOffMineral = 0;
            double distanceToRetract = 0;
            double distanceToVuMark = DISTANCE_TO_VUMARKS_FROM_CENTER;

            while (opModeIsActive() && !isStopRequested()) {
                if (!landed) {
                    //land Phoebe
                    if (!queuedLanding) {
                        this.queueLanding(robot);
                        queuedLanding = true;
                    }
                    landed = robot.operationsCompleted();
                } else if (!goldDetermined) {
                    /**
                     * When we start, we should be seeing the right and center minerals
                     * If we are seeing a gold mineral we check to see if it is on the right or center
                     * If we are not seeing gold mineral it must be to our left.
                     */
                    boolean foundGold = robot.sampledGold();
                    if (foundGold) {
                        if (robot.getGoldRectangle().x < 600) {
                            //gold is on the right
                            goldLocation = GoldMineralLocation.Right;
                        } else {
                            //gold is centered
                            goldLocation = GoldMineralLocation.Center;
                        }
                    } else {
                        //gold is on the left
                        goldLocation = GoldMineralLocation.Left;
                    }
                    switch (goldLocation) {
                        case Left: {
                            bearingToKnockOffMineral = DEGREES_BETWEEN_SAMPLING_MINERALS;
                            distanceToKnockOffMineral = distanceToKnockOffEdgeMineral;
                            distanceToRetract = distanceToRetractFromEdgeMineral;
                            distanceToVuMark -= extraDistanceToReachViewMark;
                            break;
                        }
                        case Center: {
                            bearingToKnockOffMineral = 0;
                            distanceToKnockOffMineral = distanceToKnockOffCentralMineral;
                            distanceToRetract = RETRACTION_FROM_CENTRAL_MINERAL;
                            break;
                        }
                        case Right: {
                            //extra for left mineral
                            bearingToKnockOffMineral = -(DEGREES_BETWEEN_SAMPLING_MINERALS);
                            distanceToKnockOffMineral = distanceToKnockOffEdgeMineral;
                            distanceToRetract = distanceToRetractFromEdgeMineral;
                            distanceToVuMark += extraDistanceToReachViewMark;
                            break;
                        }
                    }
                    Match.log("Gold on " + goldLocation);
                    goldDetermined = true;
                } else if (!sampled) {
                    if (!queuedSampling) {
                        queuedSampling = true;
                        this.queueSampling(robot, bearingToKnockOffMineral, distanceToKnockOffMineral, distanceToRetract);
                    }
                    sampled = robot.operationsCompleted();
                } else if (!latchLeveled) {
                    robot.queueOperation(
                            new LatchOperation(LatchOperation.LatchOperationType.Level, "Level latch"), false);
                    //don't wait for operations to complete - they are on the secondary thread
                    latchLeveled = true;
                } else if (!foundVuMark) {
                    if (!queuedFindingVuMark) {
                        queuedFindingVuMark = true;
                        this.queueFindVuMark(robot, distanceToVuMark);
                    }
                    foundVuMark = robot.operationsCompleted();
                } else if (!claimed) {
                    if (!queuedClaiming) {
                        queuedClaiming = true;
                        this.queueClaiming(robot);
                    } else {
                        claimed = robot.operationsCompleted();
                    }
                } else if (!parked) {
                    if (!queuedParking) {
                        queuedParking = true;
                        this.queueParking(robot);
                    } else {
                        parked = robot.operationsCompleted();
                    }
                } else {
                    //all done
                }
                //match.updateTelemetry(telemetry, String.format("Found vu: " + foundVuMark + "Depot:" + sampled));
                //this.sleep(100);
            }
            robot.stop();
            Match.log("Stopped autonomous  <-------------");
        } catch (Throwable e) {
            StringWriter stringWriter = new StringWriter();
            PrintWriter printWriter = new PrintWriter(stringWriter);
            e.printStackTrace(printWriter);
            telemetry.addLine(stringWriter.toString());
            telemetry.update();
            Match.log(stringWriter.toString());
            try {
                Thread.sleep(20000);
            } catch (InterruptedException ex) {
            }
        }
    }

    private void queueLanding(Robot robot) {
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.INITIAL,
                "Get arm released"), false);
        robot.queueOperation(
                new LatchOperation(LatchOperation.LatchOperationType.Unlock,
                        "Unlock to lower ourselves"));
        robot.queueOperation(
                new LatchOperation(LatchOperation.LatchOperationType.Raise,
                        "Raise latch to clear bracket"));
        robot.queueUpTravel(DISTANCE_TO_CLEAR_BRACKET, "Move away from lander");
        robot.queueUpBearing(-TURN_REQUIRED_TO_SEE_RIGHT_MINERAL, "turn to see two minerals");
    }

    /**
     * Sample gold
     * Assumes we already have the sampling arm lowered
     *
     * @param robot
     * @param bearingToKnockOffMineral
     * @param distanceToKnockOffMineral
     * @param distanceToRetract
     */
    private void queueSampling(Robot robot, double bearingToKnockOffMineral, double distanceToKnockOffMineral, double distanceToRetract) {
        robot.queueOperation(new FrontTrapOperation(FrontTrapOperation.TrapOperationType.Open,
                "Open trap"), false);
        robot.queueUpBearing(bearingToKnockOffMineral, "Face gold");
        robot.queueUpTravel(distanceToKnockOffMineral, "Knocking off gold mineral");
        robot.queueOperation(
                new FrontTrapOperation(FrontTrapOperation.TrapOperationType.Close, "Spring trap"));
        robot.queueUpTravel(-distanceToRetract,
                "Retracting after knocking off gold mineral");
    }

    /**
     * Travel to VuMark and find it
     *
     * @param robot
     * @param distanceToVuMark
     */
    private void queueFindVuMark(Robot robot, double distanceToVuMark) {
        robot.queueUpBearing(90, "Pointing to vuMark");
        robot.queueUpTravelIMU(distanceToVuMark,
                90, "Get closer to vuMark");
        robot.queueUpBearing(45, "Turn to face vumark");
        robot.queueUpTravelUntilVuMark(0, 0, "Find vumark");
    }

    /**
     * Reach depot and claim after seeing the initial vuMark
     *
     * @param robot
     */
    private void queueClaiming(Robot robot) {
        double bearingToSeeVuMark = AutonomousHelper.bearingToSeeVuMark();
        double initialBearing = robot.getBearing();
        double travelToGetNearWall;
        double travelToReachDepot;

        if (Match.getInstance().getAlliance() == Match.Alliance.Red) {
            if (Match.getInstance().getLaunchedFacing() == Field.LaunchedFacing.Crater) {
                travelToGetNearWall = Field.FIELD_WIDTH / 2 - Robot.WHEEL_OFFSET_FROM_FRONT + 2*Field.MM_PER_INCH
                        - Math.abs(robot.getCurrentY());
                travelToReachDepot =
                        (Field.FIELD_WIDTH / 2) - (Field.TILE_WIDTH/2) - Robot.WHEEL_OFFSET_FROM_FRONT
                                - robot.getCurrentX();
            } else {
                travelToGetNearWall = Field.FIELD_WIDTH / 2 - Robot.WHEEL_OFFSET_FROM_FRONT
                        - Math.abs(robot.getCurrentX() + 2*Field.MM_PER_INCH);
                travelToReachDepot =
                        (Field.FIELD_WIDTH / 2) - Field.TILE_WIDTH/2 - Robot.WHEEL_OFFSET_FROM_FRONT
                                + robot.getCurrentY();
            }
        }
        else {
            if (Match.getInstance().getLaunchedFacing() == Field.LaunchedFacing.Crater) {
                travelToGetNearWall = Field.FIELD_WIDTH/2 - Robot.WHEEL_OFFSET_FROM_FRONT + 2*Field.MM_PER_INCH
                        - Math.abs(robot.getCurrentY());
                travelToReachDepot=
                        (Field.FIELD_WIDTH / 2) - Field.TILE_WIDTH/2 - Robot.WHEEL_OFFSET_FROM_FRONT
                                + robot.getCurrentX();
            }
            else {
                travelToGetNearWall = Field.FIELD_WIDTH/2 - Robot.WHEEL_OFFSET_FROM_FRONT + 2*Field.MM_PER_INCH
                        - Math.abs(robot.getCurrentX());
                travelToReachDepot=
                        (Field.FIELD_WIDTH / 2) - Field.TILE_WIDTH/2 - Robot.WHEEL_OFFSET_FROM_FRONT + 2*Field.MM_PER_INCH
                                - robot.getCurrentY();
            }
        }
        //compensate for being off the desired bearing to the wall
        double bearingOffset = Math.toRadians(initialBearing - bearingToSeeVuMark);
        if (bearingOffset != 0) {
            travelToGetNearWall = Math.abs(travelToGetNearWall / Math.cos(bearingOffset));
            if (Match.getInstance().getLaunchedFacing() == Field.LaunchedFacing.Crater) {
                travelToReachDepot -= (travelToGetNearWall * Math.sin(bearingOffset));
            }
            else {
                travelToReachDepot += (travelToGetNearWall * Math.sin(bearingOffset));
            }
        }

        robot.queueUpTravel(travelToGetNearWall, "Get near wall");
        double bearing = AutonomousHelper.getCraterToDepotBearing();
        robot.queueUpBearing(bearing, "Correct ");
        robot.queueUpTravelIMU(travelToReachDepot,
                bearing, "Get to depot");
        robot.queueOperation(new FrontTrapOperation(FrontTrapOperation.TrapOperationType.Open, "Open trap"));
        robot.queueUpBearing(bearing, "Turn rear to crater");
        robot.queueUpTravel(-8 * Field.MM_PER_INCH, "Retract to release gold mineral");
        robot.queueOperation(new FrontTrapOperation(FrontTrapOperation.TrapOperationType.Close, "Close trap"));
        robot.queueOperation(new ThroneOperation(ThroneOperation.ThroneOperationType.Lower, "Drop marker"));
    }

    private void queueParking(Robot robot) {
        robot.queueOperation(new ThroneOperation(ThroneOperation.ThroneOperationType.Raise, "Raise throne back"), false);
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_DISTAL, "Open"), false);
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_PROXIMAL, "Open"), false);
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.Harvest, "Harvest"), false);
        double bearing = AutonomousHelper.getCraterToDepotBearing();
        robot.queueUpBearing(bearing, "Turn rear to crater");
        robot.queueUpTravelIMU(-2.5 * Field.TILE_WIDTH - 4*Field.MM_PER_INCH, bearing, "Get to crater");
/*
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.Harvest, "Harvest"), false);
        robot.queueOperation(new PickerOperation(PickerOperation.PickerOperationType.INTAKE_ON, "Start intake"), false);
*/
    }

    /**
     * Method to figure out the distance and orientation from point1 to point2
     *
     * @param point1
     * @param point2
     * @return DistanceAndOrientation
     */
    public static DistanceAndOrientation getOrientationAndDistance(Coordinates point1, Coordinates point2) {
        double xDiff = point2.getX() - point1.getX(), yDiff = point2.getY() - point1.getY();
        DistanceAndOrientation distanceAndOrientation = new DistanceAndOrientation();
        distanceAndOrientation.distance = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
        if (xDiff != 0) {
            if (yDiff != 0) {
                distanceAndOrientation.orientation = Math.toDegrees(Math.atan(Math.abs(yDiff) / Math.abs(xDiff)));
                if (yDiff > 0) {
                    if (xDiff < 0) {
                        distanceAndOrientation.orientation = 180 - distanceAndOrientation.orientation;
                    }
                } else {
                    if (xDiff > 0) {
                        distanceAndOrientation.orientation = -distanceAndOrientation.orientation;
                    } else {
                        distanceAndOrientation.orientation = 180 + distanceAndOrientation.orientation;
                    }
                }
            } else {
                distanceAndOrientation.orientation = xDiff > 0 ? 0 : -180;
            }
        } else {
            distanceAndOrientation.orientation = yDiff > 0 ? 90 : -90;
        }
        Match.log("From " + point1.getXInInches() + ", " + point1.getYInInches()
                + " to " + point2.getXInInches() + ", " + point2.getYInInches() + ", angle="
                + distanceAndOrientation.orientation + ", d=" + distanceAndOrientation.distance);
        return distanceAndOrientation;
    }

    public static DistanceAndOrientation getOrientationAndDistanceToDrop(Coordinates point1, Coordinates point2) {
        DistanceAndOrientation fromCenterToPoint2 = getOrientationAndDistance(point1, point2);
        double additionalDegrees = Math.asin(PickerArm.OFFSET_FROM_BASE / fromCenterToPoint2.distance);
        DistanceAndOrientation distanceAndOrientation = new DistanceAndOrientation();
        distanceAndOrientation.distance = fromCenterToPoint2.distance * Math.cos(additionalDegrees)
                - PickerArm.GRIPPER_HORIZONTAL_EXTENSION_AT_DELIVERY;
        distanceAndOrientation.orientation = fromCenterToPoint2.orientation + Math.toDegrees(additionalDegrees);
        Match.log("Drop from " + point1.getXInInches() + ", " + point1.getYInInches()
                + " to " + point2.getXInInches() + ", " + point2.getYInInches() + ", angle="
                + distanceAndOrientation.orientation + ", d=" + distanceAndOrientation.distance);

        return distanceAndOrientation;
    }

    public static class DistanceAndOrientation {
        public DistanceAndOrientation() {
        }

        public double distance, orientation;
    }
}

