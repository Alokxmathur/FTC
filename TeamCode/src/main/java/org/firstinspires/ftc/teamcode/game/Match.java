package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.Date;

/**
 * Created by Silver Titans on 9/19/17.
 */

public class Match {

    static Match match;
    public static String TEAM = "SilverTitans";
    private Robot robot = null;
    private Field field = null;
    private Alliance alliance;
    private Field.LaunchedFacing startingLaunchedFacing;
    private Date startTime = new Date();
    private Date teleopStartTime = new Date();

    synchronized public static Match getNewInstance() {
        match = new Match();
        return match;
    }

    synchronized public static Match getInstance() {
        if (match == null) {
            return getNewInstance();
        }
        else {
            return match;
        }
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public void setStart() {
        this.startTime = new Date();
    }

    public long getElapsed() {
        return new Date().getTime() - startTime.getTime();
    }

    public Date getTeleopStartTime() {
        return teleopStartTime;
    }

    public void setTeleopStartTime(Date teleopStartTime) {
        this.teleopStartTime = teleopStartTime;
    }

    public enum Alliance {
        Red, Blue
    }

    public Field.LaunchedFacing getLaunchedFacing() {
        return startingLaunchedFacing;
    }

    public void setStartingLaunchedFacing(Field.LaunchedFacing launchedFacing) {
        startingLaunchedFacing = launchedFacing;
    }

    synchronized public Robot getRobot() {
        if (robot == null) {
            robot = new Robot();
            log ("Created new robot instance");
        }
        return robot;
    }

    public Field getField()
    {
        if (field == null) {
            field = new Field();
            log("Created new field instance");
        }
        return field;
    }

    public void init() {
        robot = new Robot();
        field = new Field();
    }

    public static void log(String s) {
        RobotLog.a(TEAM + ":" + s);
    }

    /**
     * Give the driver station a state of the union
     *
     * @param telemetry
     */
    public void updateTelemetry(Telemetry telemetry, String status) {
        if (robot != null && field != null) {
            // Send telemetry message to signify robot context;
            telemetry.addData("State:", status);
            telemetry.addData("M", robot.getMotorStatus());
            //telemetry.addData("S", robot.getSensorStatus());
            telemetry.addData("G", robot.getGoldSampleStatus());

            telemetry.addData("Pos", robot.getPositionStatus());
            telemetry.addData("VBearing", robot.getVuforiaBearingStatus());
            telemetry.addData("IBearing", robot.getIMUBearingStatus());
            telemetry.addData("Latch", robot.getLatchStatus());
            telemetry.addData("Picker", robot.getPickerArmStatus());
            telemetry.addData("Color", robot.getColorStatus());
        }
        else {
            telemetry.addData("Context", "Robot not initialized");
        }
        telemetry.update();
    }
}
