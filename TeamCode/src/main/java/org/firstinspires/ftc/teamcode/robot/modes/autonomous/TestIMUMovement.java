package org.firstinspires.ftc.teamcode.robot.modes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Created by Silver Titans on 9/19/18.
 * <p>
 * Here's our approach for the autonomous period
 * Steps
 * 1. Unlatch
 * 2. Find gold mineral
 * 3. Rotate 45 degrees to find a vuMark - this shuld tell us which alliance we are and which
 * whether we started from the silver or the gold lander storage area. It should also tell us
 * our current position.
 * 4. Travel to the knock off the gold mineral.
 * 5. Travel to the depot
 * 6. Deposit marker
 * 7. Travel to our crater and park there.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Test 80cm travel IMU", group = "Autonomous")
@Disabled
public class TestIMUMovement extends LinearOpMode {

    @Override
    public void runOpMode() {
        Match match = Match.getNewInstance();
        try {
            match.init();
            Robot robot = match.getRobot();
            robot.init(hardwareMap, this.telemetry, match);
            //wait to begin
            // make sure the imu gyro is calibrated before continuing.
            while (!isStopRequested() && !robot.isGyroCalibrated())
            {
                sleep(50);
                idle();
            }
            waitForStart();

            Match.log("--------> Starting 80cm IMU autonomous");
            robot.queueUpTravelIMU(800, robot.getBearing(),"Testing 80cm IMU movement");

            while (opModeIsActive() && !isStopRequested()) {
                match.updateTelemetry(telemetry, "Doing nothing");
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
}
