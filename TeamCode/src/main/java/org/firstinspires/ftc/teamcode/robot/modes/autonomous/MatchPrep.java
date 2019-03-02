package org.firstinspires.ftc.teamcode.robot.modes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Created by Silver Titans on 9/19/18.
 * <p>
 This autonomous prepares for match. Should be run before match
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Unlock/Lock", group = "Autonomous")
//@Disabled
public class MatchPrep extends LinearOpMode {

    @Override
    public void runOpMode() {
        Match match = Match.getNewInstance();
        try {
            match.init();
            Robot robot = match.getRobot();
            robot.init(hardwareMap, telemetry, match);
            Match.getInstance().updateTelemetry(telemetry, "Press start after lowering latch");
            robot.unlockLatch();

            //wait to begin
            waitForStart();
            robot.lockLatch();

            robot.stop();
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
