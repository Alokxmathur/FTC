package org.firstinspires.ftc.teamcode.robot.modes.autonomous;

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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Facing Crater 2x Sampling", group = "Autonomous")
//@Disabled
public class AutonomousFacingCrater2xSampling extends Autonomous {
    @Override
    public void runOpMode() {
        super.setExtraSampling(true);
        super.runOpMode();
    }
}
