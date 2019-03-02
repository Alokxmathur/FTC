package org.firstinspires.ftc.teamcode.game;

/**
 * Created by Silver Titans on 9/16/17.
 */

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.HashMap;

public class Field {
    public static final double MM_PER_INCH        = 25.4f;
    // the width of the FTC field (from the center point to the outer panels)
    public static final double TILE_WIDTH = 24 * MM_PER_INCH;
    public static final double FIELD_WIDTH  = 6 * TILE_WIDTH;
    public static final double TILE_DIAGONAL = Math.sqrt(2 * TILE_WIDTH * TILE_WIDTH);
    public static final Coordinates BracketCoordinates = new Coordinates
            (8.25*MM_PER_INCH, 8.25*MM_PER_INCH, 22*MM_PER_INCH);

    public static final Coordinates SilverDropOffCoordinates_Blue = new Coordinates
            (9*MM_PER_INCH, 9*MM_PER_INCH, 36*MM_PER_INCH);
    public static final Coordinates GoldDropOffCoordinates_Blue = new Coordinates
            (-9*MM_PER_INCH, 9*MM_PER_INCH, 36*MM_PER_INCH);

    public static final Coordinates SilverDropOffCoordinates_Red = new Coordinates
            (-9*MM_PER_INCH, -9*MM_PER_INCH, 36*MM_PER_INCH);
    public static final Coordinates GoldDropOffCoordinates_Red = new Coordinates
            (9*MM_PER_INCH, -9*MM_PER_INCH, 36*MM_PER_INCH);

    public static final Coordinates DropOffCoordinates_Red_Footprint = new Coordinates
            (-20*MM_PER_INCH, -34*MM_PER_INCH, 36*MM_PER_INCH);

    public static final Coordinates DropOffCoordinates_Blue_Rover = new Coordinates
            (20*MM_PER_INCH, 34*MM_PER_INCH, 36*MM_PER_INCH);

    public static final Coordinates DropOffCoordinates_Red_Space = new Coordinates
            (28.5*MM_PER_INCH, -29.5*MM_PER_INCH, 36*MM_PER_INCH);

    public static final Coordinates DropOffCoordinates_Blue_Craters = new Coordinates
            (-28.5*MM_PER_INCH, 29.5*MM_PER_INCH, 36*MM_PER_INCH);

    public static final double BearingToDropRedFootprintSilver = -129;
    public static final double BearingToDropRedFootprintGold = -135;

    public static final double BearingToDropRedSpaceSilver = -37;
    public static final double BearingToDropRedSpaceGold = -48;

    public static final double BearingToDropBlueRoverSilver = 51;
    public static final double BearingToDropBlueRoverGold = 45;

    public static final double BearingToDropBlueCratersSilver = 143;
    public static final double BearingToDropBlueCratersGold = 132;

    public static final double[] DISTANCE_TO_CRATER_FROM_CHANNEL = {
            20*MM_PER_INCH,
            19.5*MM_PER_INCH,
            18.5*MM_PER_INCH,
            18*MM_PER_INCH,
            17.5*MM_PER_INCH
    };

    // the height of the center of the target image above the floor
    public static final float VUMARKS_HEIGHT   = (float) (5.5 * MM_PER_INCH);


    public static final Coordinates redDepotCoordinates = new Coordinates(
            60 * MM_PER_INCH,
            -60 * MM_PER_INCH,
            0);
    public static final Coordinates blueDepotCoordinates = new Coordinates(
            -60 * MM_PER_INCH,
            60 * MM_PER_INCH,
            0);

    public static final Coordinates centralMineralRawCoordinates = new Coordinates(
            1000, 1000, 0);

    public static final double DISTANCE_TO_CENTRAL_MINERAL_FROM_BRACKET = 1150;

    public enum Wall {
        Front, Rear, Left, Right
    }

    public enum LaunchedFacing {
        Depot, Crater
    }

    //the positions of the four vumarks on the field
    private static HashMap<Wall, OpenGLMatrix> pictographPositions =
            new HashMap<>();

    public Field() {
    }

    public OpenGLMatrix getPictographLocation(Wall wall, Match.Alliance alliance) {
        return pictographPositions.get(wall);
    }

    public static Coordinates getDepot() {
        if (Match.getInstance().getAlliance() == Match.Alliance.Red) {
            return redDepotCoordinates;
        }
        else  {
            return blueDepotCoordinates;
        }
    }
}