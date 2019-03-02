package org.firstinspires.ftc.teamcode.game;

/**
 * Created by Silver Titans on 11/9/17.
 */

public class Lander {
    Coordinates coordinates;

    public Lander(Coordinates coordinates) {
        this.coordinates = coordinates;
    }
    public Lander(float x, float y, float z) {
        this(new Coordinates(x, y, z));
    }

    public Coordinates getCoordinates() {
        return this.coordinates;
    }
}
