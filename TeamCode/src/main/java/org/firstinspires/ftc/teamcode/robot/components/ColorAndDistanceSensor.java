package org.firstinspires.ftc.teamcode.robot.components;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.camera.Camera;

import java.util.Locale;

public class ColorAndDistanceSensor {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    NormalizedColorSensor colorSensor;
    DistanceSensor distanceSensor;

    public ColorAndDistanceSensor(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        // Define and Initialize sensor
        this.colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        this.distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
    }

    public NormalizedRGBA getColors() {
        // Read the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        return colors;
    }

    public String getStatus() {
/*
        float[] hsvValues = new float[3];
        Color.colorToHSV(getColors().toColor(), hsvValues);
        return String.format(Locale.US, "D:%.2f,H:%.3f,S:%.3f,V:%.3f",
                distalDistanceSensor.getDistance(DistanceUnit.CM), hsvValues[0], hsvValues[1], hsvValues[2]);
*/
        NormalizedRGBA colors = getColors();
        return String.format(Locale.US, "D:%.2f,R:%d,G:%d,B:%d",
                distanceSensor.getDistance(DistanceUnit.CM),
                (int) (colors.red*Camera.SCALE_FACTOR),
                (int) (colors.green*Camera.SCALE_FACTOR),
                (int) (colors.blue*Camera.SCALE_FACTOR));
    }

    public boolean isSeeing(Match.Alliance color) {
        if (color == Match.Alliance.Red) {
            return colorSensor.getNormalizedColors().red > .005;
        }
        else {
            return  colorSensor.getNormalizedColors().blue > .005;
        }
    }
}
