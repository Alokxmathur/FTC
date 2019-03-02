package org.firstinspires.ftc.teamcode.robot.components.camera;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import org.firstinspires.ftc.teamcode.game.Mineral;
import org.firstinspires.ftc.teamcode.robot.operations.CameraOperation;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * Created by silver titans on 9/19/17.
 */

public class Camera {
    public static final String RED_FOOTPRINT = "Red-Footprint";
    public static final String FRONT_CRATERS = "Front-Craters";
    public static final String BACK_SPACE = "Back-Space";
    public static final String BLUE_ROVER = "Blue-Rover";
    public static final int SCALE_FACTOR = 255;
    public static final int ATTEMPTS_TO_SEE_GOLD = 1;

    VuforiaLocalizerImpl vuforiaLocalizer;
    Telemetry telemetry;
    OpenGLMatrix lastLocation = new OpenGLMatrix();
    ArrayList<VuforiaTrackable> allTrackables = new ArrayList<>();

    // Level and Upper bounds for range checking in HSV color space
    public static final Scalar goldLowerBounds = new Scalar(11, 230, 219);
    public static final Scalar goldUpperBounds = new Scalar(36, 280, 269);

    public static final Scalar silverLowerBounds = new Scalar(16, -25, 230);
    public static final Scalar silverUpperBounds = new Scalar(41, 25, 280);

    //mineral must be below this line
    public static final double SAMPLE_Y_COORDINATE_THRESHOLD = 100;

    // Cache
    Mat initialFrame = new Mat();
    Mat mPyrDownMat = new Mat();
    Mat mHsvMat = new Mat();
    Mat mMask = new Mat();
    Mat mDilatedMask = new Mat();
    Mat mHierarchy = new Mat();

    // Minimum contour area in percent for contours filtering
    private static double mMinContourArea = 0.05;
    private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();


    static {
        // load the native OpenCV library
        System.loadLibrary("opencv_java3");

    }

    public OpenGLMatrix getLastKnownPosition() {
        return this.lastLocation;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry, OpenGLMatrix phoneLocationOnRobot) {

        this.telemetry = telemetry;
        /*
         * To start up Camera, tell it the view that we wish to use for camera monitor
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AaNwnOj/////AAAAGVvurMxJGkBBhhZ5YsDdn5wdUjQczk6Hz4Y5hGMiJsFzr3rdezyRHTLmGu3eoRrCryIH9Ht7OTuCZqInYAOeC4gc+loxV+u1i6Uz1oiKFPyfQq5X1vHN0JAowJDGwSOvPZwy4XI+uIJslp7SDfqu0PVkqAJF/kvfsByqVAcadpCH4ETryNepR3fyVn3JtTUF0nopBRZvS4zwLkudrxt9a6ZCuM7ct4t64H44KzrFlKYcBCVBWzPurY4M66n38v98gf6TqgKdbcUmIBRgpImm1JNQtdCjQ7GhYGCiZLm6NkHO2S2BpvDO2IrXemImGk/Ou2L9eLmT/pC5EmFbRuFHtMZkK0+wcoGDHgrf0iKAMy/v\n";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the rear (HiRes) camera.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;


        //  Instantiate the Camera engine
        vuforiaLocalizer = new VuforiaLocalizerImpl(parameters);
        vuforiaLocalizer.enableConvertFrameToBitmap();
        vuforiaLocalizer.setFrameQueueCapacity(1);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforiaLocalizer.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName(BLUE_ROVER);
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName(RED_FOOTPRINT);
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName(FRONT_CRATERS);
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName(BACK_SPACE);

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, (float) Field.FIELD_WIDTH / 2, Field.VUMARKS_HEIGHT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, (float) -Field.FIELD_WIDTH / 2, Field.VUMARKS_HEIGHT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation((float) -Field.FIELD_WIDTH / 2, 0, Field.VUMARKS_HEIGHT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation((float) Field.FIELD_WIDTH / 2, 0, Field.VUMARKS_HEIGHT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener())
                    .setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();
    }

    /**
     * Tries to find one of the four vuMarks on the walls of the field.
     *
     * @return
     */
    public String findTarget() {
        // check all the trackable target to see which one (if any) is visible.
        String foundTarget = null;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                foundTarget = trackable.getName();

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform =
                        ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                    Match.log("Robot location from target " + trackable.getName() + "=" + getPosition() + ", " + getBearing());
                }
                break;
            }
        }

        return foundTarget;
    }

    public float getCurrentX() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            return
                    translation.get(0);
        } else {
            return -1;
        }
    }

    public float getCurrentY() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            return
                    translation.get(1);
        } else {
            return -1;
        }
    }

    public float getCurrentZ() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            return
                    translation.get(2);
        } else {
            return -1;
        }
    }

    public float getCurrentBearing() {
        if (lastLocation != null) {
            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            return rotation.thirdAngle;
        } else {
            return 0;
        }
    }

    public String getPosition() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            return String.format("{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / Field.MM_PER_INCH,
                    translation.get(1) / Field.MM_PER_INCH,
                    translation.get(2) / Field.MM_PER_INCH);
        } else {
            return "Target not found";
        }
    }

    public String getBearing() {
        if (lastLocation != null) {
            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            return String.format("{R, P, B} = %.0f, %.0f, %.0f",
                    rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            return "Target not found";
        }
    }

    /**
     * attempt to sample gold mineral
     *
     * @return The rectangle representing gold mineral if found
     */
    public Rect sampleGold() {
        /*To access the image: we need to iterate through the images of the frame object:*/
        VuforiaLocalizer.CloseableFrame vuforiaFrame = null; //takes the frame at the head of the queue
        for (int tries = 0; tries < ATTEMPTS_TO_SEE_GOLD; tries++) {
            try {
                vuforiaFrame = vuforiaLocalizer.getFrameQueue().take();

                Image image = null;
                long numImages = vuforiaFrame.getNumImages();
                for (int i = 0; i < numImages; i++) {
                    Image checkedImage = vuforiaFrame.getImage(i);
                    int format = checkedImage.getFormat();
                    if (format == PIXEL_FORMAT.RGB565) {
                        image = checkedImage;
                        break;
                    }//if
                }//for
                if (image == null) {
                    Match.getInstance().getRobot().setState( "Unable to get image from vuForia camera out of " + numImages);
                    return null;
                }

                Bitmap bitmap = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);
                bitmap.copyPixelsFromBuffer(image.getPixels());

                //convert bitmap to Mat
                Utils.bitmapToMat(bitmap, initialFrame);

                //pyramid down twice
                Imgproc.pyrDown(initialFrame, mPyrDownMat);
                Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);

                //convert to HSV
                Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

                //apply mask
                Core.inRange(mHsvMat, goldLowerBounds, goldUpperBounds, mMask);
                //dilate
                Imgproc.dilate(mMask, mDilatedMask, new Mat());

                List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
                //find contours
                Imgproc.findContours(mDilatedMask, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                // Find max contour area
                double maxArea = 0;
                Iterator<MatOfPoint> each = contours.iterator();
                while (each.hasNext()) {
                    MatOfPoint wrapper = each.next();
                    double area = Imgproc.contourArea(wrapper);
                    if (area > maxArea)
                        maxArea = area;
                }

                // Filter contours by area and resize to fit the original image size
                mContours.clear();
                each = contours.iterator();
                while (each.hasNext()) {
                    MatOfPoint contour = each.next();
                    if (Imgproc.contourArea(contour) > mMinContourArea*maxArea) {
                        Core.multiply(contour, new Scalar(4,4), contour);
                        Rect boundingRectangle = Imgproc.boundingRect(contour);
                        if (boundingRectangle.y <= SAMPLE_Y_COORDINATE_THRESHOLD) {
                            Match.log("Found contour matching criteria " + boundingRectangle.toString());
                            return boundingRectangle;
                        }
                    }
                }
                Thread.sleep(100);
            } catch (Exception e) {
                Match.log("Exception " + e + " in finding gold");
            }
        }
        return null;
    }

    public void handleOperation(CameraOperation operation) {
        if (operation.getCameraOperationType() == CameraOperation.CameraOperationType.FLASH_ON) {
            if (CameraDevice.getInstance().setFlashTorchMode(true)) {
                Match.log("Turned on camera flash");
            } else {
                Match.log("Unable to turn on camera flash");
            }
        }
        else if (operation.getCameraOperationType() == CameraOperation.CameraOperationType.FLASH_OFF) {
            if (CameraDevice.getInstance().setFlashTorchMode(false)) {
                Match.log("Turned off camera flash");
            } else {
                Match.log("Unable to turn off camera flash");
            }
        }
    }
}
