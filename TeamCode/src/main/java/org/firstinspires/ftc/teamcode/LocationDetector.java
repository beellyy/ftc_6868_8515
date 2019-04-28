package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public class LocationDetector {

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AegblNT/////AAABmQbC4gavNk/2oo4dxCeTz2iGYV+lkOtnpdmw2WqsQ5CjQ0t6bRbHhpWDKJdkCBBg3B1Lt/JXq+Na0bEv0VAFZwwnG3QK0RAmXEnHlSzjRxeb2c7PGaUQfwXIerK6R8Wc9HQfkR4Pgrw/9Ux0/QemHNU0nogkiIoxY2aLsYqsFBtUUnXVq6QxQ4MQ5ITOXI4dz+r/A1304CIrDAVEqemBGsu21xBdVJWFWuct9v1LQ5EAbHyX8nnf6uF2ybqHL8sJayYcfbdk5ZaDDUx/O/kFLJHTF7wTkAHLCnMvOndHFqoLpc7SUD0zliqGP/sI5t1hlxHl9ntQQDTlefoRE+eItIogKXNbTUMawanzTbKZYzSd";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    /**
     * {@link #vuforia} is the variable we will use to store our fieldInstance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia = null;
    private List<VuforiaTrackable> allTrackables = null;
    private RobotRoverRuckus robot = null;
    private RobotLocation location = new RobotLocation();
    private HardwareMap hardwareMap = null;
    private Telemetry telemetry = null;


    public LocationDetector() {
    }

    public void init(RobotRoverRuckus robotRoverRuckus) {
        robot = robotRoverRuckus;
        hardwareMap = robot.hardwareMap;
        telemetry = robot.telemetry;
        init();
    }

    public void start() {

    }

    public void init() {
        try {
            vuforia = VuforiaFactory.initForLocate(hardwareMap);

            // Load the data sets that for the trackable objects. These particular data
            // sets are stored in the 'assets' part of our application.
            VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
            VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
            blueRover.setName("Blue-Rover");
            VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
            redFootprint.setName("Red-Footprint");
            VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
            frontCraters.setName("Front-Craters");
            VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
            backSpace.setName("Back-Space");

            // For convenience, gather together all the trackable objects in one easily-iterable collection */
            allTrackables = new ArrayList<VuforiaTrackable>();
            allTrackables.addAll(targetsRoverRuckus);

            OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                    .translation(0, mmFTCFieldWidth, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ,
                            DEGREES, 90, 0, 0));
            blueRover.setLocation(blueRoverLocationOnField);

            OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                    .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ,
                            DEGREES, 90, 0, 180));
            redFootprint.setLocation(redFootprintLocationOnField);

            OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                    .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ,
                            DEGREES, 90, 0, 90));
            frontCraters.setLocation(frontCratersLocationOnField);

            OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                    .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ,
                            DEGREES, 90, 0, -90));
            backSpace.setLocation(backSpaceLocationOnField);


            OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                    .translation(robot.CAMERA_FORWARD_DISPLACEMENT,
                            robot.CAMERA_LEFT_DISPLACEMENT,
                            robot.CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                            CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

            /**  Let all the trackable listeners know where the phone is.  */
            for (VuforiaTrackable trackable : allTrackables) {
                if (vuforia.getCameraName() != null) {
                    ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(vuforia.getCameraName(), phoneLocationOnRobot);
                } else {

                    ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, VuforiaLocalizer.CameraDirection.BACK);
                }

            }

            /** Wait for the game to begin */
            telemetry.addData(">", "Press Play to start tracking");
            telemetry.update();


            /** Start tracking the data sets we care about. */
            targetsRoverRuckus.activate();


        } catch (Exception ex) {

        }

    }

    public RobotLocation searchLocation() {
        // check all the trackable target to see which one (if any) is visible.
        robot.location.found = false;
        targetVisible = false;
        try {
            String targetName = "";
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    targetName = trackable.getName();
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch,
                        translation.get(1) / mmPerInch,
                        translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f",
                        rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                robot.location.x = translation.get(0);
                robot.location.y = translation.get(1);
                robot.location.heading = rotation.thirdAngle;
                robot.location.targetName = targetName;
                robot.location.found = true;
            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();

        } catch (Exception ex) {

        }

        return location;

    }


    public void stop() {

    }


}
