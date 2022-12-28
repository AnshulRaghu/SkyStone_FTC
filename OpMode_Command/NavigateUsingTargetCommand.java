package com.hadronknights.ftc.opmode.command;

import com.hadronknights.ftc.robot.HkRobot;
import com.hadronknights.ftc.robot.drivetrain.Drivetrain;
import com.hadronknights.ftc.robot.subsystem.HkVision;
import com.hadronknights.ftc.robot.subsystem.HkVisionConstants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * This commands navigates the robot towards a specific target.
 */
public class NavigateUsingTargetCommand extends BaseCommand implements HkVisionConstants {
    private static final int MAX_RETRY_COUNT = 10; // 10 attempts
    private static final float HEADING_TOLERANCE = 1.0f;  // 1 degree
    private static final float POSITION_TOLERANCE = 0.25f; // 1 inch

    /**
     * Drivetrain
     */
    private Drivetrain drivetrain;

    /**
     * Handle of list of trackable in Skystone season
     */
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    /**
     * Name of the trackable, robot is interested in using for navigation
     */
    private String targetTrackableName;

    /**
     * Destination coordinate for the robot
     */
    private double robot_dest_x;
    private double robot_dest_y;


    /**
     * Set to true once the desired target is visible.
     */
    private boolean targetVisible = false;

    /**
     * Reference to the visible trackable target
     */
    private VuforiaTrackable target;

    /**
     * Location of the robot w.r.t field coordinates
     */
    private OpenGLMatrix lastLocation = null;


    /**
     * Constructor
     *
     * @param opMode
     * @param vuforia
     * @param robot
     * @param targetTrackableName
     * @param robot_dest_x
     * @param robot_dest_y
     */
    public NavigateUsingTargetCommand(LinearOpMode opMode, HkRobot robot, HkVision vuforia, String targetTrackableName, double robot_dest_x, double robot_dest_y) {
        super(opMode, robot);

        this.allTrackables = vuforia.getAllTargets();

        this.targetTrackableName = targetTrackableName;
        this.robot_dest_x = robot_dest_x;
        this.robot_dest_y = robot_dest_y;
    }

    @Override
    public void run() {
        if (!opMode.opModeIsActive()) return;

        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;

        int retryCount = 0;

        double power = 0.4;


        while (opMode.opModeIsActive() && !targetVisible && retryCount < MAX_RETRY_COUNT) {
            trackTarget();

            telemetry.addData("Target visible True or False:", targetVisible);
            telemetry.update();
            sleepWithActiveCheck(20);

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                telemetry.update();
                float robotHeading = rotation.thirdAngle;
                float robot_x = translation.get(0) / mmPerInch;
                float robot_y = translation.get(1) / mmPerInch;

                telemetry.addData("Robot location", "{Heading, X-coord, Y-coord} = %.0f, %.0f, %.0f", robotHeading, robot_x, robot_y);
                telemetry.update();

                Orientation targetRotation = Orientation.getOrientation(target.getLocation(), EXTRINSIC, XYZ, DEGREES);

                float targetHeading = targetRotation.thirdAngle;

                // align the robot heading to the target heading
                if (targetHeading - robotHeading > HEADING_TOLERANCE) {
                    drivetrain.turnLeft(Math.abs(targetHeading - robotHeading), 0.4);
                } else if (targetHeading - robotHeading < -HEADING_TOLERANCE) {
                    drivetrain.turnRight(Math.abs(targetHeading - robotHeading), 0.4);
                } else {
                    // robot heading and target heading very close, no need to turn
                }

                // move robot to destination x,y coordinate
                if (targetHeading > 0.0 - HEADING_TOLERANCE && targetHeading < 0.0 + HEADING_TOLERANCE) {
                    // strafe left/right for x, move forward/backward for y

                    if (robot_dest_x - robot_x > POSITION_TOLERANCE) {
                        drivetrain.strafeRight(power, Math.abs(robot_dest_x - robot_x), false, 0, true);
                    } else if (robot_dest_x - robot_x < -POSITION_TOLERANCE) {
                        drivetrain.strafeLeft(power, Math.abs(robot_dest_x - robot_x), false, 0, true);
                    } else {
                        // robot is already desired x coordinate
                    }

                    if (robot_dest_y - robot_y > POSITION_TOLERANCE) {
                        drivetrain.moveForward(power, Math.abs(robot_dest_y - robot_y), false, 0, true);
                    } else if (robot_dest_y - robot_y < -POSITION_TOLERANCE) {
                        drivetrain.moveBackward(power, Math.abs(robot_dest_y - robot_y), false, 0, true);
                    } else {
                        // robot is already desired y coordinate
                    }
                } else if (targetHeading > 180.0 - HEADING_TOLERANCE && targetHeading < 180 + HEADING_TOLERANCE) {
                    // strafe left/right for x, move forward/backward for y

                    if (robot_dest_x - robot_x > POSITION_TOLERANCE) {
                        drivetrain.strafeLeft(power, Math.abs(robot_dest_x - robot_x), false, 0, true);
                    } else if (robot_dest_x - robot_x < -POSITION_TOLERANCE) {
                        drivetrain.strafeRight(power, Math.abs(robot_dest_x - robot_x), false, 0, true);
                    } else {
                        // robot is already desired x coordinate
                    }

                    if (robot_dest_y - robot_y > POSITION_TOLERANCE) {
                        drivetrain.moveBackward(power, Math.abs(robot_dest_y - robot_y), false, 0, true);
                    } else if (robot_dest_y - robot_y < -POSITION_TOLERANCE) {
                        drivetrain.moveForward(power, Math.abs(robot_dest_y - robot_y), false, 0, true);
                    } else {
                        // robot is already desired y coordinate
                    }
                } else if (targetHeading > -90.0 - HEADING_TOLERANCE && targetHeading < -90 + HEADING_TOLERANCE) {
                    // move forward/backward for x, strafe left/right for y
                    if (robot_dest_x - robot_x > POSITION_TOLERANCE) {
                        drivetrain.moveForward(power, Math.abs(robot_dest_x - robot_x), false, 0, true);
                    } else if (robot_dest_x - robot_x < -POSITION_TOLERANCE) {
                        drivetrain.moveBackward(power, Math.abs(robot_dest_x - robot_x), false, 0, true);
                    } else {
                        // robot is already desired x coordinate
                    }

                    if (robot_dest_y - robot_y > POSITION_TOLERANCE) {
                        drivetrain.strafeLeft(power, Math.abs(robot_dest_y - robot_y), false, 0, true);
                    } else if (robot_dest_y - robot_y < -POSITION_TOLERANCE) {
                        drivetrain.strafeRight(power, Math.abs(robot_dest_y - robot_y), false, 0, true);
                    } else {
                        // robot is already desired y coordinate
                    }

                } else if (targetHeading > 90 - HEADING_TOLERANCE && targetHeading < 90 + HEADING_TOLERANCE) {
                    // move forward/backward for x, strafe left/right for y
                    if (robot_dest_x - robot_x > POSITION_TOLERANCE) {
                        drivetrain.moveBackward(Math.abs(robot_dest_x - robot_x), 0.4, false, 0, true);
                    } else if (robot_dest_x - robot_x < -POSITION_TOLERANCE) {
                        drivetrain.moveForward(Math.abs(robot_dest_x - robot_x), 0.4, false, 0, true);
                    } else {
                        // robot is already desired x coordinate
                    }

                    if (robot_dest_y - robot_y > POSITION_TOLERANCE) {
                        drivetrain.strafeRight(Math.abs(robot_dest_y - robot_y), 0.4, false, 0, true);
                    } else if (robot_dest_y - robot_y < -POSITION_TOLERANCE) {
                        drivetrain.strafeLeft(Math.abs(robot_dest_y - robot_y), 0.4, false, 0, true);
                    } else {
                        // robot is already desired y coordinate
                    }

                } else {
                    // not valid scenario
                }
            } else {
                drivetrain.setPower(new double[]{power, power, power, power});
                telemetry.addData("Target not visible", "... Retrying");
                telemetry.addData("Retry count", retryCount);
                telemetry.update();
            }

            retryCount++;
        }

    }

    /**
     * Helper method to track the desired trackable target.
     */
    private void trackTarget() {
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible target", trackable.getName());
                telemetry.update();
                sleepWithActiveCheck(20);
                if (trackable.getName().equals(targetTrackableName)) {
                    target = trackable;
                    telemetry.addData("Desired target visible", trackable.getName());
                    telemetry.update();

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        drivetrain.stop();
                        targetVisible = true;
                        telemetry.addData("Robot location is available", trackable.getName());
                        telemetry.update();
                        sleepWithActiveCheck(20);
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
        }
    }
}
