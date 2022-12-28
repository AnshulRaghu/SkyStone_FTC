package com.hadronknights.ftc.opmode.command;

import com.hadronknights.ftc.robot.HkPosition;
import com.hadronknights.ftc.robot.HkRobot;
import com.hadronknights.ftc.robot.subsystem.HkVision;
import com.hadronknights.ftc.robot.subsystem.HkVisionConstants;
import com.hadronknights.ftc.robot.subsystem.HkVisionSkystone;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

/**
 * This command detects the Skystone using tensor flow.
 */
public class BlueDetectFirstSkystoneUsingWebcamCommand extends BaseCommand implements HkVisionConstants {

    private TFObjectDetector tfod;
    private DistanceSensor distanceSensor;
    private Double desiredOrientationAngle;

    private List<Recognition> skystones = new ArrayList<Recognition>();

    private boolean skystoneFound = false;
    private static final int MAX_RETRY = 10;
    private Recognition targetSkystone = null;

    HkPosition skystonePosition;

    private BlueDetectFirstSkystoneUsingWebcamCommand(LinearOpMode opMode, HkRobot robot, HkVision vuforia, Double desiredOrientationAngle) {
        super(opMode, robot);

        distanceSensor = robot.getRightDistanceSensor();
        this.desiredOrientationAngle = desiredOrientationAngle;

        tfod = ((HkVisionSkystone) vuforia).getTfod();

        telemetry.addData("Tensor flow object", tfod);
        telemetry.update();
    }


    @Override
    public void run() {
        if (!opMode.opModeIsActive()) return;
        int retryCount = 0;
        while (opMode.opModeIsActive() && !skystoneFound && retryCount < MAX_RETRY && distanceSensor.getDistance(DistanceUnit.INCH) > 4.0) {
            detectSkystone();
            if (!skystones.isEmpty()) {
                targetSkystone = skystones.get(0);
                skystoneFound = true;
                moveToTagetSkystone();
            } else {
                drivetrain.strafeLeft(0.2, 4, false, 0, true);
                sleepWithActiveCheck(20);
                retryCount++;
            }
        }
    }

    public Recognition getTargetSkystone() {
        return targetSkystone;
    }

    public boolean isSkystoneFound() {
        return skystoneFound;
    }

    private void detectSkystone() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                        skystones.add(recognition);
                        break;
                    }
                }
                telemetry.addData("Skystone found", skystones.size());
                telemetry.update();
            }
        }
    }

    private void moveToTagetSkystone() {
        // Only uses right grabber for BLUE alliance
        robot.getFoundationHauler().moveDown();

        if (targetSkystone != null) {
            double estimateAngle = targetSkystone.estimateAngleToObject(AngleUnit.RADIANS);
            telemetry.addData("Target skystone angle", targetSkystone.estimateAngleToObject(AngleUnit.DEGREES));

            double x_mid = 12 * Math.tan(estimateAngle);
            double travelDistance = x_mid - 3.0;
            if (estimateAngle < 0) {
                travelDistance -= 2.25;
            }

            telemetry.addData("Travel distance", travelDistance);

            if (travelDistance > 0) {
                drivetrain.strafeRight(0.3, travelDistance, false, 0.0, true, 0.0, 0.0, 0.0, 0.0,
                        true, desiredOrientationAngle, 0.02);
            } else if (travelDistance < 0) {
                drivetrain.strafeLeft(0.3, Math.abs(travelDistance), false, 0, true);
            }
            telemetry.update();
            sleepWithActiveCheck(20);
//            correctPositionError();
        }
    }

    private void correctPositionError() {
        double travelDistance = 0;
        double currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);

        if (currentDistance >= 19 - 3.5 && currentDistance <= 19 + 3.5) {
            telemetry.addData("Stone number", 4);
            travelDistance = 19 - currentDistance;
        } else if (currentDistance >= 27 - 3.5 && currentDistance < 27 + 3) {
            telemetry.addData("Stone number", 5);
            travelDistance = 27 - currentDistance - 1;
        } else if (currentDistance >= 35 - 3 && currentDistance < 35 + 3.5) {
            telemetry.addData("Stone number", 6);
            travelDistance = 35 - currentDistance - 1.5;
        } else {
            telemetry.addData("Stone number", "Didn't fall in range");
        }

        telemetry.addData("Travel distance", travelDistance);
        telemetry.update();

        if (travelDistance > 0) {
            drivetrain.strafeRight(0.3, travelDistance, false, 0.0, true, 0.0, 0.0, 0.0, 0.0,
                    true, desiredOrientationAngle, 0.02);
        } else if (travelDistance < 0) {
            drivetrain.strafeLeft(0.3, Math.abs(travelDistance), false, 0, true);
        }
    }

    private void determineSkystonePosition() {
        double currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);
        if (currentDistance >= 19 - 3.5 && currentDistance <= 19 + 3.5) {
            telemetry.addData("Stone number", 4);
            skystonePosition = HkPosition.FOUR;
        } else if (currentDistance >= 27 - 3.5 && currentDistance < 27 + 3) {
            telemetry.addData("Stone number", 5);
            skystonePosition = HkPosition.FIVE;
        } else if (currentDistance >= 35 - 3 && currentDistance < 35 + 3.5) {
            telemetry.addData("Stone number", 6);
            skystonePosition = HkPosition.SIX;
        } else {
            telemetry.addData("Stone number", "Didn't fall in range");
            skystonePosition = HkPosition.FIVE;
        }
        telemetry.update();
    }

    public HkPosition getSkystonePosition() {
        return skystonePosition;
    }
}