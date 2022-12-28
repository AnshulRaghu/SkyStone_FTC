package com.hadronknights.ftc.opmode.command;

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
public class RedDetectFirstSkystoneUsingWebcamCommand extends BaseCommand implements HkVisionConstants {

    private TFObjectDetector tfod;

    private DistanceSensor distanceSensor;
    private Double desiredOrientationAngle;

    private List<Recognition> skystones = new ArrayList<Recognition>();

    private boolean skystoneFound = false;
    private static final int MAX_RETRY = 10;
    private Recognition targetSkystone = null;

    private RedDetectFirstSkystoneUsingWebcamCommand(LinearOpMode opMode, HkRobot robot, HkVision vuforia, Double desiredOrientationAngle) {
        super(opMode, robot);

        distanceSensor = robot.getLeftDistanceSensor();
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
                drivetrain.strafeRight(0.2, 4, false, 0, true);
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

        robot.getFoundationHauler().moveDown();
        // Only uses right grabber for BLUE alliance
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
            //correctPositionError();
        }
    }

    private void correctPositionError() {
        double travelDistance = 0;
        double currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);

        if (currentDistance >= 20.5 - 3.5 && currentDistance <= 20.5 + 3.5) {
            telemetry.addData("Stone number", 4);
            travelDistance = 20.5 - currentDistance;
        } else if (currentDistance >= 28.5 - 3.5 && currentDistance < 28.5 + 3.5) {
            telemetry.addData("Stone number", 5);
            travelDistance = 28.5 - currentDistance;
        } else if (currentDistance >= 36.5 - 3.5 && currentDistance < 36.5 + 3.5) {
            telemetry.addData("Stone number", 6);
            travelDistance = 36.5 - currentDistance;
        } else {
            telemetry.addData("Stone number", "Did n't fall in range");
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
}