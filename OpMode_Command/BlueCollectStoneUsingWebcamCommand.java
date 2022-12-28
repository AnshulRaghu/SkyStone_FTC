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

public class BlueCollectStoneUsingWebcamCommand extends BaseCommand implements HkVisionConstants {

    private TFObjectDetector tfod;
    private DistanceSensor distanceSensor;
    private Double desiredOrientationAngle;
    boolean stoneFound = false;
    private List<Recognition> stones = new ArrayList<Recognition>();

    private static final int MAX_RETRY = 10;
    private Recognition targetStone = null;

    public BlueCollectStoneUsingWebcamCommand(LinearOpMode opMode, HkRobot robot, HkVision vision, Double desiredOrientationAngle) {
        super(opMode, robot);
        distanceSensor = robot.getRightDistanceSensor();
        this.desiredOrientationAngle = desiredOrientationAngle;

        tfod = ((HkVisionSkystone) vision).getTfod();

        telemetry.addData("Tensor flow object", tfod);
        telemetry.update();
    }

    @Override
    public void run() {
        if (!opMode.opModeIsActive()) return;
        int retryCount = 0;
        while (opMode.opModeIsActive() && !stoneFound && retryCount < MAX_RETRY && distanceSensor.getDistance(DistanceUnit.INCH) > 4.0) {
            detectSkystone();
            if (!stones.isEmpty()) {
                targetStone = stones.get(0);
                stoneFound = true;
                moveToTagetSkystone();
            } else {
                drivetrain.strafeLeft(0.2, 4, false, 0, true);
                sleepWithActiveCheck(20);
                retryCount++;
            }
        }

        drivetrain.moveForward(0.2, 2, false, 0, true);
        sleepWithActiveCheck(100);

        robot.getClaw().grabStone();
        sleepWithActiveCheck(200);

        drivetrain.moveBackward(0.3, 5, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
        sleepWithActiveCheck(20);

        checkForTwoStones();
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
                    if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                        stones.add(recognition);
                        break;
                    }
                }
                telemetry.addData("Stone found", stones.size());
                telemetry.update();
            }
        }
    }

    private void moveToTagetSkystone() {
        // Only uses right grabber for BLUE alliance
        robot.getFoundationHauler().moveDown();

        if (targetStone != null) {
            double estimateAngle = targetStone.estimateAngleToObject(AngleUnit.RADIANS);
            telemetry.addData("Target stone angle", targetStone.estimateAngleToObject(AngleUnit.DEGREES));

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
        }
    }

    private void checkForTwoStones() {
        if (robot.getFrontLeftDistanceSensor().getDistance(DistanceUnit.INCH) < 3 && robot.getFrontRightDistanceSensor().getDistance(DistanceUnit.INCH) < 3) {
            // two blocks picked up
            robot.getClaw().releaseStone();
            sleepWithActiveCheck(300);

            double leftBlueGreenRatio = getBlueToGreenColorValueRatio(robot.getFrontLeftColorSensor());
            double rightBlueGreenRatio = getBlueToGreenColorValueRatio(robot.getFrontRightColorSensor());
            telemetry.addData("Left avg blue/green ratio value", "%.2f", leftBlueGreenRatio);
            telemetry.addData("Right avg blue/green ratio value", "%.2f", rightBlueGreenRatio);
            telemetry.update();

            if (leftBlueGreenRatio < BLUE_TO_GREEN_RATIO_THRESHOLD) {
                // left stone is stone
                drivetrain.strafeLeft(0.35, 3, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
            } else if (rightBlueGreenRatio < BLUE_TO_GREEN_RATIO_THRESHOLD) {
                // right stone is stone
                drivetrain.strafeRight(0.35, 5, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
            } else {
                // both are stone - yellow, choosing left one
                drivetrain.strafeLeft(0.35, 3, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
            }

            robot.getClaw().grabStone();
        }
    }
}
