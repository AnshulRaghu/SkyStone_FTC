package com.hadronknights.ftc.opmode.command;

import com.hadronknights.ftc.robot.HkRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BlueCollectStoneCommand extends BaseCommand {
    private Double desiredOrientationAngle;
    private ColorSensor rightColorSensor;
    private ColorSensor leftColorSensor;

    public BlueCollectStoneCommand(LinearOpMode opMode, HkRobot robot, Double desiredOrientationAngle) {
        super(opMode, robot);
        this.desiredOrientationAngle = desiredOrientationAngle;
        this.rightColorSensor = robot.getRightColorSensor();
        this.leftColorSensor = robot.getFrontLeftColorSensor();
    }

    @Override
    public void run() {
        if (!opMode.opModeIsActive()) return;
        double leftBlueGreenRatio = getBlueToGreenColorValueRatio(leftColorSensor);
        double rightBlueGreenRatio = getBlueToGreenColorValueRatio(rightColorSensor);
        telemetry.addData("Left avg blue/green ratio value", "%.2f", leftBlueGreenRatio);
        telemetry.addData("Right avg blue/green ratio value", "%.2f", rightBlueGreenRatio);
        telemetry.update();


        if (leftBlueGreenRatio > BLUE_TO_GREEN_RATIO_THRESHOLD) {
            // left is Skystone, go to right for regular stone
            drivetrain.strafeRight(0.3, 3, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);

        } else if (rightBlueGreenRatio > BLUE_TO_GREEN_RATIO_THRESHOLD) {
            // right is Skystone, go to left for regular stone
            drivetrain.strafeLeft(0.3, 5, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
        } else {
            // both stones are regular, go for the left
            drivetrain.strafeLeft(0.3, 5, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
        }
        sleepWithActiveCheck(100);


        drivetrain.moveForward(0.2, 2, false, 0, true);
        sleepWithActiveCheck(100);

        robot.getClaw().grabStone();
        sleepWithActiveCheck(200);

        drivetrain.moveBackward(0.3, 5, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
        sleepWithActiveCheck(20);

        checkForTwoStones();
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
