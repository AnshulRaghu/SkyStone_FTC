package com.hadronknights.ftc.opmode.command;

import com.hadronknights.ftc.robot.HkPosition;
import com.hadronknights.ftc.robot.HkRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BlueCollectFirstSkystoneForThreeCommand extends BaseCommand {

    private ColorSensor rightColorSensor;
    private ColorSensor leftColorSensor;
    private Double desiredOrientationAngle;
    private HkPosition skystonePosition;

    public BlueCollectFirstSkystoneForThreeCommand(LinearOpMode opMode, HkRobot robot, Double desiredOrientationAngle) {
        super(opMode, robot);
        rightColorSensor = robot.getFrontRightColorSensor();
        leftColorSensor = robot.getFrontLeftColorSensor();
        this.desiredOrientationAngle = desiredOrientationAngle;
    }

    @Override
    public void run() {
        if (!opMode.opModeIsActive()) return;
        double leftBlueGreenRatio = getBlueToGreenColorValueRatio(leftColorSensor);
        double rightBlueGreenRatio = getBlueToGreenColorValueRatio(rightColorSensor);
        telemetry.addData("Left avg blue/green ratio value", "%.2f", leftBlueGreenRatio);
        telemetry.addData("Right avg blue/green ratio value", "%.2f", rightBlueGreenRatio);
        telemetry.update();

        robot.getClaw().grabStoneEvenMoreLoose();

        if (rightBlueGreenRatio > BLUE_TO_GREEN_RATIO_THRESHOLD) {
            drivetrain.strafeRight(0.4, 4.25, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
            skystonePosition = HkPosition.SIX;
        } else if (leftBlueGreenRatio > BLUE_TO_GREEN_RATIO_THRESHOLD) {
            drivetrain.strafeLeft(0.4, 4.5, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
            skystonePosition = HkPosition.FOUR;
        } else {
            drivetrain.strafeRight(0.4, 12, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
            skystonePosition = HkPosition.FIVE;
        }

        robot.getClaw().grabStone();
        sleepWithActiveCheck(50);

        drivetrain.moveBackward(0.3, 4, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
        checkForTwoStones();
    }

    private void checkForTwoStones() {
        if (robot.getFrontLeftDistanceSensor().getDistance(DistanceUnit.INCH) < 3 && robot.getFrontRightDistanceSensor().getDistance(DistanceUnit.INCH) < 3) {
            // two blocks picked up
            robot.getClaw().grabStoneLoose();
            sleepWithActiveCheck(300);

            double leftBlueGreenRatio = getBlueToGreenColorValueRatio(robot.getFrontLeftColorSensor());
            double rightBlueGreenRatio = getBlueToGreenColorValueRatio(robot.getFrontRightColorSensor());
            telemetry.addData("Left avg blue/green ratio value", "%.2f", leftBlueGreenRatio);
            telemetry.addData("Right avg blue/green ratio value", "%.2f", rightBlueGreenRatio);
            telemetry.update();

            if (leftBlueGreenRatio > BLUE_TO_GREEN_RATIO_THRESHOLD) {
                // left stone is skystone
                drivetrain.strafeLeft(0.4, 3, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
            } else if (rightBlueGreenRatio > BLUE_TO_GREEN_RATIO_THRESHOLD) {
                // right stone is skystone
                drivetrain.strafeRight(0.4, 3, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
            } else {
                // both are stone - yellow, choosing left one
                drivetrain.strafeLeft(0.4, 3, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
            }

            robot.getClaw().grabStone();

            sleepWithActiveCheck(50);
        }
    }

    public HkPosition getSkystonePosition() {
        return skystonePosition;
    }
}
