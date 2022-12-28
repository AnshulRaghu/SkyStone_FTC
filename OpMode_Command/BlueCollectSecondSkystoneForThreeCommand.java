package com.hadronknights.ftc.opmode.command;

import com.hadronknights.ftc.robot.HkPosition;
import com.hadronknights.ftc.robot.HkRobot;
import com.hadronknights.ftc.robot.subsystem.Claw;
import com.hadronknights.ftc.robot.subsystem.FoundationHauler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BlueCollectSecondSkystoneForThreeCommand extends BaseCommand {
    private Claw claw;
    private FoundationHauler foundationHauler;
    private HkPosition skystonePosition;
    private Double desiredOrientationAngle;

    public BlueCollectSecondSkystoneForThreeCommand(LinearOpMode opMode, HkRobot robot, HkPosition skystonePosition, Double desiredOrientationAngle) {
        super(opMode, robot);
        this.claw = robot.getClaw();
        this.foundationHauler = robot.getFoundationHauler();
        this.desiredOrientationAngle = desiredOrientationAngle;
        this.skystonePosition = skystonePosition;
    }

    @Override
    public void run() {
        if (!opMode.opModeIsActive()) return;
        double distanceFromWall;
        MoveDiagonallyUntilObjectCommand moveDiagonallyUntilObjectCommand;

        switch (skystonePosition) {
            case SIX:
            default:
                distanceFromWall = 38;

                moveDiagonallyUntilObjectCommand = new MoveDiagonallyUntilObjectCommand(opMode, robot, robot.getRightDistanceSensor(), distanceFromWall, 1, 10);
                moveDiagonallyUntilObjectCommand.run();

                claw.grabStone();
                sleepWithActiveCheck(200);

            case FIVE:
                distanceFromWall = 30;

                moveDiagonallyUntilObjectCommand = new MoveDiagonallyUntilObjectCommand(opMode, robot, robot.getRightDistanceSensor(), distanceFromWall, 1, 10);
                moveDiagonallyUntilObjectCommand.run();

                claw.grabStone();
                sleepWithActiveCheck(200);

                drivetrain.moveBackward(0.3, 4, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
                break;

            case FOUR:
                distanceFromWall = 13;
                foundationHauler.moveDownOutward();

                moveDiagonallyUntilObjectCommand = new MoveDiagonallyUntilObjectCommand(opMode, robot, robot.getRightDistanceSensor(), distanceFromWall, 1, 10);
                moveDiagonallyUntilObjectCommand.run();

                drivetrain.moveForward(0.3, 9, false, 0, true);
                sleepWithActiveCheck(10);

                foundationHauler.moveUp();

                drivetrain.moveBackward(0.3, 12, false, 0, true);
                sleepWithActiveCheck(100);

                drivetrain.strafeRight(0.3, distanceFromWall + 3, true, 1.5, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
                sleepWithActiveCheck(10);

                drivetrain.moveForward(0.3, 2, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
                sleepWithActiveCheck(50);

                claw.grabStone();
                sleepWithActiveCheck(200);

        }

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
}
