package com.hadronknights.ftc.opmode.command;

import com.hadronknights.ftc.robot.HkPosition;
import com.hadronknights.ftc.robot.HkRobot;
import com.hadronknights.ftc.robot.subsystem.Claw;
import com.hadronknights.ftc.robot.subsystem.FoundationHauler;
import com.hadronknights.ftc.robot.subsystem.StoneGuard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RedCollectSecondSkystoneCommand extends BaseCommand {
    private Claw claw;
    private StoneGuard stoneGuard;
    private FoundationHauler foundationHauler;
    private HkPosition skystonePosition;
    private Double desiredOrientationAngle;

    public RedCollectSecondSkystoneCommand(LinearOpMode opMode, HkRobot robot, HkPosition skystonePosition, Double desiredOrientationAngle) {
        super(opMode, robot);
        this.claw = robot.getClaw();
        this.stoneGuard = robot.getStoneGuard();
        this.foundationHauler = robot.getFoundationHauler();
        this.desiredOrientationAngle = desiredOrientationAngle;
        this.skystonePosition = skystonePosition;
    }

    @Override
    public void run() {
        if (!opMode.opModeIsActive()) return;
        double distanceFromWall;
        StrafeUntilObjectCommand strafeUntilWallCommand;
        MoveUntilObjectCommand moveUntilStoneCommand;

        switch (skystonePosition) {
            case SIX:
            default:
                distanceFromWall = 12;
                stoneGuard.moveOutFront();

                // Move from the building zone to the loading zone
                drivetrain.strafeLeft(0.9, 50, false, 0, false, 0.2, 10, 0, 0, true, desiredOrientationAngle, 0.04);

                strafeUntilWallCommand = new StrafeUntilObjectCommand(opMode, robot, robot.getLeftDistanceSensor(), distanceFromWall, -0.9,
                        false, 0.0, 0, 0, distanceFromWall + 20, -0.2,
                        true, desiredOrientationAngle, 0.04);
                strafeUntilWallCommand.run();
                sleepWithActiveCheck(50);

                moveUntilStoneCommand = new MoveUntilObjectCommand(opMode, robot, robot.getFrontLeftDistanceSensor(), 6, 0.3, true, 6, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
                moveUntilStoneCommand.run();
                sleepWithActiveCheck(50);

                adjustRobotPositionUsingDistanceSensor(11.5);
                sleepWithActiveCheck(10);

                drivetrain.moveForward(0.2, 6, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
                sleepWithActiveCheck(50);

                claw.grabStone();
                sleepWithActiveCheck(250);

                drivetrain.moveBackward(0.3, 8, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
                break;

            case FIVE:
                distanceFromWall = 4;
                stoneGuard.moveOutFront();

                // Move from the building zone to the loading zone
                drivetrain.strafeLeft(0.9, 50, false, 0, false, 0.2, 10, 0, 0, true, desiredOrientationAngle, 0.04);

                strafeUntilWallCommand = new StrafeUntilObjectCommand(opMode, robot, robot.getLeftDistanceSensor(), distanceFromWall, -0.9,
                        false, 0.0, 0, 0, distanceFromWall + 20, -0.2,
                        true, desiredOrientationAngle, 0.04);
                strafeUntilWallCommand.run();
                sleepWithActiveCheck(50);

                moveUntilStoneCommand = new MoveUntilObjectCommand(opMode, robot, robot.getFrontLeftDistanceSensor(), 6, 0.3, true, 6, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
                moveUntilStoneCommand.run();
                sleepWithActiveCheck(50);

                adjustRobotPositionUsingDistanceSensor(3.5);
                sleepWithActiveCheck(10);

                drivetrain.moveForward(0.3, 6, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
                sleepWithActiveCheck(50);

                claw.grabStone();
                sleepWithActiveCheck(250);

                drivetrain.moveBackward(0.3, 8, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
                break;

            case FOUR:
                distanceFromWall = 11;
                foundationHauler.moveDownOutward();

                // Move from the building zone to the loading zone
                drivetrain.strafeLeft(0.9, 50, false, 0, false, 0.2, 10, 0, 0, true, desiredOrientationAngle, 0.04);

                strafeUntilWallCommand = new StrafeUntilObjectCommand(opMode, robot, robot.getLeftDistanceSensor(), distanceFromWall, -0.9,
                        false, 0.0, 0, 0, distanceFromWall + 20, -0.2,
                        true, desiredOrientationAngle, 0.04);
                strafeUntilWallCommand.run();
                sleepWithActiveCheck(50);

                moveUntilStoneCommand = new MoveUntilObjectCommand(opMode, robot, robot.getFrontLeftDistanceSensor(), 6, 0.3, true, 6, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
                moveUntilStoneCommand.run();
                sleepWithActiveCheck(50);

                adjustRobotPositionUsingDistanceSensor(distanceFromWall);

                foundationHauler.moveDown();
                sleepWithActiveCheck(50);

                drivetrain.moveForward(0.4, 11, false, 0, true);
                sleepWithActiveCheck(10);

                drivetrain.moveBackward(0.3, 10.5, false, 0, true);
                foundationHauler.moveUp();
                sleepWithActiveCheck(100);

                drivetrain.strafeLeft(0.5, distanceFromWall + 3, true, 1.5, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
                sleepWithActiveCheck(10);

                drivetrain.moveForward(0.3, 3, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
                sleepWithActiveCheck(50);

                claw.grabStone();
                sleepWithActiveCheck(250);

                drivetrain.strafeRight(0.4, 8, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
//                break;
        }

        checkForTwoStones();
    }

    private void adjustRobotPositionUsingDistanceSensor(double targetDistance) {
        double currentDistance = robot.getLeftDistanceSensor().getDistance(DistanceUnit.INCH);
        double distanceCorrection = targetDistance - currentDistance;

        if (distanceCorrection > 0) {
            drivetrain.strafeRight(0.2, distanceCorrection, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
            sleepWithActiveCheck(50);
        }
    }

    private void checkForTwoStones() {
        if (robot.getFrontLeftDistanceSensor().getDistance(DistanceUnit.INCH) < 3 && robot.getFrontRightDistanceSensor().getDistance(DistanceUnit.INCH) < 3) {
            // two blocks picked up
            robot.getClaw().grabStoneEvenMoreLoose();
            sleepWithActiveCheck(300);

            double leftBlueGreenRatio = getBlueToGreenColorValueRatio(robot.getFrontLeftColorSensor());
            double rightBlueGreenRatio = getBlueToGreenColorValueRatio(robot.getFrontRightColorSensor());
            telemetry.addData("Left avg blue/green ratio value", "%.2f", leftBlueGreenRatio);
            telemetry.addData("Right avg blue/green ratio value", "%.2f", rightBlueGreenRatio);
            telemetry.update();

            if (leftBlueGreenRatio > BLUE_TO_GREEN_RATIO_THRESHOLD) {
                // left stone is skystone
                drivetrain.strafeLeft(0.35, 3, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
            } else if (rightBlueGreenRatio > BLUE_TO_GREEN_RATIO_THRESHOLD) {
                // right stone is skystone
                drivetrain.strafeRight(0.35, 3, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
            } else {
                // both are stone - yellow, choosing right one
                drivetrain.strafeRight(0.35, 3, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
            }

            robot.getClaw().grabStone();

            sleepWithActiveCheck(50);
        }
    }
}
