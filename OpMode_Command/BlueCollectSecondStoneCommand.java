package com.hadronknights.ftc.opmode.command;

import com.hadronknights.ftc.robot.HkPosition;
import com.hadronknights.ftc.robot.HkRobot;
import com.hadronknights.ftc.robot.subsystem.Claw;
import com.hadronknights.ftc.robot.subsystem.FoundationHauler;
import com.hadronknights.ftc.robot.subsystem.StoneGuard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BlueCollectSecondStoneCommand extends BaseCommand {
    private Claw claw;
    private StoneGuard stoneGuard;
    private FoundationHauler foundationHauler;
    private HkPosition skystonePosition;
    private Double desiredOrientationAngle;

    public BlueCollectSecondStoneCommand(LinearOpMode opMode, HkRobot robot, HkPosition skystonePosition, Double desiredOrientationAngle) {
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
        MoveAwayFromObjectCommand moveAwayFromWallCommand;

        claw.reset();

        switch (skystonePosition) {
            case SIX:
            default:
                distanceFromWall = 30;

                // Move from the building zone to the loading zone
                drivetrain.strafeRight(0.9, 50, false, 0, false, 0.2, 10, 0, 0, true, desiredOrientationAngle, 0.04);

                strafeUntilWallCommand = new StrafeUntilObjectCommand(opMode, robot, robot.getRightDistanceSensor(), distanceFromWall, 0.9,
                        false, 0.0, 0, 0, distanceFromWall + 20, 0.2,
                        true, desiredOrientationAngle, 0.04);
                strafeUntilWallCommand.run();
                sleepWithActiveCheck(50);

                moveAwayFromWallCommand = new MoveAwayFromObjectCommand(opMode, robot, robot.getBackDistanceSensor(), 28, 0.7, false, 0, 10, 0.2, 18, 0.2, true, desiredOrientationAngle, 0.02);
                moveAwayFromWallCommand.run();
                claw.getReadyToGrabStone();
                sleepWithActiveCheck(50);

                claw.grabStone();
                sleepWithActiveCheck(200);

                drivetrain.moveBackward(0.3, 2, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
                break;

            case FIVE:

            case FOUR:
                distanceFromWall = 38;

                // Move from the building zone to the loading zone
                drivetrain.strafeRight(0.9, 50, false, 0, false, 0.2, 10, 0, 0, true, desiredOrientationAngle, 0.04);

                strafeUntilWallCommand = new StrafeUntilObjectCommand(opMode, robot, robot.getRightDistanceSensor(), distanceFromWall, 0.9,
                        false, 0.0, 0, 0, distanceFromWall + 20, 0.2,
                        true, desiredOrientationAngle, 0.04);
                strafeUntilWallCommand.run();
                sleepWithActiveCheck(50);

                moveAwayFromWallCommand = new MoveAwayFromObjectCommand(opMode, robot, robot.getBackDistanceSensor(), 28, 0.7, false, 0, 10, 0.2, 18, 0.2, true, desiredOrientationAngle, 0.02);
                moveAwayFromWallCommand.run();
                claw.getReadyToGrabStone();
                sleepWithActiveCheck(50);

                claw.grabStone();
                sleepWithActiveCheck(200);

                drivetrain.moveBackward(0.3, 2, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
                break;

        }

        checkForTwoStones();
    }

    private void checkForTwoStones() {
        if (robot.getFrontLeftDistanceSensor().getDistance(DistanceUnit.INCH) < 3 && robot.getFrontRightDistanceSensor().getDistance(DistanceUnit.INCH) < 3) {
            // two blocks picked up
            robot.getClaw().releaseStone();
            sleepWithActiveCheck(50);

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
                // both are stone - yellow, choosing left one
                drivetrain.strafeLeft(0.35, 3, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
            }

            robot.getClaw().grabStone();

            sleepWithActiveCheck(200);
        }
    }
}
