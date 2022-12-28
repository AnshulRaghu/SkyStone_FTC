package com.hadronknights.ftc.opmode.autonomous;

import com.hadronknights.ftc.opmode.command.BlueCollectFirstSkystoneForThreeCommand;
import com.hadronknights.ftc.opmode.command.BlueCollectSecondSkystoneForThreeCommand;
import com.hadronknights.ftc.opmode.command.BlueCollectThirdStoneForThreeCommand;
import com.hadronknights.ftc.opmode.command.MoveDiagonallyUntilLineCommand;
import com.hadronknights.ftc.opmode.command.MoveDiagonallyUntilObjectCommand;
import com.hadronknights.ftc.opmode.command.MoveLiftUpCommand;
import com.hadronknights.ftc.opmode.command.MoveUntilObjectCommand;
import com.hadronknights.ftc.opmode.command.StrafeUntilObjectCommand;
import com.hadronknights.ftc.robot.HkColor;

/**
 * This abstract class implements the common logic across all autonomous scenarios for Skystone season.
 * <p>
 * Created by Rahul V, Arko, Rahul D, & Aarush on 8/27/2017.
 */

public abstract class BaseBlueSkystoneAutonomousThree extends BaseSkystoneAutonomous {

    protected Double desiredOrientationAngle;

    /**
     * Common post initialization logic to expand our robot
     */
    protected void postInitAutonomous() {
        desiredOrientationAngle = drivetrain.getRobotOrientation();
        robot.getStoneGuard().reset();
    }

    /**
     * Series of steps to collect first Skystone in BLUE side
     */
    protected void collectFirstSkystone() {

        // Bring the claw down to get ready to grab the stone
        robot.getClaw().getReadyToGrabStone();

        drivetrain.strafeRight(0.5, 7, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.04);

        // Move to quarry
        MoveUntilObjectCommand moveUntilObjectCommand = new MoveUntilObjectCommand(this, robot, robot.getFrontRightDistanceSensor(), 6, 0.6, false,
                0, 15, 0.2, 24, 0.2, true, desiredOrientationAngle, 0.02);
        moveUntilObjectCommand.run();
        sleepWithActiveCheck(20);

        // Collect the first Skystone
        BlueCollectFirstSkystoneForThreeCommand collectFirstSkystoneCommand = new BlueCollectFirstSkystoneForThreeCommand(this, robot, desiredOrientationAngle);
        collectFirstSkystoneCommand.run();
        sleepWithActiveCheck(20);

        // Record the position of the first Skystone
        firstSkystonePosition = collectFirstSkystoneCommand.getSkystonePosition();

        telemetry.addData("Skystone position", firstSkystonePosition);
        telemetry.update();

        MoveLiftUpCommand moveLiftCommand = new MoveLiftUpCommand(this, robot, 0.7, 0.5);
        moveLiftCommand.run();
        sleepWithActiveCheck(20);
    }

    protected void moveDiagonallyToBuildingZone(double power, double angleToLine, double angleToWall, double targetDistance, double liftDistance) {
        // Move from the loading zone to the building zone
        MoveDiagonallyUntilLineCommand moveDiagonallyUntilLineCommand = new MoveDiagonallyUntilLineCommand(this, robot, HkColor.BLUE, robot.getRightColorSensor(), power, angleToLine, false, 0);
        moveDiagonallyUntilLineCommand.run();

        MoveLiftUpCommand moveLiftCommand = new MoveLiftUpCommand(this, robot, 0.7, liftDistance);
        Thread moveLiftThread = new Thread(moveLiftCommand);
        moveLiftThread.start();

        MoveDiagonallyUntilObjectCommand moveDiagonallyUntilWallCommand = new MoveDiagonallyUntilObjectCommand(this, robot, robot.getLeftDistanceSensor(), targetDistance, power, angleToWall, false, 0);
        moveDiagonallyUntilWallCommand.run();
        sleepWithActiveCheck(20);
    }

    /**
     * Series of steps to collect second Skystone in BLUE side
     */
    protected void collectSecondSkystone() {
        MoveDiagonallyUntilLineCommand moveUntilLineGoToStoneTwo = new MoveDiagonallyUntilLineCommand(this, robot, HkColor.BLUE, robot.getRightColorSensor(), 1, 345);
        moveUntilLineGoToStoneTwo.run();

        // Collect the second Skystone
        BlueCollectSecondSkystoneForThreeCommand collectSecondSkystoneCommand = new BlueCollectSecondSkystoneForThreeCommand(this, robot, firstSkystonePosition, desiredOrientationAngle);
        collectSecondSkystoneCommand.run();
        sleepWithActiveCheck(20);

        robot.getStoneGuard().reset();
    }

    protected void collectThirdStone(){
        MoveDiagonallyUntilLineCommand moveUntilLineGoToStoneTwo = new MoveDiagonallyUntilLineCommand(this, robot, HkColor.BLUE, robot.getRightColorSensor(), 1, 345);
        moveUntilLineGoToStoneTwo.run();

        BlueCollectThirdStoneForThreeCommand collectThirdStoneCommand = new BlueCollectThirdStoneForThreeCommand(this, robot, firstSkystonePosition, desiredOrientationAngle);
        collectThirdStoneCommand.run();
        sleepWithActiveCheck(20);

        robot.getStoneGuard().reset();
    }
}