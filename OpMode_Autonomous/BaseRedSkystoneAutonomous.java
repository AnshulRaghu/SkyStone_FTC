package com.hadronknights.ftc.opmode.autonomous;

import com.hadronknights.ftc.opmode.command.MoveLiftUpCommand;
import com.hadronknights.ftc.opmode.command.MoveUntilObjectCommand;
import com.hadronknights.ftc.opmode.command.RedCollectFirstSkystoneCommand;
import com.hadronknights.ftc.opmode.command.RedCollectSecondSkystoneCommand;
import com.hadronknights.ftc.opmode.command.StrafeUntilObjectCommand;

/**
 * This abstract class implements the common logic across all autonomous scenarios for Skystone season.
 * <p>
 * Created by Rahul V, Arko, Rahul D, & Aarush on 8/27/2017.
 */

public abstract class BaseRedSkystoneAutonomous extends BaseSkystoneAutonomous {

    protected Double desiredOrientationAngle;

    /**
     * Common post initialization logic to expand our robot
     */
    protected void postInitAutonomous() {
        desiredOrientationAngle = drivetrain.getRobotOrientation();
        robot.getStoneGuard().reset();
    }

    /**
     * Series of steps to collect first Skystone in RED side
     */
    protected void collectFirstSkystone() {

        // Bring the claw down to get ready to grab the stone
        robot.getClaw().getReadyToGrabStone();

        drivetrain.moveForward(0.5, 8, false, 0, false, 0.2, 8, 0, 0, true, desiredOrientationAngle, 0.02);

        // Move to quarry
        MoveUntilObjectCommand moveUntilObjectCommand = new MoveUntilObjectCommand(this, robot, robot.getFrontLeftDistanceSensor(), 5, 0.5, false,
                0, 10, 0.5, 25, 0.2, true, desiredOrientationAngle, 0.02);
        moveUntilObjectCommand.run();
        sleepWithActiveCheck(20);

        // Collect the first Skystone
        RedCollectFirstSkystoneCommand collectFirstSkystoneCommand = new RedCollectFirstSkystoneCommand(this, robot, desiredOrientationAngle);
        collectFirstSkystoneCommand.run();
        sleepWithActiveCheck(20);

        // Record the position of the first Skystone
        firstSkystonePosition = collectFirstSkystoneCommand.getSkystonePosition();

        telemetry.addData("Skystone position", firstSkystonePosition);
        telemetry.update();

        // Move out front the Stone Guard
        robot.getStoneGuard().moveStraightFront();

        // Retract from Quarry after collecting the first Skystone
        MoveUntilObjectCommand moveUntilWallCommand = new MoveUntilObjectCommand(this, robot, robot.getBackDistanceSensor(), 26.5, -0.3, false, 0, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
        moveUntilWallCommand.run();

        // Align stone
        robot.getClaw().grabStoneLoose();
        robot.getStoneGuard().centerStone();
        sleepWithActiveCheck(250);
        robot.getClaw().grabStone();
        sleepWithActiveCheck(50);
        robot.getStoneGuard().reset();

        MoveLiftUpCommand moveLiftCommand = new MoveLiftUpCommand(this, robot, 0.7, 0.5);
        moveLiftCommand.run();
        sleepWithActiveCheck(20);
    }

    /**
     * Series of steps to strafe from loading zone to building zone to align the robot for placing the stone on the foundation
     */
    protected void strafeToBuildingZoneForStonePlacement(double strafeDistance, double targetDistanceFromWall, double liftDistance) {
        // Move from the loading zone to the building zone
        drivetrain.strafeRight(0.9, strafeDistance, false, 0, false, 0.2, 15, 0, 0, true, desiredOrientationAngle, 0.02);

        MoveLiftUpCommand moveLiftCommand = new MoveLiftUpCommand(this, robot, 0.7, liftDistance);
        Thread moveLiftThread = new Thread(moveLiftCommand);
        moveLiftThread.start();

        StrafeUntilObjectCommand strafeUntilWallCommand = new StrafeUntilObjectCommand(this, robot, robot.getRightDistanceSensor(), targetDistanceFromWall, 0.9,
                false, 0.0, 0, 0, targetDistanceFromWall + 20, 0.2,
                true, desiredOrientationAngle, 0.04);
        strafeUntilWallCommand.run();
        sleepWithActiveCheck(50);
    }

    /**
     * Series of steps to collect second Skystone in RED side
     */
    protected void collectSecondSkystone(double distanceBuildingToLoadingZone) {
        // Collect the second Skystone
        RedCollectSecondSkystoneCommand collectSecondSkystoneCommand = new RedCollectSecondSkystoneCommand(this, robot, firstSkystonePosition, desiredOrientationAngle);
        collectSecondSkystoneCommand.run();

        robot.getStoneGuard().centerStone();

        // Retract from the Quarry after collecting the second Skystone
        MoveUntilObjectCommand retractFromQuarryCommand = new MoveUntilObjectCommand(this, robot, robot.getBackDistanceSensor(), 27, -0.6, false, 0, 8, -0.2, 21, -0.2, true, desiredOrientationAngle, 0.02);
        retractFromQuarryCommand.run();

        robot.getClaw().grabStoneLoose();
        sleepWithActiveCheck(250);
        robot.getClaw().grabStone();
        sleepWithActiveCheck(50);

        robot.getStoneGuard().reset();
    }
}