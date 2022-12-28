package com.hadronknights.ftc.opmode.autonomous;

import com.hadronknights.ftc.opmode.command.BlueCollectStoneCommand;
import com.hadronknights.ftc.opmode.command.MoveUntilObjectCommand;
import com.hadronknights.ftc.opmode.command.StrafeUntilLineCommandV2;
import com.hadronknights.ftc.opmode.command.StrafeUntilObjectCommand;
import com.hadronknights.ftc.robot.HkColor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This autonomous class for is for blue collecting a stone.
 * <p>
 * Created by Anshul R on 02/04/20
 */
@Disabled
@Autonomous(name = "11 - Blue One Stone Autonomous", group = "HK Auto")
public class BlueOneStoneAutonomous extends BaseBlueSkystoneAutonomous {
    @Override
    public void runAutonomous() {

        // Post initialization
        postInitAutonomous();

        ElapsedTime runtime = new ElapsedTime();

        while (opModeIsActive() && (runtime.seconds() < 6.0)) {
            telemetry.addData("Wait", "%2.5f seconds elapsed", runtime.seconds());
            telemetry.update();
        }

        StrafeUntilObjectCommand strafeUntilWallCommand = new StrafeUntilObjectCommand(this, robot, robot.getRightDistanceSensor(), 9, 0.5,
                false, 0.0, 10, 0.2, 24, 0.2,
                true, desiredOrientationAngle, 0.04);
        strafeUntilWallCommand.run();
        sleepWithActiveCheck(50);

        // Bring the claw down to get ready to grab the stone
        robot.getClaw().getReadyToGrabStone();

        // Move to quarry
        MoveUntilObjectCommand moveUntilObjectCommand = new MoveUntilObjectCommand(this, robot, robot.getFrontRightDistanceSensor(), 3, 0.4, true,
                8, 10, 0.2, 25, 0.15, true, desiredOrientationAngle, 0.02);
        moveUntilObjectCommand.run();
        sleepWithActiveCheck(20);

        // Collect a stone
        BlueCollectStoneCommand collectStoneCommand = new BlueCollectStoneCommand(this, robot, desiredOrientationAngle);
        collectStoneCommand.run();
        sleepWithActiveCheck(100);

        // Move out front the Stone Guard
        robot.getStoneGuard().moveOutFront();

        // Retract from Quarry after collecting the stone
        MoveUntilObjectCommand moveUntilWallCommand = new MoveUntilObjectCommand(this, robot, robot.getBackDistanceSensor(), 3, -0.3, false, 0, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
        moveUntilWallCommand.run();
        sleepWithActiveCheck(20);

        // Align stone
        robot.getClaw().grabStoneLoose();
        robot.getStoneGuard().centerStone();
        sleepWithActiveCheck(200);
        robot.getClaw().grabStone();
        sleepWithActiveCheck(50);
        robot.getStoneGuard().reset();


        while (opModeIsActive() && (runtime.seconds() < 24.0)) {
            telemetry.addData("Wait", "%2.5f second elapsed", runtime.seconds());
            telemetry.update();
        }

        StrafeUntilLineCommandV2 strafeUntilLineFromLoadingZoneCommand = new StrafeUntilLineCommandV2(this, robot, HkColor.BLUE, robot.getRightColorSensor(), -0.8, false, 0, 15,
                -0.2, 32, 40, -0.3, true, desiredOrientationAngle, 0.02);
        strafeUntilLineFromLoadingZoneCommand.run();

        drivetrain.strafeLeft(0.8, 10, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
        robot.getStoneGuard().extendForParking();
        robot.getClaw().reset();
        sleepWithActiveCheck(100);

        // Park
        StrafeUntilLineCommandV2 strafeUntilLineFromBuildingZoneCommand = new StrafeUntilLineCommandV2(this, robot, HkColor.BLUE, robot.getRightColorSensor(), 0.3, false, 0, 0, 0, 0, 0, 0, true, desiredOrientationAngle, 0.2);
        strafeUntilLineFromBuildingZoneCommand.run();
        sleepWithActiveCheck(50);
    }
}