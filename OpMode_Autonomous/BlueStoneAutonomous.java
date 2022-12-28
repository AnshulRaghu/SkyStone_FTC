package com.hadronknights.ftc.opmode.autonomous;

import com.hadronknights.ftc.opmode.command.BlueCollectStoneCommand;
import com.hadronknights.ftc.opmode.command.MoveLiftUpCommand;
import com.hadronknights.ftc.opmode.command.MoveUntilObjectCommand;
import com.hadronknights.ftc.opmode.command.StrafeUntilLineCommand;
import com.hadronknights.ftc.opmode.command.StrafeUntilLineCommandV2;
import com.hadronknights.ftc.opmode.command.StrafeUntilObjectCommand;
import com.hadronknights.ftc.robot.drivetrain.Drivetrain;
import com.hadronknights.ftc.robot.HkColor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This autonomous class for is for blue collecting a stone.
 * <p>
 * Created by Anshul R on 02/04/20
 */

@Autonomous(name = "2 - Blue Two Stones Autonomous", group = "HK Auto")
    public class BlueStoneAutonomous extends BaseAutonomous {
    private Double desiredOrientationAngle;
    private LinearOpMode opMode;

    @Override
    public void runAutonomous() {

        sleepWithActiveCheck(5000);

        Drivetrain drivetrain = robot.getDrivetrain();

        Double desiredOrientationAngle = drivetrain.getRobotOrientation();

        robot.getClaw().releaseStone();
        sleepWithActiveCheck(50);

        double distanceFromWall;

        distanceFromWall = 9;

        StrafeUntilObjectCommand strafeUntilWallCommand = new StrafeUntilObjectCommand(opMode, robot, robot.getRightDistanceSensor(), distanceFromWall, 0.4,
                false, 0.0, 0, 0, 0, 0.2,
                true, desiredOrientationAngle, 0.04);
        strafeUntilWallCommand.run();
        sleepWithActiveCheck(50);

        MoveUntilObjectCommand moveUntilStoneCommand = new MoveUntilObjectCommand(opMode, robot, robot.getFrontRightDistanceSensor(), 35, 0.3, true, 2, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
        moveUntilStoneCommand.run();
        sleepWithActiveCheck(50);

        drivetrain.moveForward(0.2, 2, false, 0, true);
        sleepWithActiveCheck(100);

        BlueCollectStoneCommand collectStoneCommand = new BlueCollectStoneCommand(this, robot, desiredOrientationAngle);
        collectStoneCommand.run();
        sleepWithActiveCheck(100);

        MoveLiftUpCommand moveLiftCommand = new MoveLiftUpCommand(this, robot, 0.5, 2.25);
        moveLiftCommand.run();

        StrafeUntilLineCommandV2 strafeUntilLineCommand = new StrafeUntilLineCommandV2(this, robot, HkColor.BLUE, robot.getRightColorSensor(), -0.8, false, 0, 15,
                -0.2, 0, 0, -0.3, true, desiredOrientationAngle, 0.02);
        strafeUntilLineCommand.run();

        drivetrain.strafeLeft(0.8, 10, false, 0, true, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
        robot.getStoneGuard();

        StrafeUntilLineCommand strafeUntilLineCommand2 = new StrafeUntilLineCommand(this, robot, HkColor.BLUE, robot.getRightColorSensor(), 0.3);
        strafeUntilLineCommand2.run();
        sleepWithActiveCheck(50);
    }
}