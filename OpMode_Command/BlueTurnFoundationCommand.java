package com.hadronknights.ftc.opmode.command;

import com.hadronknights.ftc.robot.HkRobot;
import com.hadronknights.ftc.robot.subsystem.Claw;
import com.hadronknights.ftc.robot.subsystem.FoundationHauler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class BlueTurnFoundationCommand extends BaseCommand {
    private FoundationHauler foundationHaluer;
    private Claw claw;
    private Double desiredOrientationAngle;

    public BlueTurnFoundationCommand(LinearOpMode opMode, HkRobot robot, Double desiredOrientationAngle) {
        super(opMode, robot);
        foundationHaluer = robot.getFoundationHauler();
        claw = robot.getClaw();
        this.desiredOrientationAngle = desiredOrientationAngle;
    }

    @Override
    public void run() {
        if (!opMode.opModeIsActive()) return;
        foundationHaluer.grabFoundation();
        robot.getClaw().reset();
        sleepWithActiveCheck(650);

        MoveUntilObjectCommand moveUntilWallCommand = new MoveUntilObjectCommand(opMode, robot, robot.getBackDistanceSensor(), 24, -0.7, false, 0.0, 10,
                -0.15, 39, -0.2, false, desiredOrientationAngle, 0.04);
        moveUntilWallCommand.run();
        sleepWithActiveCheck(100);

        drivetrain.rotateCounterClockwise(0.6, 0.2, 90);
        claw.releaseStone();
        sleepWithActiveCheck(200);

        ParkCommand parkCommand = new ParkCommand(opMode, robot);
        Thread parkThread = new Thread(parkCommand);
        parkThread.start();

        foundationHaluer.releaseFoundation();
        sleepWithActiveCheck(200);

        drivetrain.strafeRight(0.3, 8, false, 0, true);
        sleepWithActiveCheck(20);

        drivetrain.moveForward(0.5, 10, true, 0.5, true);
        sleepWithActiveCheck(20);

        drivetrain.moveBackward(0.5, 16, false, 0, false, 0, 0, 0, 0, true, desiredOrientationAngle, 0.02);
    }
}