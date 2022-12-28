package com.hadronknights.ftc.opmode.command;

import com.hadronknights.ftc.robot.HkRobot;
import com.hadronknights.ftc.robot.subsystem.FoundationHauler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RedPositionFoundationCommand extends BaseCommand {
    private FoundationHauler puller;
    private Double desiredOrientationAngle;

    public RedPositionFoundationCommand(LinearOpMode opMode, HkRobot robot, Double desiredOrientationAngle) {
        super(opMode, robot);
        puller = robot.getFoundationHauler();
        this.desiredOrientationAngle = desiredOrientationAngle;
    }

    @Override
    public void run() {
        if (!opMode.opModeIsActive()) return;
        puller.grabFoundation();
        robot.getClaw().reset();
        sleepWithActiveCheck(650);

        MoveUntilObjectCommand moveUntilWallCommand = new MoveUntilObjectCommand(opMode, robot, robot.getBackDistanceSensor(), 4, -0.5, false, 0.0, 10, -0.15, 10, -0.2, false, null, 0);
        moveUntilWallCommand.run();
        sleepWithActiveCheck(100);

        puller.releaseFoundation();
        sleepWithActiveCheck(200);
    }
}