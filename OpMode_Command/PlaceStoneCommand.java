package com.hadronknights.ftc.opmode.command;

import com.hadronknights.ftc.robot.HkRobot;
import com.hadronknights.ftc.robot.subsystem.Claw;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class PlaceStoneCommand extends BaseCommand {

    private Claw claw;


    public PlaceStoneCommand(LinearOpMode opMode, HkRobot robot) {
        super(opMode, robot);
        claw = robot.getClaw();
    }

    @Override
    public void run() {
        if (!opMode.opModeIsActive()) return;
        claw.releaseStone();
        sleepWithActiveCheck(600);
    }
}
