package com.hadronknights.ftc.opmode.command;

import com.hadronknights.ftc.robot.HkRobot;
import com.hadronknights.ftc.robot.subsystem.ParkExtender;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ParkCommand extends BaseCommand {

    private ParkExtender parkExtender;


    public ParkCommand(LinearOpMode opMode, HkRobot robot) {
        super(opMode, robot);
        parkExtender = robot.getParkExtender();
    }

    @Override
    public void run() {
        if (!opMode.opModeIsActive()) return;
        parkExtender.extendTape(1, 40, false, 0);
    }
}
