package com.hadronknights.ftc.opmode.autonomous;

import com.hadronknights.ftc.opmode.teleop.AutoTransitioner;
import com.hadronknights.ftc.robot.HkColor;
import com.hadronknights.ftc.robot.HkRobot;
import com.hadronknights.ftc.robot.drivetrain.Drivetrain;
import com.hadronknights.ftc.robot.subsystem.HkVision;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This abstract class implements the common logic across all autonomous scenarios.
 * <p>
 * Created by Rahul V, Arko, Rahul D, & Aarush on 8/27/2017.
 */

public abstract class BaseAutonomous extends LinearOpMode {

    /**
     * Defines Hadron Knights robot
     */
    protected HkRobot robot;

    /**
     * Defines drive train
     */
    protected Drivetrain drivetrain;

    /**
     * Hk Vuforia
     */
    protected HkVision hkVision;

    /**
     * Defines alliance color
     */
    protected HkColor allianceColor;

    /**
     * Pauses the Linear Op Mode until start has been pressed or until the current thread
     * is interrupted.
     */
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new HkRobot(this, true);
        drivetrain = robot.getDrivetrain();

        //hkVision = new HkVisionSkystone(this, true, true);
        if (hkVision != null) {
            hkVision.init();
        }

        telemetry.addData("Initialization", "HK Robot created and initialized");
        telemetry.update();


        if (hkVision != null) {
            hkVision.activate();
            sleepWithActiveCheck(500);
        }

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        telemetry.addData("Waiting for Play", "Wait for Referees and then Press Play");
        telemetry.update();

        AutoTransitioner.transitionOnStop(this, "1 - TeleOp Linear");
        waitForStart();

        runAutonomous();
    }

    /**
     * Implements autonomous tasks.
     */
    public abstract void runAutonomous();

    public void sleepWithActiveCheck(long timeInMills) {
            double wait = timeInMills / 1000.0;
            double startTime = this.time;
            while (opModeIsActive() && this.time - startTime < wait) {
                sleep(1);
            }
    }
}
