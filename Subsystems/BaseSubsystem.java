package com.hadronknights.ftc.robot.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is abstract class for all subsystem to extend from and includes common attributes.
 */

public abstract class BaseSubsystem {
    protected static final double NO_TIMEOUT = 10000;
    /**
     * Opmode
     */
    protected OpMode opMode;

    /**
     * Telemetry
     */
    protected Telemetry telemetry;

    protected BaseSubsystem(OpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
    }

    public BaseSubsystem() {

    }

    /**
     * Downcast to linear opmode in autonomous.
     *
     * @return
     */
    protected LinearOpMode getLinearOpMode() {
        return (LinearOpMode) opMode;
    }

    public void sleepWithActiveCheck(long timeInMills) {
        if (getLinearOpMode().opModeIsActive()) {
            getLinearOpMode().sleep(timeInMills);
        }
    }

    public abstract void stop();
}
