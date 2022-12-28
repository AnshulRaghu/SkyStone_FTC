package com.hadronknights.ftc.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BaseTeleop extends LinearOpMode {

    /**
     * Sleep only if op mode is active.
     *
     * @param timeInMills
     */
    public void sleepWithActiveCheck(long timeInMills) {
        if (opModeIsActive()) {
            sleep(timeInMills);
        }
    }


}
