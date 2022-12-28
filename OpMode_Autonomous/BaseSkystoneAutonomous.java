package com.hadronknights.ftc.opmode.autonomous;

import com.hadronknights.ftc.robot.HkPosition;

/**
 * This abstract class implements the common logic across all autonomous scenarios for Skystone season.
 * <p>
 * Created by Rahul V, Arko, Rahul D, & Aarush on 8/27/2017.
 */

public abstract class BaseSkystoneAutonomous extends BaseAutonomous {

    protected Double desiredOrientationAngle;
    protected HkPosition firstSkystonePosition;

    /**
     * Common post initialization logic to expand our robot
     */
    protected void postInitAutonomous() {
        desiredOrientationAngle = drivetrain.getRobotOrientation();
        robot.getClaw().reset();
        robot.getFoundationHauler().reset();
        sleepWithActiveCheck(50);

    }
}

