package com.hadronknights.ftc.robot.subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.List;

/**
 * This is an interface that enables the team to replace actual implementation of Vuforia
 * leveraging different computer vision packages such as tensor flow, Doge CV, Open CV etc.
 */

public interface HkVision {
    /**
     * Initializes Vuforia and computer vision libraries.
     */
    void init();

    /**
     * Activates Vuforia and computer vision resources.
     */
    void activate();

    /**
     * Releases Vuforia and computer vision resources.
     */
    void shutdown();

    /**
     * Returns a list of known trackable targets
     *
     * @return
     */
    List<VuforiaTrackable> getAllTargets();
}
