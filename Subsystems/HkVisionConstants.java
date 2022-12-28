package com.hadronknights.ftc.robot.subsystem;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * All constants necessary for Vuforia and Tensor Flow for the season defined here in this interface.
 */

public interface HkVisionConstants {

    String TFOD_MODEL_ASSET = "Skystone.tflite";
    String LABEL_FIRST_ELEMENT = "Stone";
    String LABEL_SECOND_ELEMENT = "Skystone";

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    boolean PHONE_IS_PORTRAIT = false;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    float mmPerInch = 25.4f;
    float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    float bridgeZ = 6.42f * mmPerInch;
    float bridgeY = 23 * mmPerInch;
    float bridgeX = 5.18f * mmPerInch;
    float bridgeRotY = 59;                                 // Units are degrees
    float bridgeRotZ = 180;

    // Constants for perimeter targets
    float halfField = 72 * mmPerInch;
    float quadField = 36 * mmPerInch;
}
