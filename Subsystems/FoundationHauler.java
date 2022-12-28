package com.hadronknights.ftc.robot.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


public class FoundationHauler extends BaseSubsystem {

    /**
     * Servo to rotate the right grabber
     */
    private Servo rightPullerServo;
    private Servo leftPullerServo;

    /**
     * Constructor
     *
     * @param opMode
     */
    public FoundationHauler(OpMode opMode, boolean isAutonomous) {
        super(opMode);
        this.rightPullerServo = opMode.hardwareMap.servo.get("RightPullerServo");
        this.leftPullerServo = opMode.hardwareMap.servo.get("LeftPullerServo");

        moveUp();
    }

    public void grabFoundation() {
        moveDown();
    }

    public void releaseFoundation() {
        moveUp();
    }

    public void moveDown() {
        rightPullerServo.setPosition(0.281);
        leftPullerServo.setPosition(0.742);
    }

    public void moveUp() {
        rightPullerServo.setPosition(0.993);
        leftPullerServo.setPosition(0.052);
    }

    public void moveDownOutward() {
        rightPullerServo.setPosition(0.458);
        leftPullerServo.setPosition(0.566);
    }

    public void moveDownInward() {
        rightPullerServo.setPosition(0.026);
        leftPullerServo.setPosition(0.999);
    }


    public void reset() {
        moveUp();
    }

    public void stop() {
        // do nothing
    }
}
