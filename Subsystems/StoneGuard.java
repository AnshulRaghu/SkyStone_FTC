package com.hadronknights.ftc.robot.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


public class StoneGuard extends BaseSubsystem {

    /**
     * Servo to rotate the right guard
     */
    private Servo rightGuardServo;

    /**
     * Servo to rotate the left guard
     */
    private Servo leftGuardServo;

    /**
     * Constructor
     *
     * @param opMode
     */
    public StoneGuard(OpMode opMode, boolean isAutonomous) {
        super(opMode);

        this.rightGuardServo = opMode.hardwareMap.servo.get("RightGuardServo");
        this.leftGuardServo = opMode.hardwareMap.servo.get("LeftGuardServo");

        if (isAutonomous) {
            moveStraightFront();
        }
        else {
            reset();
        }
    }

    public void centerStone() {
        rightGuardServo.setPosition(0.9);
        leftGuardServo.setPosition(0.096);
    }

    public void centerStoneForCap() {
        leftGuardServo.setPosition(0.14);
        rightGuardServo.setPosition(0.946);
    }


    public void moveOutSideways() {
        rightGuardServo.setPosition(0.306);
        leftGuardServo.setPosition(0.663);
    }

    public void extendForParking() {
        moveOutSideways();
    }

    public void moveStraightFront() {
        rightGuardServo.setPosition(0.69);
        leftGuardServo.setPosition(0.3);
    }

    public void moveOutFront() {
        rightGuardServo.setPosition(0.565);
        leftGuardServo.setPosition(0.407);
    }

    public void reset() {
        rightGuardServo.setPosition(0.071);
        leftGuardServo.setPosition(0.9);
    }

    public void stop() {
        // do nothing
    }
}
