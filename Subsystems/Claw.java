package com.hadronknights.ftc.robot.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Anshul on 9/8/19
 */


public class Claw extends BaseSubsystem {

    //Servos to rotate the transporter arms to let go the team marker.
    private Servo clawServo;


    /**
     * Constructor
     *
     * @param opMode
     */
    public Claw(OpMode opMode, boolean isAutonomous) {
        super(opMode);
        this.clawServo = opMode.hardwareMap.servo.get("ClawServo");
        reset();
    }

    /**
     * Closes the arms of the claw to grab the stone from the wider side. Used for both autonomous and teleOp
     */
    public void getReadyToGrabStone() {
        clawServo.setPosition(0.255);
    }

    public void grabStone() {
        clawServo.setPosition(0.81);
    }

    public void grabStoneLoose() {
        clawServo.setPosition(0.736);
    }

    public void grabStoneEvenMoreLoose() { clawServo.setPosition(0.7);}

    public void releaseStone() {
        clawServo.setPosition(0.255);
    }

    public void reset() {
        clawServo.setPosition(0.1);
    }

    public void closeClawFull() {
        clawServo.setPosition(0.945);
    }

    public void stop() {
        // do nothing

    }
}
