package com.hadronknights.ftc.opmode.teleop;

import com.hadronknights.ftc.robot.HkRobot;
import com.hadronknights.ftc.robot.drivetrain.Drivetrain;
import com.hadronknights.ftc.robot.subsystem.Capper;
import com.hadronknights.ftc.robot.subsystem.Claw;
import com.hadronknights.ftc.robot.subsystem.FoundationHauler;
import com.hadronknights.ftc.robot.subsystem.Lift;
import com.hadronknights.ftc.robot.subsystem.ParkExtender;
import com.hadronknights.ftc.robot.subsystem.StoneGuard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class controls the Hadron Knights robot during the driver controlled part of the game
 * and uses Linear OpMode
 * <p>
 * Created by Rahul V, Arko, Anshul, Rahul D, Aarush on 8/19/2017.
 */

@TeleOp(name = "1 - TeleOp Linear", group = "HK TeleOp")
public class TeleopLinear extends BaseTeleop {
    private static final double FORWARD_POWER_FACTOR = 1;
    private static final double BACKWARD_POWER_FACTOR = 1;
    private static final double STRAFE_POWER_FACTOR = 1;
    private static final double TURN_POWER_FACTOR = 0.8;

    private static final double SLOW_MODE_DRIVE_POWER_FACTOR = 0.3;
    private static final double SLOW_MODE_STRAFE_POWER_FACTOR = 0.3;
    private static final double SLOW_MODE_TURN_POWER_FACTOR = 0.3;

    private HkRobot robot;
    private Drivetrain drivetrain;


    /**
     * Constructor
     */
    public TeleopLinear() {
        super();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize
        Servo capstoneHolderServo = this.hardwareMap.servo.get("CapstoneHolderServo");
        capstoneHolderServo.setPosition(0.151999) ;

        robot = new HkRobot(this, false);
        Capper capper = robot.getCapper();
        drivetrain = robot.getDrivetrain();
        Claw claw = robot.getClaw();
        Lift lift = robot.getLift();
        FoundationHauler foundationHauler = robot.getFoundationHauler();
        StoneGuard stoneGuard = robot.getStoneGuard();
        ParkExtender parkExtender = robot.getParkExtender();

        telemetry.addData("Initilization", "Initialized teleop");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Waiting for Play", "Wait for Referees and then Press Play");
        telemetry.update();
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        telemetry.addData("Loop", "Starting teleop loop");
        telemetry.update();
        while (opModeIsActive()) {

            // Driver 1 controls
            drive();

            if (claw != null && stoneGuard != null && foundationHauler != null) {
                if (gamepad1.a) {
                    claw.grabStone();
                }
                if (gamepad1.y) {
                    foundationHauler.moveUp();
                    stoneGuard.centerStoneForCap();
                    claw.grabStoneLoose();
                }
                if (gamepad1.b) {
                    claw.releaseStone();
                }
                if (gamepad1.x) {
                    foundationHauler.moveDown();
                }
            }

            if (foundationHauler != null) {
                if (gamepad1.right_bumper) {
                    foundationHauler.moveDown();
                }
                if (gamepad1.left_bumper) {
                    foundationHauler.moveUp();
                }
            }

            if (parkExtender != null) {
                if (gamepad1.left_trigger > 0) {
                    parkExtender.moveTape(-gamepad1.left_trigger);
                } else if (gamepad1.right_trigger > 0) {
                    parkExtender.moveTape(gamepad1.right_trigger);
                } else {
                    parkExtender.stop();
                }
            }


            // Driver 2 controls
            if (lift != null) {
                lift.moveLift(-gamepad2.left_stick_y);
            }

            if (foundationHauler != null) {
                if (gamepad2.dpad_down) {
                    foundationHauler.moveDown();
                }
                if (gamepad2.dpad_up) {
                    foundationHauler.moveUp();
                }
                if (gamepad2.dpad_right) {
                    foundationHauler.moveDownInward();
                }
                if (gamepad2.dpad_left) {
                    foundationHauler.moveDownOutward();
                }
            }

            if (parkExtender != null) {
                if (gamepad2.left_trigger > 0) {
                    parkExtender.moveTape(-gamepad2.left_trigger);
                } else if (gamepad2.right_trigger > 0) {
                    parkExtender.moveTape(gamepad2.right_trigger);
                } else {
                    parkExtender.stop();
                }
            }

            if (claw != null && stoneGuard != null && foundationHauler != null) {
                if (gamepad2.a) {
                    claw.closeClawFull();
                }
                if (gamepad2.b) {
                    claw.releaseStone();
                }
                if (gamepad2.x) {
                    claw.grabStone();
                    foundationHauler.moveUp();
                    stoneGuard.reset();
                }
            }

            if (capper != null) {
                if (gamepad2.right_bumper) {
                    capper.releaseCapstone();
                }
                if (gamepad2.left_bumper) {
                    capper.reset();
                }
            }
        }

        if (robot != null) {
            robot.stop();
        }
    }

    private void drive() {
        if (drivetrain != null) {
            if (gamepad2.y) {
                drivetrain.drive(gamepad1.left_stick_y * SLOW_MODE_DRIVE_POWER_FACTOR, gamepad1.left_stick_x * SLOW_MODE_TURN_POWER_FACTOR, gamepad1.right_stick_x * SLOW_MODE_STRAFE_POWER_FACTOR, 0);
            } else {
                if (gamepad1.left_stick_y > 0) {
                    drivetrain.drive(gamepad1.left_stick_y * BACKWARD_POWER_FACTOR, gamepad1.left_stick_x * TURN_POWER_FACTOR * 0.8, gamepad1.right_stick_x * STRAFE_POWER_FACTOR, 0);
                } else {
                    drivetrain.drive(gamepad1.left_stick_y * FORWARD_POWER_FACTOR, gamepad1.left_stick_x * TURN_POWER_FACTOR * 0.8, gamepad1.right_stick_x * STRAFE_POWER_FACTOR, 0);
                }
            }
        }
    }

    private void sleepUsingTimeout(long timeInMills) {
        double wait = timeInMills / 1000.0;
        double startTime = this.time;
        while (opModeIsActive() && this.time - startTime < wait) {
            drive();
        }
    }
}


