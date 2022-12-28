package com.hadronknights.ftc.robot.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem extends BaseSubsystem {

    private DcMotor frontRightIntakeMotor;
    private DcMotor frontLeftIntakeMotor;
    private int i = 0;

    /**
     * Constructor
     *
     * @param opMode
     * @param isAutonomous
     */
    public IntakeSubsystem(OpMode opMode, boolean isAutonomous) {
        super(opMode);

        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        frontRightIntakeMotor = opMode.hardwareMap.dcMotor.get("FrontRightIntakeMotor");
        frontLeftIntakeMotor = opMode.hardwareMap.dcMotor.get("FrontLeftIntakeMotor");


        telemetry.addData("Initialization", "Motors Found");
        telemetry.update();

        frontRightIntakeMotor.setDirection(DcMotor.Direction.REVERSE);

        frontRightIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void rotate(double power) {
        frontRightIntakeMotor.setPower(power);
        frontLeftIntakeMotor.setPower(power);
    }

    @Override
    public void stop() {
        frontRightIntakeMotor.setPower(0);
        frontLeftIntakeMotor.setPower(0);
    }
}