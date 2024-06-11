package org.firstinspires.ftc.teamcode.clawMachine;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Claw Machine", group="Linear OpMode")

public class ClawMachine extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftDriveMotor = null;
    private DcMotor rightDriveMotor = null;

    private CRServo xAxisPositioner = null;
    private DcMotor yAxisPositioner = null;

    private TouchSensor leftLimit = null;
    private TouchSensor rightLimit = null;

    private CRServo clawRappel = null;
    private Servo clawPincers = null;

    private static Gamepad previousGamepad1 = new Gamepad();
    private static Gamepad currentGamepad1 = new Gamepad();

    //wheel info here

    private final double PINCERS_START_POS = 0;
    private final double PINCER_POS_OPEN = 1;

    private double leftDrivePower = 0;
    private double rightDrivePower = 0;

    private double xAxisPosPower = 0.3;
    private double yAxisPosPower = 0.3;

    private double clawRappelPower = 0.3;

    private boolean leftLimitIsPressed = false;
    private boolean rightLimitIsPressed = false;

    @Override
    public void runOpMode() {
        innit();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            driveWithSticks();
            positionClaw();
            deployClaw();

            //previousGamepad1.copy(currentGamepad1);
            //currentGamepad1.copy(gamepad1);
            //may need ^^^ for isHandled?
        }
    }

    public void driveWithSticks() {
        //drive code here
    }

    public void positionClaw() {
        if (gamepad1.dpad_left && !leftLimit.isPressed() && !rightLimitIsPressed) {
            xAxisPositioner.setPower(-xAxisPosPower);
        } else if (gamepad1.dpad_right && !leftLimit.isPressed() && !rightLimitIsPressed) {
            xAxisPositioner.setPower(xAxisPosPower);
        } else {
            xAxisPositioner.setPower(0);
        }

        if (gamepad1.dpad_up) {
            yAxisPositioner.setPower(yAxisPosPower);
        } else if (gamepad1.dpad_down) {
            yAxisPositioner.setPower(-yAxisPosPower);
        } else {
            yAxisPositioner.setPower(0);
        }
    }

    public void deployClaw() {
        //move claw up and down
        if (gamepad1.left_trigger > 0.1) {
            clawRappel.setPower(-clawRappelPower);
        } else if (gamepad1.right_trigger > 0.1) {
            clawRappel.setPower((clawRappelPower));
        } else {
            clawRappel.setPower(0);
        }

        //activate claw pincers
        if (gamepad1.a &&) {
            clawPincers.setPosition(PINCER_POS_OPEN);
        } else if (gamepad1.a &&)
    }

    public void innit() {
        leftDriveMotor = hardwareMap.get(DcMotor.class, "leftDriveMotor");
        rightDriveMotor = hardwareMap.get(DcMotor.class, "rightDriveMotor");
        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        yAxisPositioner = hardwareMap.get(DcMotor.class, "yAxisPositioner");
        xAxisPositioner = hardwareMap.get(CRServo.class, "xAxisPositioner");

        leftLimit = hardwareMap.get(TouchSensor.class, "leftLimit");
        rightLimit = hardwareMap.get(TouchSensor.class, "rightLimit");

        clawRappel = hardwareMap.get(CRServo.class, "clawRappel");
        clawPincers = hardwareMap.get(Servo.class, "clawPincers");
    }
}
