package org.firstinspires.ftc.teamcode.clawMachine;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Claw Machine", group="Linear OpMode")

public class ClawMachine extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftDriveMotor = null;
    private DcMotor rightDriveMotor = null;

    private CRServo xAxisPositioner = null;
    private DcMotor yAxisPositioner = null;

    private TouchSensor leftLimit = null;
    private TouchSensor rightLimit = null;

    public DistanceSensor clawLimit = null;

    private CRServo clawRappel = null;
    private Servo clawPincers = null;

    private static Gamepad previousGamepad1 = new Gamepad();
    private static Gamepad currentGamepad1 = new Gamepad();

    private final double MAX_LEFT_DRIVE_POWER = 0.5;
    private final double MAX_RIGHT_DRIVE_POWER = 0.5;
    private final double MAX_X_POSITIONER_POWER = 0.5;
    private final double MAX_Y_POSITIONER_POWER = 0.5;

    private final double PINCERS_START_POS = 0;
    private final double PINCERS_POS_OPEN = 1;
    private final double PINCERS_POS_CLOSED = 0;

    private double xAxisPosPower = 0.3;
    private double yAxisPosPower = 0.3;

    private double clawRappelPower = 0.3;

    private boolean leftLimitIsPressed = false;
    private boolean rightLimitIsPressed = false;

    private boolean pincersHandled = false;

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

            try {
                previousGamepad1.copy(currentGamepad1);
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }
            try {
                currentGamepad1.copy(gamepad1);
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }

            telemetry.addData("colorLimit distance", clawLimit.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }

    public void driveWithSticks() {
        //drive forward and backward
        if (gamepad1.left_stick_y > 0.1) {
            leftDriveMotor.setPower(MAX_LEFT_DRIVE_POWER * gamepad1.left_stick_y);
            rightDriveMotor.setPower(MAX_RIGHT_DRIVE_POWER * gamepad1.left_stick_y);
        } else if (gamepad1.left_stick_y < -0.1) {
            leftDriveMotor.setPower(-(MAX_LEFT_DRIVE_POWER * -gamepad1.left_stick_y));
            rightDriveMotor.setPower(-(MAX_RIGHT_DRIVE_POWER * -gamepad1.left_stick_y));
        } else {
            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);
        }

        //turn
        leftDriveMotor.setPower(-(MAX_LEFT_DRIVE_POWER * gamepad1.right_stick_x));
        rightDriveMotor.setPower(MAX_RIGHT_DRIVE_POWER * gamepad1.right_stick_x);
    }

    public void positionClaw() {
        if (gamepad1.dpad_left && !leftLimit.isPressed()) {
            xAxisPositioner.setPower(xAxisPosPower);
        } else if (gamepad1.dpad_right && !rightLimit.isPressed()) {
            xAxisPositioner.setPower(-xAxisPosPower);
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
        } else if (gamepad1.right_trigger > 0.1 && clawLimit.getDistance(DistanceUnit.INCH) > 1.5) {
            clawRappel.setPower((clawRappelPower));
        } else {
            clawRappel.setPower(0);
        }

        if (currentGamepad1.a && !previousGamepad1.a) {
            pincersHandled = !pincersHandled; //handled = open
        }

        //activate claw pincers
        if (gamepad1.a && !pincersHandled) {
            clawPincers.setPosition(PINCERS_POS_OPEN);
        } else if (gamepad1.a && pincersHandled) {
            clawPincers.setPosition(PINCERS_POS_CLOSED);
        }
    }

//    public double normalize(double upperPowerLimit, double givenPower) {
//        double adjustedPower = givenPower;
//
//        if (givenPower > upperPowerLimit) {
//            adjustedPower = givenPower * upperPowerLimit;
//        }
//
//        return adjustedPower;
//    }

    public void innit() {
        leftDriveMotor = hardwareMap.get(DcMotor.class, "leftDriveMotor");
        rightDriveMotor = hardwareMap.get(DcMotor.class, "rightDriveMotor");
        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        yAxisPositioner = hardwareMap.get(DcMotor.class, "yAxisPositioner");
        xAxisPositioner = hardwareMap.get(CRServo.class, "xAxisPositioner");

        leftLimit = hardwareMap.get(TouchSensor.class, "leftLimit");
        rightLimit = hardwareMap.get(TouchSensor.class, "rightLimit");

        clawLimit = hardwareMap.get(DistanceSensor.class, "clawLimit");

        clawRappel = hardwareMap.get(CRServo.class, "clawRappel");
        clawPincers = hardwareMap.get(Servo.class, "clawPincers");
    }
}
