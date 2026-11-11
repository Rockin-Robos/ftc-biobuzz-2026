package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motor Test", group = "Test")
public class QuantifiTest extends LinearOpMode {

    // ================ SETTINGS ================
    static final double INTAKE_SERVO_POWER = 1.0;
    static final double PUSHER_POWER = 0.8;

    static final double SHOOTER_FULL = 0.53;
    static final double SHOOTER_HALF = 0.50;
    static final double SHOOTER_25 = 0.55;
    static final double SHOOTER_75 = 0.52;
    static final double SHOOTER_LOW = 0.35;
    static final double SHOOTER_MED = 0.45;
    static final double SHOOTER_HIGH = 0.65;

    // ==========================================

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake, shooter;

    private CRServo leftIntakeServo, rightIntakeServo, pusherServo;

    // Toggle states
    private double intakePower = 0.0;
    private double shooterPower = 0.0;
    private int pusherDir = 0;

    // Previous button states for detecting new presses
    private boolean prevRightTrigger = false;
    private boolean prevLeftTrigger = false;

    private boolean prevA = false;
    private boolean prevX = false;
    private boolean prevB = false;
    private boolean prevY = false;

    @Override
    public void runOpMode() {

        // -------- HARDWARE CONFIGURATION --------
        frontLeft = hardwareMap.tryGet(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.tryGet(DcMotor.class, "frontRight");
        backLeft = hardwareMap.tryGet(DcMotor.class, "backLeft");
        backRight = hardwareMap.tryGet(DcMotor.class, "backRight");

        intake = hardwareMap.tryGet(DcMotor.class, "intake");
        shooter = hardwareMap.tryGet(DcMotor.class, "shooter");

        leftIntakeServo =
                hardwareMap.tryGet(CRServo.class, "leftIntakeServo");
        rightIntakeServo =
                hardwareMap.tryGet(CRServo.class, "rightIntakeServo");
        pusherServo =
                hardwareMap.tryGet(CRServo.class, "pusherServo");

        // -------- MOTOR DIRECTIONS --------
        if (frontLeft != null)
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        if (backLeft != null)
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        if (rightIntakeServo != null)
            rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        if (shooter != null)
            shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // -------- DEVICE CHECK --------
        telemetry.addLine("DEVICE CHECK");
        showDeviceStatus();
        telemetry.addLine("Press PLAY to start");
        telemetry.update();

        waitForStart();

        // -------- MAIN LOOP --------
        while (opModeIsActive()) {

            // =====================================
            // MECANUM DRIVE
            // =====================================
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator =
                    Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            power(frontLeft, (y + x + rx) / denominator);
            power(backLeft, (y - x + rx) / denominator);
            power(frontRight, (y - x - rx) / denominator);
            power(backRight, (y + x - rx) / denominator);

            // =====================================
            // INTAKE MOTOR: TOGGLE IN / OUT / STOP
            // Right trigger = IN
            // Left trigger = OUT
            // =====================================
            boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;
            boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;

            if (rightTriggerPressed && !prevRightTrigger) {
                intakePower = (intakePower > 0.0) ? 0.0 : 1.0;
            }

            if (leftTriggerPressed && !prevLeftTrigger) {
                intakePower = (intakePower < 0.0) ? 0.0 : -1.0;
            }

            prevRightTrigger = rightTriggerPressed;
            prevLeftTrigger = leftTriggerPressed;

            power(intake, intakePower);

            // =====================================
            // SHOOTER: PRESS TO TOGGLE SPEED
            // A = 100%, X = 50%
            // B = 25%,  Y = 75%
            // =====================================
            boolean aPressed = gamepad1.a && !prevA;
            boolean xPressed = gamepad1.x && !prevX;
            boolean bPressed = gamepad1.b && !prevB;
            boolean yPressed = gamepad1.y && !prevY;

            if (aPressed) {
                shooterPower =
                        (shooterPower == SHOOTER_FULL)
                                ? 0.0 : SHOOTER_FULL;
            } else if (xPressed) {
                shooterPower =
                        (shooterPower == SHOOTER_HALF)
                                ? 0.0 : SHOOTER_HALF;
            } else if (bPressed) {
                shooterPower =
                        (shooterPower == SHOOTER_25)
                                ? 0.0 : SHOOTER_25;
            } else if (yPressed) {
                shooterPower =
                        (shooterPower == SHOOTER_75)
                                ? 0.0 : SHOOTER_75;

                shooterPower =  (shooterPower == SHOOTER_75) ? 0.0 : SHOOTER_75;

            }

            prevA = gamepad1.a;
            prevX = gamepad1.x;
            prevB = gamepad1.b;
            prevY = gamepad1.y;

            power(shooter, shooterPower);

            // =====================================
            // INTAKE SERVOS
            // Right bumper = both IN
            // Left bumper = both OUT
            // L3 = left servo IN
            // R3 = right servo IN
            // =====================================
            double leftServoPower = 0.0;
            double rightServoPower = 0.0;

            if (gamepad1.right_bumper) {
                leftServoPower = INTAKE_SERVO_POWER;
                rightServoPower = INTAKE_SERVO_POWER;
            } else if (gamepad1.left_bumper) {
                leftServoPower =- INTAKE_SERVO_POWER;
                rightServoPower = -INTAKE_SERVO_POWER;
            }

            // Temporary substitutes for M1 and M2:
            // L3 = left intake servo
            // R3 = right intake servo
            if (gamepad1.left_stick_button) {
                leftServoPower = INTAKE_SERVO_POWER;
            }

            if (gamepad1.right_stick_button) {
                rightServoPower = INTAKE_SERVO_POWER;
            }

            power(leftIntakeServo, leftServoPower);
            power(rightIntakeServo, rightServoPower);

            // =====================================
            // PUSHER
            // D-pad UP = reverse continuously
            // D-pad DOWN = forward continuously
            // D-pad LEFT = stop
            // =====================================
            if (gamepad1.dpad_up) {
                pusherDir = -1;
            }

            if (gamepad1.dpad_down) {
                pusherDir = 1;
            }

            if (gamepad1.dpad_left) {
                pusherDir = 0;
            }

            power(pusherServo, pusherDir * PUSHER_POWER);

            // =====================================
            // TELEMETRY
            // =====================================
            telemetry.addData("Intake motor", "%.2f", intakePower);
            telemetry.addData(
                    "Shooter power", "%.0f%%", shooterPower * 100);
            telemetry.addData(
                    "Left intake servo", "%.2f", leftServoPower);
            telemetry.addData(
                    "Right intake servo", "%.2f", rightServoPower);

            telemetry.addData(
                    "Pusher",
                    pusherDir == -1 ? "REVERSE" :
                    pusherDir == 1 ? "FORWARD" : "STOPPED");

            if (shooter != null) {
                telemetry.addData(
                        "Shooter encoder", shooter.getCurrentPosition());
            }

            telemetry.addLine("");
            showDeviceStatus();
            telemetry.update();
        }
    }

    // -------- SAFE MOTOR/SERVO POWER --------
    private void power(DcMotorSimple device, double p) {
        if (device != null) {
            device.setPower(p);
        }
    }

    // -------- DEVICE STATUS --------
    private void showDeviceStatus() {
        status("frontLeft", frontLeft);
        status("frontRight", frontRight);
        status("backLeft", backLeft);
        status("backRight", backRight);
        status("intake", intake);
        status("shooter", shooter);
        status("leftIntakeServo", leftIntakeServo);
        status("rightIntakeServo", rightIntakeServo);
        status("pusherServo", pusherServo);
    }

    private void status(String name, Object device) {
        telemetry.addData(
                name,
                device == null ? "MISSING (check config)" : "OK");
    }
}
