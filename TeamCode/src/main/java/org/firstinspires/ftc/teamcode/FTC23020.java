package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp


public class FTC23020 extends LinearOpMode {
    @Override

    public void runOpMode () throws InterruptedException {
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftRear = hardwareMap.dcMotor.get("leftRear");
        DcMotor rightRear = hardwareMap.dcMotor.get("rightRear");
        DcMotor ARM = hardwareMap.dcMotor.get("ARM");
        Servo shooting = hardwareMap.servo.get("shooting");
        Servo wrist = hardwareMap.servo.get("wrist");
        Servo gripper1 = hardwareMap.servo.get("gripper1");
        Servo gripper2 = hardwareMap.servo.get("gripper2");

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        imu.initialize(parameters);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        ARM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int sPOSION = 0;
        double s2POSION = 0.1;
        boolean swAstatus = false;
        boolean swAcurrent;
        boolean swBstatus = false;
        boolean swBcurrent;
        boolean swUpstatus = false;
        boolean swUpcurrent;
        boolean swDownstatus = false;
        boolean swDowncurrent;
        int targetPosition = 0;
        int currentPosition = 0;
        int gPOSITION = 0;
        int g2POSITION = 0;
        double wPOSITION = 0;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.update();
            swAcurrent = gamepad1.x;
            swBcurrent = gamepad1.b;
            swDowncurrent = gamepad2.dpad_left;
            swUpcurrent = gamepad2.dpad_right;

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double Y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double X = gamepad1.left_stick_x;
            double R = gamepad1.right_stick_x;
            double slow = 0.9 - (0.7 * gamepad1.right_trigger);
            double A = -gamepad2.right_stick_y;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = X * Math.cos(-botHeading) - Y * Math.sin(-botHeading);
            double rotY = X * Math.sin(-botHeading) + Y * Math.cos(-botHeading);

            rotX = rotX * 1.1; // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(R), 1);
            double leftFrontPower = ((rotY + rotX + R) / denominator) * slow;
            double leftRearPower = ((rotY - rotX + R) / denominator) * slow;
            double rightFrontPower = ((rotY - rotX - R) / denominator) * slow;
            double rightRearPower  = ((rotY + rotX - R) / denominator) * slow;
            double armPower = A;

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);
            ARM.setPower(armPower);

            if (gamepad1.dpad_left) {
                shooting.setPosition(0);
            }
            if (gamepad1.dpad_right) {
                shooting.setPosition(0.37);
            }
            while (gamepad2.b) {
                targetPosition = currentPosition - 100;
                ARM.setTargetPosition(targetPosition);
                ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ARM.setPower(0.3);
                currentPosition = ARM.getCurrentPosition();
            }
            while (gamepad2.x) {
                if (targetPosition < -10) {
                    targetPosition = currentPosition + 100;
                    ARM.setTargetPosition(targetPosition);
                    ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ARM.setPower(0.3);
                    currentPosition = ARM.getCurrentPosition();
                } else {
                    targetPosition = -10;
                    ARM.setTargetPosition(targetPosition);
                    ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ARM.setPower(0.3);
                    currentPosition = ARM.getCurrentPosition();
                }
            }

            while (gamepad2.a) {
                targetPosition = -30;
                ARM.setTargetPosition(targetPosition);
                ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ARM.setPower(0.5);
                currentPosition = ARM.getCurrentPosition();
            }

            while (gamepad2.y) {
                targetPosition = -1000;
                ARM.setTargetPosition(targetPosition);
                ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ARM.setPower(0.5);
                currentPosition = ARM.getCurrentPosition();
            }
            while (gamepad2.dpad_up) {
                wPOSITION = 0.65;
                wrist.setPosition(wPOSITION);
            }
            while (gamepad2.dpad_down) {
                wPOSITION = 0.6;
                wrist.setPosition(wPOSITION);
            }
            if (swUpcurrent == true && swUpcurrent != swUpstatus) {
                wPOSITION += -0.05;
                wrist.setPosition(wPOSITION);
            }
            swUpstatus = swUpcurrent;
            if (swDowncurrent == true && swDowncurrent != swDownstatus) {
                wPOSITION += 0.05;
                wrist.setPosition(wPOSITION);
            }
            swDownstatus = swDowncurrent;
            if (swAcurrent == true && swAcurrent != swAstatus) {
                if (gPOSITION == 0) {
                    gPOSITION = 1;
                } else {
                    gPOSITION = 0;
                }
                gripper1.setPosition(gPOSITION);
            }
            if (swBcurrent == true && swBcurrent != swBstatus) {
                if (g2POSITION == 0) {
                    g2POSITION = 1;
                } else {
                    g2POSITION = 0;
                }
                gripper2.setPosition(g2POSITION);
            }
            swAstatus = swAcurrent;
            telemetry.addData("encoder", ARM.getCurrentPosition());
            telemetry.addData("Astatus", swAstatus);
            telemetry.addData("wPosition", wPOSITION);
            telemetry.addData("X", gamepad1.left_stick_x);
            telemetry.addData("leftFrontPower", leftFrontPower);
            telemetry.update();
        }
    }
}
