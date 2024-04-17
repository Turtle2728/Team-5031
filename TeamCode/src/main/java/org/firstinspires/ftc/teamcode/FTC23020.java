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
        Servo Shooting = hardwareMap.servo.get("Shooting");
        Servo ShootingAngle = hardwareMap.servo.get("ShootingAngle");
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

        waitForStart();

        boolean swUpstatus = false;
        boolean swUpcurrent;
        boolean swDownstatus = false;
        boolean swDowncurrent;

        int targetPosition = 0;
        int currentPosition = 0;

        double wPOSITION = 0;

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("encoder", ARM.getCurrentPosition());
            telemetry.addData("wPosition", wPOSITION);
            telemetry.addData("X", gamepad1.left_stick_x);
            telemetry.addData("gripper1", gripper1);
            telemetry.addData("gripper2", gripper2);
            telemetry.update();

            swDowncurrent = gamepad2.dpad_left;
            swUpcurrent = gamepad2.dpad_right;

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double Y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double X = gamepad1.left_stick_x;
            double R = gamepad1.right_stick_x;
            double slow = 1.5 - (0.7 * gamepad1.right_trigger);

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = X * Math.cos(-botHeading) - Y * Math.sin(-botHeading);
            double rotY = X * Math.sin(-botHeading) + Y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(R), 1);
            double leftFrontPower = ((rotY + rotX + R) / denominator) * slow;
            double leftRearPower = ((rotY - rotX + R) / denominator) * slow;
            double rightFrontPower = ((rotY - rotX - R) / denominator) * slow;
            double rightRearPower  = ((rotY + rotX - R) / denominator) * slow;

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);

            if (gamepad1.dpad_left) {
                Shooting.setPosition(0);
            }
            if (gamepad1.dpad_right) {
                Shooting.setPosition(0.37);
            }
            if (gamepad1.dpad_up) {
                ShootingAngle.setPosition(0.65);
            }
            if (gamepad1.dpad_down) {
                ShootingAngle.setPosition(1);
            }

            if (gamepad2.b) {
                targetPosition = currentPosition + 100;
                ARM.setTargetPosition(targetPosition);
                ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ARM.setPower(0.6);
                currentPosition = ARM.getCurrentPosition();
            }
            if (gamepad2.x) {
                if (targetPosition > 10) {
                    targetPosition = currentPosition - 100;
                    ARM.setTargetPosition(targetPosition);
                    ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ARM.setPower(0.3);
                    currentPosition = ARM.getCurrentPosition();
                } else {
                    targetPosition = 10;
                    ARM.setTargetPosition(targetPosition);
                    ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ARM.setPower(0.3);
                    currentPosition = ARM.getCurrentPosition();
                }
            }

            if (gamepad2.a) {
                targetPosition = 20;
                ARM.setTargetPosition(targetPosition);
                ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ARM.setPower(1);
                currentPosition = ARM.getCurrentPosition();
            }
            //리깅단축키
            if (gamepad2.right_bumper) {
                targetPosition = 2000;
                ARM.setTargetPosition(targetPosition);
                ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ARM.setPower(1);
                currentPosition = ARM.getCurrentPosition();
            }
            //백스테이지 높은 층 단축키

            if (gamepad2.y) {
                targetPosition = 2480;
                ARM.setTargetPosition(targetPosition);
                ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ARM.setPower(1);
                currentPosition = ARM.getCurrentPosition();
                wrist.setPosition(0.4);
            }

            //강제로 팔 내리는 키
            if (gamepad2.left_stick_button) {
                targetPosition = currentPosition - 20;
                ARM.setTargetPosition(targetPosition);
                ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ARM.setPower(0.3);
                currentPosition = ARM.getCurrentPosition();
            }

            if (gamepad2.right_stick_button) {
                targetPosition = currentPosition + 20;
                ARM.setTargetPosition(targetPosition);
                ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ARM.setPower(0.3);
                currentPosition = ARM.getCurrentPosition();
            }

            if (gamepad2.options) {
                ARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                currentPosition=0;
            }

            while (gamepad2.dpad_up) {
                wPOSITION = 0.65;
                wrist.setPosition(wPOSITION);
            }
            while (gamepad2.dpad_down) {
                wPOSITION = 0.55;
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

            if (gamepad1.left_bumper) {
               gripper2.setPosition(0.4);
            } else {
                gripper2.setPosition(0.1);
            }

            if (gamepad1.right_bumper) {
                gripper1.setPosition(0.6);
            } else {
                gripper1.setPosition(0.95);
            }

            }

        }
    }
