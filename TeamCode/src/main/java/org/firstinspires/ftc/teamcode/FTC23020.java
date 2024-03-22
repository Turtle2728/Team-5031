package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
//@TeleOp


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
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.initialize(parameters);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            double Y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double X = gamepad1.left_stick_x;
            double R = gamepad1.right_stick_x;
            double slow = 0.8 * (0.6 * gamepad1.right_trigger);
            double A = -gamepad2.right_stick_y;

            if (gamepad1.options) {
                imu.resetYaw();
            }
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = X * Math.cos(-botHeading) - Y * Math.sin(-botHeading);
            double rotY = X * Math.sin(-botHeading) + Y * Math.cos(-botHeading);

            rotX = rotX * 1.1; // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(R), 1);
            double leftFrontPower = ((rotY + rotX - R) / denominator) * slow;
            double leftRearPower = ((rotY - rotX - R) / denominator) * slow;
            double rightFronttPower = ((rotY - rotX + R) / denominator) * slow;
            double rightRearPower  = ((rotY + rotX + R) / denominator) * slow;
            double armPower = A;

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFronttPower);
            rightRear.setPower(rightRearPower);
            ARM.setPower(armPower);


        }
    }
}
