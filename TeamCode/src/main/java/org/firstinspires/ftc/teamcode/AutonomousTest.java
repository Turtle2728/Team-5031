
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;


import java.util.List;

@Autonomous(name = "RedCloseTest", group = "RedClose")
public class AutonomousTest extends LinearOpMode {


    int biconPosition = 1;
    private static final boolean USE_WEBCAM = true;

    private static final String TFOD_MODEL_ASSET = "5031Red.tflite";

    private static final String[] LABELS = {"RED"};

    private TfodProcessor tfod;

    private VisionPortal visionPortal;

    double rightopen = 0.6;
    double leftopen = 0.4;

    double rightclose = 1;
    double leftclose = 0;


    DcMotor armMotor;
    Servo wrist;
    Servo right_claw;
    Servo left_claw;

    public void armadjust(double armPower, int armTarget, double wristTarget) {

        armMotor.setTargetPosition(armTarget);
        wrist.setPosition(wristTarget);
        armMotor.setPower(armPower);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void gripper(double Left_Target, double Right_target) {
        left_claw.setPosition(Left_Target);
        right_claw.setPosition(Right_target);
    }

   public void customSleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void runOpMode() {

        armMotor = hardwareMap.dcMotor.get("ARM");

        left_claw = hardwareMap.servo.get("gripper2");
        right_claw = hardwareMap.servo.get("gripper1");
        wrist = hardwareMap.servo.get("wrist");

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        drive.setPoseEstimate(new Pose2d(12, -60, Math.toRadians(90)));





        //right traj

        Trajectory M1 = drive.trajectoryBuilder(new Pose2d(12, -60, Math.toRadians(90)))//M.b.p
                .lineToLinearHeading(new Pose2d(12, -31, Math.toRadians(90)))

                .build();

        Trajectory backCenter = drive.trajectoryBuilder(new Pose2d(12, -31, Math.toRadians(90)))//M.b.d
                .lineToLinearHeading(new Pose2d(52, -35, Math.toRadians(0)))

                .build();

        Trajectory M2 = drive.trajectoryBuilder(new Pose2d(52, -31, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(20, -33, Math.toRadians(180)))

                .build();

        Trajectory M3 = drive.trajectoryBuilder(new Pose2d(20, -33, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-59, -31.5, Math.toRadians(177)))

                .build();

        Trajectory M4 = drive.trajectoryBuilder(new Pose2d(-59, -31.5, Math.toRadians(177)))
                .lineToLinearHeading(new Pose2d(-40, -33, Math.toRadians(0)))

                .build();

        Trajectory M5 = drive.trajectoryBuilder(new Pose2d(-40, -33, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(52, -38, Math.toRadians(0)))

                .build();

        Trajectory M6 = drive.trajectoryBuilder(new Pose2d(52,-38,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(50,-10,Math.toRadians(180)))

                .build();

        Trajectory L1 = drive.trajectoryBuilder(new Pose2d(12,-60,90))
                .splineToLinearHeading(new Pose2d(12,-31,Math.toRadians(90)),Math.toRadians(180))

                .build();

        Trajectory backleft = drive.trajectoryBuilder(new Pose2d(12,-31,180))
                .lineToLinearHeading(new Pose2d(50,-30,Math.toRadians(0)))

                .build();

        Trajectory L2 = drive.trajectoryBuilder(new Pose2d(50,-30,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(20,-33,Math.toRadians(180)))

                .build();






        initTfod();

        while (!isStarted() && !isStopRequested()) {
            // Wait for the DS start button to be touched.
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.addData("position", biconPosition);
            telemetry.update();


            telemetryTfod();

            gripper(leftclose, rightclose);
            // Push telemetry to the Driver Station.
            telemetry.update();
        }

        // Share the CPU.
        sleep(20);

        if (biconPosition == 1) {  //code RedC_trajLn
            gripper(leftclose,rightclose);
            armadjust(1,+100,0.7);

            drive.followTrajectory(L1);

            gripper(leftopen,rightclose);
            customSleep(300);

            armadjust(1,+500,0.65);
            gripper(leftopen,rightclose);

            drive.followTrajectory(backleft);

            customSleep(300);

            gripper(leftopen,rightopen);
            customSleep(300);

            drive.followTrajectory(L2);
            armadjust(1,100,0.55);
            customSleep(300);


        }
        else if (biconPosition == 2) {  //code RedC_trajMn
            gripper(leftclose, rightclose);
            armadjust(1, 100, 0.8);

            drive.followTrajectory(M1);

            armadjust(1,100,0.5);
            gripper(leftopen, rightclose);
            customSleep(300);

            armadjust(1, 500, 0.65);
            gripper(leftopen,rightclose);

            drive.followTrajectory(backCenter);

            customSleep(200);

            gripper(leftopen,rightopen);
            customSleep(200);

            drive.followTrajectory(M2);
            armadjust(1, 100, 0.55);
            customSleep(200);

            drive.followTrajectory(M3);
            armadjust(1, 70, 0.55);
            customSleep(200);

            gripper(leftclose, rightclose);
            customSleep(1000);

            drive.followTrajectory(M4);
            customSleep(200);
            armadjust(1, 500, 0.65);


            drive.followTrajectory(M5);
            customSleep(200);
            armadjust(1, 700, 0.55);
            customSleep(1000);

            gripper(leftopen, rightclose);
            customSleep(500);
            armadjust(1, 700, 0.50);
            customSleep(1000);

            drive.followTrajectory(M6);
            customSleep(200);
            armadjust(1,0,0.55);
            customSleep(1000);

        }
        else {  //code RedC_trajRn



        }



        visionPortal.close();

    } //run opmode end

    private void initTfod() {

        tfod = new TfodProcessor.Builder()

                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }


        builder.addProcessor(tfod);

        visionPortal = builder.build();

        telemetry.update();
    }

    private void telemetryTfod() {

        biconPosition = 1;

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2;

            if (x > 0 && x < 300) {
                biconPosition = 2;
            } else if (x >= 300) {
                biconPosition = 3;
            } else {
                biconPosition = 1;
            }


            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("biconPosition", biconPosition);


            telemetry.update();


        }


    }
}