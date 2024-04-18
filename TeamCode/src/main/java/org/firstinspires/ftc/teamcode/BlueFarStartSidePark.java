
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


@Autonomous(name = "BlueFarStartSidePark", group = "BlueAuto", preselectTeleOp = "FTC23020")
public class BlueFarStartSidePark extends LinearOpMode {

    int biconPosition = 1;
    private static final boolean USE_WEBCAM = true;

    private static final String TFOD_MODEL_ASSET = "5031Blue.tflite";

    private static final String[] LABELS = {"BLUE"};

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

    int start_delay = 0;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2= new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();


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

        drive.setPoseEstimate(new Pose2d(-40, 65.3, Math.toRadians(270)));

        Trajectory L1 = drive.trajectoryBuilder(new Pose2d(-40,65.3,Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-33,40,Math.toRadians(315)))
                .build();

        Trajectory L2 = drive.trajectoryBuilder(L1.end())
                .lineToLinearHeading(new Pose2d(-51,39,Math.toRadians(180)))
                .build();

        Trajectory L21 = drive.trajectoryBuilder(L2.end())
                .lineToLinearHeading(new Pose2d(-55,39,Math.toRadians(180)))
                .build();

        Trajectory L3 = drive.trajectoryBuilder(L2.end())
                .lineToLinearHeading(new Pose2d(-37,12,Math.toRadians(0)))
                .build();

        Trajectory L4 = drive.trajectoryBuilder(L3.end())
                .splineToConstantHeading(new Vector2d(21,12),Math.toRadians(0))
                .addSpatialMarker(new Vector2d(21,12),() ->{
                    armadjust(1,400,0.5);
                })
                .splineToConstantHeading(new Vector2d(51,49),Math.toRadians(0))
                .build();

        Trajectory L5 = drive.trajectoryBuilder(L4.end())
                .lineToLinearHeading(new Pose2d(45,63,Math.toRadians(180)))
                .build();

        Trajectory M1 = drive.trajectoryBuilder(new Pose2d(-40,65.3,Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-40,38,Math.toRadians(270)))
                        .build();

        Trajectory M2 = drive.trajectoryBuilder(M1.end())
                .lineToLinearHeading(new Pose2d(-60,38,Math.toRadians(270)))
                        .build();

        Trajectory M21 = drive.trajectoryBuilder(M2.end())
                .lineToLinearHeading(new Pose2d(-60,14,Math.toRadians(270)))
                .build();

        Trajectory M3 = drive.trajectoryBuilder(M21.end())
                .lineToLinearHeading(new Pose2d(-50,14,Math.toRadians(0)))
                        .build();

        Trajectory M4 = drive.trajectoryBuilder(M3.end())
                .splineToConstantHeading(new Vector2d(21,14),Math.toRadians(0))
                .addSpatialMarker(new Vector2d(21,14),() ->{
                    armadjust(1,400,0.5);
                })
                .splineToConstantHeading(new Vector2d(51,40),Math.toRadians(0))
                .build();

        Trajectory M41 = drive.trajectoryBuilder(M4.end())
                .lineToLinearHeading(new Pose2d(48,40,Math.toRadians(0)))
                .build();

        Trajectory M5 = drive.trajectoryBuilder(M4.end())
                .lineToLinearHeading(new Pose2d(45,63,Math.toRadians(180)))
                .build();

        Trajectory R1 = drive.trajectoryBuilder(new Pose2d(-40,65.3,Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-38,40,Math.toRadians(225)))
                        .build();
        Trajectory R11 = drive.trajectoryBuilder(R1.end())
                .lineToLinearHeading(new Pose2d(-58,40,Math.toRadians(270)))
                    .build();

        Trajectory R2 = drive.trajectoryBuilder(R11.end())
                .lineToLinearHeading(new Pose2d(-58,15,Math.toRadians(270)))
                        .build();

        Trajectory R3 = drive.trajectoryBuilder(R2.end())
                .lineToLinearHeading(new Pose2d(-37,15,Math.toRadians(2)))
                .build();

        Trajectory R4 = drive.trajectoryBuilder(R3.end())
                .splineToConstantHeading(new Vector2d(21,15),Math.toRadians(2))
                .addSpatialMarker(new Vector2d(21,15),() ->{
                    armadjust(1,400,0.5);
                })
                .splineToConstantHeading(new Vector2d(51.5,36),Math.toRadians(2))
                .build();

        Trajectory R5 = drive.trajectoryBuilder(R4.end())
                .lineToLinearHeading(new Pose2d(45,63,Math.toRadians(180)))
                .build();

        //오른쪽



        initTfod();

        while (!isStarted() && !isStopRequested()) {
            // Wait for the DS start button to be touched.
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.addData("position", biconPosition);
            telemetry.update();

            telemetryTfod();

           if (currentGamepad2.x && !previousGamepad2.x) {
                start_delay = start_delay + 500;
                telemetry.addData("delay", start_delay);
                telemetry.update();
            }  //시작 딜레이, 한번 누를때마다 0.5초

            gripper(leftclose, rightclose);
            // Push telemetry to the Driver Station.
            telemetry.update();
        }

        //customSleep(start_delay);  //시작 딜레이

        // Share the CPU.
        sleep(20);

        if (biconPosition == 1) {  //왼쪽 코드
            gripper(leftclose, rightclose);
            armadjust(1, 100, 0.5);

            drive.followTrajectory(L1);
            armadjust(1,0,0.5);
            customSleep(100);
            gripper(leftopen,rightclose);
            customSleep(100);
            armadjust(1,200,0.5);
            customSleep(200);

            drive.followTrajectory(L2);
            armadjust(1,200,0.45);
            customSleep(200);

            drive.followTrajectory(L21);
            gripper(leftclose,rightclose);
            customSleep(200);

            drive.followTrajectory(L3);
            armadjust(1,400,0.3);
            customSleep(100);
            gripper(leftclose,rightclose);
            customSleep(100);

            drive.followTrajectory(L4);
            gripper(leftopen, rightopen);
            customSleep(200);
            armadjust(1,500,0.45);
            customSleep(200);
            armadjust(1,700,0.6);

            drive.followTrajectory(L5);
            armadjust(1,0,0.5);
        }

        else if (biconPosition == 2) {  //중간코드
            gripper(leftclose, rightclose);
            armadjust(1, 100, 0.5);

            drive.followTrajectory(M1);
            armadjust(1,0,0.5);
            customSleep(100);
            gripper(leftopen,rightclose);
            customSleep(100);
            armadjust(1,200,0.5);
            customSleep(100);

            drive.followTrajectory(M2);

            drive.followTrajectory(M21);

            drive.followTrajectory(M3);
            armadjust(1,400,0.3);
            customSleep(100);
            gripper(leftclose,rightclose);
            customSleep(100);

            drive.followTrajectory(M4);
            armadjust(1,400,0.5);
            customSleep(200);
            gripper(leftopen, rightopen);
            customSleep(500);
            armadjust(1,700,0.6);

            drive.followTrajectory(M41);

            drive.followTrajectory(M5);
            armadjust(1,0,0.5);
        }
        else {  //오른쪽 코드
            gripper(leftclose,rightclose);
            armadjust(1,100,0.5);

            drive.followTrajectory(R1);
            armadjust(1,0,0.5);
            customSleep(100);
            gripper(leftopen,rightclose);
            customSleep(100);
            armadjust(1,200,0.5);
            customSleep(100);

            drive.followTrajectory(R11);

            drive.followTrajectory(R2);

            drive.followTrajectory(R3);
            armadjust(1,400,0.3);
            customSleep(100);
            gripper(leftclose,rightclose);
            customSleep(100);

            drive.followTrajectory(R4);
            gripper(leftopen, rightopen);
            customSleep(200);
            armadjust(1,500,0.45);
            customSleep(200);
            armadjust(1,700,0.6);

            drive.followTrajectory(R5);
            armadjust(1,0,0.5);




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

            if (x > 70 && x < 260) {
                biconPosition = 2;  // 가운데
            } else if (x >= 320) {
                biconPosition = 3; // 오른쪽
            } else {
                biconPosition = 1; // 왼쪽
            }


            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("biconPosition", biconPosition);


            telemetry.update();


        }


    }
}