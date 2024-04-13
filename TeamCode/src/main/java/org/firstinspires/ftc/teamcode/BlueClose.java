
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

import java.security.spec.PSSParameterSpec;
import java.util.List;


@Autonomous(name = "BlueCloseTest", group = "BlueClose", preselectTeleOp = "FTC23020")
public class BlueClose extends LinearOpMode {

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

        drive.setPoseEstimate(new Pose2d(12, 65.3, Math.toRadians(270)));

        Trajectory M1 = drive.trajectoryBuilder(new Pose2d(12, 65.3, Math.toRadians(270)))//M.b.p
                .lineToLinearHeading(new Pose2d(52, 39.3, Math.toRadians(0)))

                .build();

        Trajectory M2 = drive.trajectoryBuilder(M1.end())
                .lineToLinearHeading(new Pose2d(31, 36.3, Math.toRadians(225)))

                .build();

        Trajectory M3 = drive.trajectoryBuilder(M2.end())
                .lineToLinearHeading(new Pose2d(21, 37.3, Math.toRadians(225)))

                .build();

        Trajectory M4 = drive.trajectoryBuilder(M3.end())
                .lineToLinearHeading(new Pose2d(17.4, 37, Math.toRadians(180)))

                .build();

        Trajectory M5 = drive.trajectoryBuilder(M4.end())
                .lineToLinearHeading(new Pose2d(-51, 38, Math.toRadians(200))) // 흰색 픽셀 잡는 위치

                .build();

        Trajectory M6 = drive.trajectoryBuilder(M5.end())
                .lineToLinearHeading(new Pose2d(-57.8,40,Math.toRadians(170))) // 흰색 픽셀 잡는 위치

                .build();

        Trajectory M7 = drive.trajectoryBuilder(M6.end())
                .lineToLinearHeading(new Pose2d(42,40,Math.toRadians(180))) // 48,-10 parking center

                .build();

        Trajectory M8 = drive.trajectoryBuilder(M7.end())
                .lineToLinearHeading(new Pose2d(55,30.9,Math.toRadians(0))) // 흰색 픽셀 내려놓는 위치

                .build();

        Trajectory M9 = drive.trajectoryBuilder(M8.end())
                .lineToLinearHeading(new Pose2d(45,57,Math.toRadians(180))) // 48,-10 parking center

                .build();

        Trajectory M10 = drive.trajectoryBuilder(M9.end())
                .lineToLinearHeading(new Pose2d(57,62.3,Math.toRadians(180))) // 48,-10 parking center

                .build();

        Trajectory L1 = drive.trajectoryBuilder(new Pose2d(12,65.3,Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(52.5,46,Math.toRadians(0)))
                .build();

        Trajectory L2 = drive.trajectoryBuilder(L1.end())
                .lineToLinearHeading(new Pose2d(40,41.3,Math.toRadians(225)))
                .build();

        Trajectory L3 = drive.trajectoryBuilder(L2.end())
                .lineToLinearHeading(new Pose2d(31,46.3,Math.toRadians(225)))
                .build();

        Trajectory L4 = drive.trajectoryBuilder(L3.end())
                .lineToLinearHeading(new Pose2d(38,60.3,Math.toRadians(180)))
                .build();

        Trajectory L5 = drive.trajectoryBuilder(L4.end())
                .lineToLinearHeading(new Pose2d(-34,60.3,Math.toRadians(180)))
                .build();

        Trajectory L6 = drive.trajectoryBuilder(L5.end())
                .lineToLinearHeading(new Pose2d(-51, 38, Math.toRadians(200)))
                .build();

        Trajectory L7 = drive.trajectoryBuilder(L6.end())
                .lineToLinearHeading(new Pose2d(-57.8,33,Math.toRadians(170)))
                .build();

        Trajectory L8 = drive.trajectoryBuilder(L7.end())
                .lineToLinearHeading(new Pose2d(-34,60.3,Math.toRadians(0)))
                .build();

        Trajectory L9 = drive.trajectoryBuilder(L8.end())
                .lineToLinearHeading(new Pose2d(50,60.3,Math.toRadians(0)))
                .build();

        Trajectory L10 = drive.trajectoryBuilder(L9.end())
                .lineToLinearHeading(new Pose2d(55,35.3,Math.toRadians(0)))
                .build();

        Trajectory L11 = drive.trajectoryBuilder(L10.end())
                .lineToLinearHeading(new Pose2d(46,60.3,Math.toRadians(180)))
                .build();

        Trajectory L12 = drive.trajectoryBuilder(L11.end())
                .lineToLinearHeading(new Pose2d(58,60.3,Math.toRadians(180)))
                .build();


        //오른쪽

        Trajectory R1 = drive.trajectoryBuilder(new Pose2d(12,65.3, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(12,40,Math.toRadians(200)))
                .build();

        Trajectory R2 = drive.trajectoryBuilder(R1.end())
                .lineToLinearHeading(new Pose2d(52.7,30,Math.toRadians(0)))
                .build();

        Trajectory R3 = drive.trajectoryBuilder(R2.end())
                .lineToLinearHeading(new Pose2d(11.2,58, Math.toRadians(180)))
                .build();


        Trajectory R4 = drive.trajectoryBuilder(R3.end())
                .lineToLinearHeading(new Pose2d(-35,56,Math.toRadians(180)))
                .build();

        Trajectory R5 = drive.trajectoryBuilder(R4.end())
                .lineToLinearHeading(new Pose2d(-48,35,Math.toRadians(200)))
                .build();

        Trajectory R6 = drive.trajectoryBuilder(R5.end())
                .lineToLinearHeading(new Pose2d(-51,30,Math.toRadians(210))) //흰색픽셀 잡는 위치
                .build();

        Trajectory R7 = drive.trajectoryBuilder(R6.end())
                .lineToLinearHeading(new Pose2d(-57.8,33,Math.toRadians(170))) //흰색픽셀 잡는 위치
                .build();

        Trajectory R8 = drive.trajectoryBuilder(R7.end())
                .lineToLinearHeading(new Pose2d(-39.8,58,Math.toRadians(0)))
                .build();

        Trajectory R9 = drive.trajectoryBuilder(R8.end())
                .splineToConstantHeading(new Vector2d(11.2,58),Math.toRadians(0))
                .addSpatialMarker(new Vector2d(11.2,58),() ->{
                    armadjust(1,600,0.5);
                })
                .splineToConstantHeading(new Vector2d(53.5,43.3),Math.toRadians(0))
                .build();

        Trajectory R10 = drive.trajectoryBuilder(R9.end())
                .lineToLinearHeading(new Pose2d(46,60.3,Math.toRadians(180)))
                .build();

        Trajectory R11 = drive.trajectoryBuilder(R10.end())
                .lineToLinearHeading(new Pose2d(58,60.3,Math.toRadians(180)))
                .build();
        /*
        Trajectory R1 = drive.trajectoryBuilder(new Pose2d(12,65.3, Math.toRadians(270)))
                .splineTo(new Vector2d(9,42.3),Math.toRadians(225))
                    .build();

        Trajectory R2 = drive.trajectoryBuilder(R1.end())
                .lineToLinearHeading(new Pose2d(52.7,30,Math.toRadians(0)))
                    .build();

        Trajectory R3 = drive.trajectoryBuilder(R2.end())
                .lineToLinearHeading(new Pose2d(11.2,58, Math.toRadians(180)))
                    .build();


        Trajectory R4 = drive.trajectoryBuilder(R3.end())
                .splineToConstantHeading(new Vector2d(-35,58),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-48,35),Math.toRadians(200))
                .build();

        Trajectory R5 = drive.trajectoryBuilder(R4.end())
                .lineToLinearHeading(new Pose2d(-51,30,Math.toRadians(210))) //흰색픽셀 잡는 위치
                        .build();

        Trajectory R6 = drive.trajectoryBuilder(R5.end())
                .lineToLinearHeading(new Pose2d(-57.8,33,Math.toRadians(170))) //흰색픽셀 잡는 위치
                .build();

        Trajectory R7 = drive.trajectoryBuilder(R6.end())
                .lineToLinearHeading(new Pose2d(-39.8,58,Math.toRadians(0)))
                .build();

        Trajectory R8 = drive.trajectoryBuilder(R7.end())
                .splineToConstantHeading(new Vector2d(11.2,58),Math.toRadians(0))
                .addSpatialMarker(new Vector2d(11.2,58),() ->{
                    armadjust(1,600,0.5);
                })
                .splineToConstantHeading(new Vector2d(53.5,43.3),Math.toRadians(0))
                .build();

        Trajectory R9 = drive.trajectoryBuilder(R8.end())
                .lineToLinearHeading(new Pose2d(46,60.3,Math.toRadians(180)))
                .build();

        Trajectory R10 = drive.trajectoryBuilder(R9.end())
                .lineToLinearHeading(new Pose2d(58,60.3,Math.toRadians(180)))
                .build();

*/
        initTfod();

        while (!isStarted() && !isStopRequested()) {
            // Wait for the DS start button to be touched.
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.addData("position", biconPosition);
            telemetry.update();

            telemetryTfod();

           /* if (currentGamepad2.x && !previousGamepad2.x) {
                start_delay = start_delay + 500;
                telemetry.addData("delay", start_delay);
                telemetry.update();
            } */    //시작 딜레이, 한번 누를때마다 0.5초

            gripper(leftclose, rightclose);
            // Push telemetry to the Driver Station.
            telemetry.update();
        }

        //customSleep(start_delay);  //시작 딜레이

        // Share the CPU.
        sleep(20);

        if (biconPosition == 1) {  //왼쪽 코드

            gripper(leftclose, rightclose);
            armadjust(1, 400, 0.65);

            drive.followTrajectory(L1);
            gripper(leftclose, rightopen);
            armadjust(1, 400, 0.65);
            customSleep(200);

            drive.followTrajectory(L2);
            armadjust(1,100,0.55);

            drive.followTrajectory(L3);
            gripper(leftopen,rightopen);
            customSleep(200);
            armadjust(1,300,0.55);

            drive.followTrajectory(L4);

            drive.followTrajectory(L5);

            drive.followTrajectory(L6);
            armadjust(1, 0, 0.5);

            drive.followTrajectory(L7);
            gripper(leftopen,rightclose);
            customSleep(300);
            armadjust(1, 400, 0.3);

            drive.followTrajectory(L8);
            armadjust(1, 500, 0.5);

            drive.followTrajectory(L9);
            armadjust(1,600,0.5);

            drive.followTrajectory(L10);
            gripper(leftopen,rightopen);
            customSleep(200);
            armadjust(1,600,0.45);
            customSleep(200);

            drive.followTrajectory(L11);

            drive.followTrajectory(L12);
            armadjust(1, 0, 0.5);
        }

        else if (biconPosition == 2) {  //중간코드
            gripper(leftclose, rightclose);
            armadjust(1, 400, 0.65);

            drive.followTrajectory(M1);
            gripper(leftclose, rightopen);
            armadjust(1, 400, 0.65);
            customSleep(200);

            drive.followTrajectory(M2);
            armadjust(1, 100, 0.55);

            drive.followTrajectory(M3);
            gripper(leftopen,rightopen);
            customSleep(200);
            armadjust(1, 300, 0.55);

            drive.followTrajectory(M4);

            drive.followTrajectory(M5);
            armadjust(1, 0, 0.5);

            drive.followTrajectory(M6);
            gripper(leftopen,rightclose);
            customSleep(400);
            armadjust(1, 400, 0.3);

            drive.followTrajectory(M7);
            armadjust(1, 600, 0.5);

            drive.followTrajectory(M8); //흰 픽셀 내려놓기
            gripper(leftopen,rightopen);
            customSleep(200);
            armadjust(1,600,0.45);
            customSleep(200);

            drive.followTrajectory(M9);

            drive.followTrajectory(M10); // 블루 코너 주차 위치
            armadjust(1, 0, 0.5);
        }
        else {  //오른쪽 코드

            gripper(leftclose, rightclose);
            armadjust(1,100,0.5);

            drive.followTrajectory(R1);
            gripper(leftopen, rightclose);
            customSleep(200);

            armadjust(1,400,0.65);

            drive.followTrajectory(R2);
            gripper(leftopen, rightopen);
            armadjust(1,500,0.5);

            drive.followTrajectory(R3);

            drive.followTrajectory(R4);
            armadjust(1,0,0.5);

            drive.followTrajectory(R5);

            drive.followTrajectory(R6);

            drive.followTrajectory(R7);
            gripper(leftclose, rightclose);
            customSleep(400);
            armadjust(0.5,400,0.3);

            drive.followTrajectory(R8);

            drive.followTrajectory(R9);
            gripper(leftopen, rightopen);
            customSleep(200);
            armadjust(1,500,0.45);
            customSleep(200);

            drive.followTrajectory(R10);
            drive.followTrajectory(R11);


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

            if (x > 150 && x < 370) {
                biconPosition = 2;  // 가운데
            } else if (x >= 370) {
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