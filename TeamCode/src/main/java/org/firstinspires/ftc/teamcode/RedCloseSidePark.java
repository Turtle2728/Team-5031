
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous(name = "RedCloseSidePark", group = "RedAuto", preselectTeleOp = "FTC23020")
public class RedCloseSidePark extends LinearOpMode {


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

        drive.setPoseEstimate(new Pose2d(12, -60, Math.toRadians(90)));

        Trajectory M1 = drive.trajectoryBuilder(new Pose2d(12, -60, Math.toRadians(90)))//M.b.p
                .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(90)))

                .build();

        Trajectory backCenter = drive.trajectoryBuilder(M1.end())//M.b.d
                .lineToLinearHeading(new Pose2d(50.5, -31, Math.toRadians(0)))

                .build();

        Trajectory M2 = drive.trajectoryBuilder(backCenter.end())
                .lineToLinearHeading(new Pose2d(20, -35, Math.toRadians(180)))

                .build();

        Trajectory M3 = drive.trajectoryBuilder(M2.end())
                .lineToLinearHeading(new Pose2d(-53, -30.5, Math.toRadians(170)))
                .build();

        Trajectory M31 = drive.trajectoryBuilder(M3.end())
                .lineToLinearHeading(new Pose2d(-58.5, -32.5, Math.toRadians(190)))
                .build();


        Trajectory M4 = drive.trajectoryBuilder(M31.end())
                .lineToLinearHeading(new Pose2d(-40, -33, Math.toRadians(0)))
                .build();

        Trajectory M5 = drive.trajectoryBuilder(M4.end())
                .lineToLinearHeading(new Pose2d(45, -33, Math.toRadians(0)))

                .build();

        Trajectory M51 = drive.trajectoryBuilder(M5.end())
                .lineToLinearHeading(new Pose2d(53, -36, Math.toRadians(0)))

                .build();

        Trajectory M52 = drive.trajectoryBuilder(M51.end())
                .lineToLinearHeading(new Pose2d(42, -36, Math.toRadians(0)))

                .build();

        Trajectory M6 = drive.trajectoryBuilder(M51.end())
                .lineToLinearHeading(new Pose2d(51,-55,Math.toRadians(180))) // 48,-10 parking center

                .build();

        Trajectory L1 = drive.trajectoryBuilder(new Pose2d(12,-60,Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(14,-27,Math.toRadians(180)),Math.toRadians(90))

                .build();

        Trajectory backleft = drive.trajectoryBuilder(L1.end())
                .lineToLinearHeading(new Pose2d(52.5,-22,Math.toRadians(-3)))

                .build();

        Trajectory L2 = drive.trajectoryBuilder(backleft.end())
                .lineToLinearHeading(new Pose2d(20,-55,Math.toRadians(180)))

                .build();

        Trajectory L3 = drive.trajectoryBuilder(L2.end())
                .lineToLinearHeading(new Pose2d(-28,-55,Math.toRadians(180)))

                .build();

        Trajectory L4 = drive.trajectoryBuilder(L3.end())
                .lineToLinearHeading(new Pose2d(-51.5, -30, Math.toRadians(142))) // 흰색픽셀잡기전
                .build();

        Trajectory L5 = drive.trajectoryBuilder(L4.end())
                .lineToLinearHeading(new Pose2d(-57,-28,Math.toRadians(190))) //흰색 픽셀잡는 곳

                .build();

        Trajectory L6 = drive.trajectoryBuilder(L5.end())
                .lineToLinearHeading(new Pose2d(-50,-54,Math.toRadians(3)))

                .build();

        Trajectory L7 = drive.trajectoryBuilder(L6.end())
                .lineToLinearHeading(new Pose2d(48,-54,Math.toRadians(3)))

                .build();

        Trajectory L8 = drive.trajectoryBuilder(L7.end())
                .lineToLinearHeading(new Pose2d(53,-36,Math.toRadians(3)))

                .build();
        Trajectory L9 = drive.trajectoryBuilder(L8.end())
                .lineToLinearHeading(new Pose2d(50,-55,Math.toRadians(180)))

                .build();

        Trajectory R1 = drive.trajectoryBuilder(new Pose2d(12,-60,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(52,-38,Math.toRadians(0)))

                .build();

        Trajectory R2 = drive.trajectoryBuilder(R1.end())
                .lineToLinearHeading(new Pose2d(32,-38,Math.toRadians(90)))
                .build();

        Trajectory R3 = drive.trajectoryBuilder(R2.end())
                .lineToLinearHeading(new Pose2d(30,-40,Math.toRadians(117)))
                .build();

        Trajectory R4 = drive.trajectoryBuilder(R3.end())
                .lineToLinearHeading(new Pose2d(15,-55,Math.toRadians(180)))

                .build();

        Trajectory R5 = drive.trajectoryBuilder(R4.end())
                        .lineToLinearHeading(new Pose2d(-33,-55,Math.toRadians(180)))
                                .build();

        Trajectory R6 = drive.trajectoryBuilder(R5.end())
                        .lineToLinearHeading(new Pose2d(-54,-44,Math.toRadians(138)))
                                .build();
        Trajectory R7 = drive.trajectoryBuilder(R6.end())
                        .lineToLinearHeading(new Pose2d(-57,-35, Math.toRadians(160)))
                                .build();
        Trajectory R8 = drive.trajectoryBuilder(R7.end())
                        .lineToLinearHeading(new Pose2d(-54, -38,Math.toRadians(138)))
                                .build();
        Trajectory R9 = drive.trajectoryBuilder(R8.end())
                        .lineToLinearHeading(new Pose2d(-37, -58, Math.toRadians(0)))
                                .build();
        Trajectory R10 = drive.trajectoryBuilder(R9.end())
                        .lineToLinearHeading(new Pose2d(15,-58, Math.toRadians(0)))
                                .build();
        Trajectory R11 = drive.trajectoryBuilder(R10.end())
                        .lineToLinearHeading(new Pose2d(53.2,-28,Math.toRadians(0)))
                                .build();
        Trajectory R12 = drive.trajectoryBuilder(R11.end())
                        .lineToLinearHeading(new Pose2d(51,-58, Math.toRadians(180)))
                                .build();
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

        // Share the CPU
        sleep(20);

        if (biconPosition == 1) {  //left code
            gripper(leftclose, rightclose);
            armadjust(1, 100, 0.5);

            drive.followTrajectory(L1);

            gripper(leftopen, rightclose);
            customSleep(200);

            armadjust(1, 400, 0.65);

            drive.followTrajectory(backleft);
            gripper(leftopen,rightopen);
            customSleep(200);
            armadjust(1, 600, 0.5);
            customSleep(200);

            drive.followTrajectory(L2);
            armadjust(1, 150, 0.5);

            drive.followTrajectory(L3);

            drive.followTrajectory(L4);
            armadjust(1, 0, 0.5);

            drive.followTrajectory(L5);
            gripper(leftclose, rightclose);
            customSleep(400);
            armadjust(1, 400, 0.3);

            drive.followTrajectory(L6);

            drive.followTrajectory(L7);
            armadjust(1, 600, 0.6);
            customSleep(200);
            drive.followTrajectory(L8);
            armadjust(1, 600, 0.45);
            gripper(leftopen, rightopen);
            customSleep(400);
            armadjust(1, 600, 0.4);
            customSleep(1000);

            drive.followTrajectory(L9);
            customSleep(200);
            armadjust(1,0,0.55);
            customSleep(200);
        }
        else if (biconPosition == 2) {  //code RedC_trajMn
            gripper(leftclose, rightclose);
            armadjust(1, 100, 0.5);

            drive.followTrajectory(M1);

            gripper(leftopen, rightclose);
            customSleep(200);

            armadjust(1, 400, 0.65);
            gripper(leftopen,rightclose);

            drive.followTrajectory(backCenter);
            customSleep(200);
            armadjust(1, 400, 0.5);
            customSleep(200);
            gripper(leftopen,rightopen);
            customSleep(600);
            armadjust(1, 600, 0.4);
            customSleep(200);


            drive.followTrajectory(M2);
            armadjust(1, 150, 0.5);
            customSleep(200);

            drive.followTrajectory(M3);
            armadjust(1, 0, 0.5);

            drive.followTrajectory(M31);
            customSleep(200);
            gripper(leftclose, rightclose);
            customSleep(600);
            armadjust(0.5, 400, 0.3);

            drive.followTrajectory(M4);
            customSleep(200);

            drive.followTrajectory(M5);
            armadjust(1, 600, 0.55);

            drive.followTrajectory(M51);
            gripper(leftopen, rightopen);
            customSleep(500);
            armadjust(1, 600, 0.4);
            customSleep(1000);

            drive.followTrajectory(M52);

            drive.followTrajectory(M6);
            customSleep(200);
            armadjust(1,0,0.55);
            customSleep(1000);

        }
        else {  //code RedC_trajRn
            gripper(leftclose, rightclose);
            customSleep(200);
            armadjust(1, 500, 0.55);

            drive.followTrajectory(R1);

            gripper(leftclose,rightopen);
            customSleep(200);

            drive.followTrajectory(R2);
            armadjust(1,0,0.5);
            customSleep(200);

            drive.followTrajectory(R3);
            gripper(leftopen,rightopen);
            customSleep(200);
            armadjust(1,200,0.5);
            customSleep(200);


            drive.followTrajectory(R4);

            drive.followTrajectory(R5);

            drive.followTrajectory(R6);
            armadjust(1,0,0.55);

            drive.followTrajectory(R7);
            gripper(leftclose,rightclose);
            customSleep(200);

            drive.followTrajectory(R8);
            armadjust(1,400,0.3);

            drive.followTrajectory(R9);

            drive.followTrajectory(R10);
            armadjust(1,600,0.5);

            drive.followTrajectory(R11);
            gripper(leftopen,rightopen);
            customSleep(200);
            armadjust(1,600,0.45);
            customSleep(200);

            drive.followTrajectory(R12);
            armadjust(1,0,0.5);
            customSleep(1000);












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

            if (x > 120 && x < 300) {
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