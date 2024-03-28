
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    private static final String[] LABELS = {
            "RED",
    };

    private TfodProcessor tfod;

    private VisionPortal visionPortal;

    double rightopen = 0.5;
    double leftopen = 0.5;

    double rightclose = 0.85;
    double leftclose = 0.15;

    public void customSleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, 0));



        //right traj

        Trajectory R1 = drive.trajectoryBuilder(new Pose2d(0, 0))  //to backdrop
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))

                .addTemporalMarker(0.1, () -> {
                    // Run your action in here!

                })

                .build();

        Trajectory rotateR = drive.trajectoryBuilder(R1.end())
                .lineToLinearHeading(new Pose2d(43.5, -30, Math.toRadians(170)))

                .addTemporalMarker(0, () -> {
                })

                .addTemporalMarker(1, () -> {
                })

                .build();

        Trajectory R2 = drive.trajectoryBuilder(rotateR.end())  //to park-1

                .addTemporalMarker(0, () -> {
                })

                .lineToLinearHeading(new Pose2d(43.5, -13, Math.toRadians(270)))

                .addTemporalMarker(0.2, () -> {
                    // Run your action in here!

                })

                .build();

        Trajectory R3 = drive.trajectoryBuilder(R2.end())  //to park-2

                .strafeLeft(15)

                .build();




        //mid traj

        Trajectory M1 = drive.trajectoryBuilder(new Pose2d(11.5, -65))
                .lineToLinearHeading(new Pose2d(11.5, -41, Math.toRadians(90)))

                .addTemporalMarker(1, () -> {
                    // Run your action in here!


                })

                .build();


        Trajectory M2 = drive.trajectoryBuilder(M1.end())

                .addTemporalMarker(0, () -> {
                    // Run your action in here!

                })

                .addTemporalMarker(0.5, () -> {
                })

                .lineToLinearHeading(new Pose2d(44.5, -37, Math.toRadians(0)))


                .build();


        Trajectory M3 = drive.trajectoryBuilder(M2.end())

                .addTemporalMarker(0.3, () -> {
                    // Run your action in here!


                })

                .addTemporalMarker(0.1, () -> {
                })

                .lineToLinearHeading(new Pose2d(43.5, -13, Math.toRadians(270)))



                .build();

        Trajectory M4 = drive.trajectoryBuilder(M3.end())
                .strafeLeft(15)
                .build();

        //left traj

        Trajectory L1 = drive.trajectoryBuilder(new Pose2d(11.5, -65))
                .lineToLinearHeading(new Pose2d(11.5, -41, Math.toRadians(150)))

                .addTemporalMarker(0, () -> {
                    // Run your action in here!


                })


                .build();


        Trajectory L2 = drive.trajectoryBuilder(L1.end())
                .addTemporalMarker(0, () -> {
                    // Run your action in here!

                })

                .addTemporalMarker(0.5, () -> {
                })

                .lineToLinearHeading(new Pose2d(44.5, -30, Math.toRadians(0)))

                .build();


        Trajectory L3 = drive.trajectoryBuilder(L2.end())
                .lineToLinearHeading(new Pose2d(43.5, -13, Math.toRadians(270)))


                .addTemporalMarker(0, () -> {
                    // Run your action in here!


                })

                .addTemporalMarker(0.1, () -> {
                })


                .build();

        Trajectory L4 = drive.trajectoryBuilder(L3.end())
                .strafeLeft(15)
                .build();






        initTfod();

        while (!isStarted() && !isStopRequested()) {
            // Wait for the DS start button to be touched.
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.addData("position", biconPosition);
            telemetry.update();


            telemetryTfod();

            // Push telemetry to the Driver Station.
            telemetry.update();
        }

        // Share the CPU.
        sleep(20);


        if (biconPosition == 1) {  //code RedC_trajLn


            drive.followTrajectory(L1);  //move to backdrop place, extend arm


            drive.followTrajectory(L2);

            customSleep(100);


            drive.followTrajectory(L3);
            drive.followTrajectory(L4);







        }
        else if (biconPosition == 2) {  //code RedC_trajMn


            drive.followTrajectory(M1);  //move to backdrop place, extend arm

            customSleep(100);

            drive.followTrajectory(M2);

            customSleep(100);


            drive.followTrajectory(M3);
            drive.followTrajectory(M4);




        }
        else {  //code RedC_trajRn



            drive.followTrajectory(R1);  //move to backdrop place, extend arm

            customSleep(100);

            drive.followTrajectory(rotateR);


            customSleep(100);


            drive.followTrajectory(R2);

            drive.followTrajectory(R3);



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
