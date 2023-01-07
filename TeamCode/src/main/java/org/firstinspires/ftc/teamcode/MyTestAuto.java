package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name="MyTestAuto")
public class MyTestAuto extends OrcaAutoBase {
    SleevePosition position;

    @Override
    public void runOpMode() throws InterruptedException {
        setup();
//        unlock();
//        sleep(500);
//        waitForStart();
//        SleevePosition position = pipeline.getAnalysis();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.lock();
        drive.raiseHolder();
        drive.turnArm(0.88);
        waitForStart();
//        sleep(2000);
//        position = drive.pipeline.getAnalysis();
//        telemetry.addData("pos", position);
//        telemetry.update();
        Pose2d startPos = new Pose2d(-62, -38, 0);
        drive.setPoseEstimate(startPos);

        TrajectorySequence trajSeq;

        TrajectorySequenceBuilder trajSeqBuilder = drive.trajectorySequenceBuilder(startPos)
//                .setVelConstraint(0)

                .addTemporalMarker(() -> {


                    drive.lock();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(()->{
                    drive.raiseSlider(-(int)(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION));
                })
                .waitSeconds(0.1)
//                .strafeLeft(8.25)
//                .forward(37.5)
                .splineTo(new Vector2d(-24.5, -33), 0.0)
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    drive.unlock();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    drive.turnArm(0.55);
                })
//                .waitSeconds(0.3)
                .addTemporalMarker(() -> {

                    drive.raiseSlider(-ARM_COUNTS_FOR_FIVE_CONES);
                })
                .waitSeconds(0.1)
//                .forward(13.7)
//                .turn(Math.toRadians(-90))
//                .addTemporalMarker(() -> {
//                    drive.lowerConeHolder();
//                })
//                .forward(15.5)
//                .setVelConstraint(drive.getVelocityConstraint(MAX_VEL/6, MAX_ANG_VEL, TRACK_WIDTH))
//                .setAccelConstraint(drive.getAccelerationConstraint(MAX_ACCEL/4))
//
//                .forward(5)

//                .waitSeconds(0.5)
                .resetAccelConstraint()
                .resetVelConstraint()
                .addTemporalMarker(() -> {
                    drive.raiseSlider(-ARM_COUNTS_FOR_FIVE_CONES_DOWN);

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    drive.lock();
                })
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    drive.raiseSlider(-ARM_COUNTS_FOR_LOW_JUNCTION);

                })
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    drive.turnArm(0.22);
                })
                .back(9.5)
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    drive.unlock();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    drive.turnArm(0.55);
                })
//                .waitSeconds(0.2)
                // start here!
                .forward(6.5)
                .setVelConstraint(drive.getVelocityConstraint(MAX_VEL/6, MAX_ANG_VEL, TRACK_WIDTH))
                .setAccelConstraint(drive.getAccelerationConstraint(MAX_ACCEL/4))
                .forward(4.5)
//                .waitSeconds(0.5)
                .resetAccelConstraint()
                .resetVelConstraint()
                .addTemporalMarker(() -> {
                    drive.raiseSlider(-ARM_COUNTS_FOR_FOUR_CONES_DOWN);

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    drive.lock();
                })
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    drive.raiseSlider(-ARM_COUNTS_FOR_LOW_JUNCTION);

                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    drive.turnArm(0.22);
                })
                .back(9.5)
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    drive.unlock();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    drive.turnArm(0.55);
                })
                .waitSeconds(0.2)
                // stop here!
                .forward(6.5)
                .setVelConstraint(drive.getVelocityConstraint(MAX_VEL/6, MAX_ANG_VEL, TRACK_WIDTH))
                .setAccelConstraint(drive.getAccelerationConstraint(MAX_ACCEL/4))
                .forward(4.5)
//                .waitSeconds(0.5)
                .resetAccelConstraint()
                .resetVelConstraint()
                .addTemporalMarker(() -> {
                    drive.raiseSlider(-ARM_COUNTS_FOR_THREE_CONES_DOWN);

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    drive.lock();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    drive.raiseSlider(-ARM_COUNTS_FOR_LOW_JUNCTION);

                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    drive.turnArm(0.22);
                })
                .back(9.5)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    drive.unlock();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    drive.turnArm(0.55);
                })
                .waitSeconds(0.2)
                .forward(6.5)
                .setVelConstraint(drive.getVelocityConstraint(MAX_VEL/6, MAX_ANG_VEL, TRACK_WIDTH))
                .setAccelConstraint(drive.getAccelerationConstraint(MAX_ACCEL/4))
                .forward(4.5)
//                .waitSeconds(0.5)
                .resetAccelConstraint()
                .resetVelConstraint()
                .addTemporalMarker(() -> {
                    drive.raiseSlider(-ARM_COUNTS_FOR_TWO_CONES_DOWN);

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    drive.lock();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    drive.raiseSlider(-ARM_COUNTS_FOR_LOW_JUNCTION);

                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    drive.turnArm(0.22);
                })
                .back(9.5)
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    drive.unlock();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    drive.turnArm(0.55);
                })
                .waitSeconds(0.2);
//        if(position == SleevePosition.LEFT){
//            trajSeq = trajSeqBuilder.forward(42)
//                    .build();
//        }else if(position == SleevePosition.RIGHT){
//            trajSeq = trajSeqBuilder.back(5)
//                    .build();
//        }else{
            trajSeq = trajSeqBuilder
                    .build();
//                    }

//                .addTemporalMarker(()->{ // Fourth stack cone
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_ONE_CONES);
//                })
//                .waitSeconds(0.3)
//                .turn(Math.toRadians(121))
//                .addTemporalMarker(() -> {
//                    drive.lock();
//                })
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_LOW_JUNCTION);
//                })
//                .waitSeconds(0.3)
//                .turn(-Math.toRadians(121))
//                .addTemporalMarker(() -> {
//                    drive.unlock();
//                })


        if(isStopRequested()) return;
        SleevePosition position = pipeline.getAnalysis();
        telemetry.addData("Position", position);
        sendTelemetry();
        drive.followTrajectorySequence(trajSeq);
    }
}