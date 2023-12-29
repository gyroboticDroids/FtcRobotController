package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Red Left Auto Old")
@Disabled
public class RedLeftAutoOld extends LinearOpMode{
    DistanceSensor leftSensor;
    DistanceSensor rightSensor;
    Servo leftGripper;
    Servo rightGripper;
    Pose2d currentPose;
    int purplePos;
    boolean armUp = false;
    int slidePos;
    double rampPos = 0.501;
    Servo arm;
    double slidePosError;
    double slidePower;
    DcMotor slideMotor1;
    DcMotor slideMotor2;
    double dist;
    @Override
    public void runOpMode() throws InterruptedException {
        leftSensor = hardwareMap.get(DistanceSensor.class, "checkLeft");
        rightSensor = hardwareMap.get(DistanceSensor.class, "checkRight");

        leftGripper = hardwareMap.servo.get("leftGripperServo");
        rightGripper = hardwareMap.servo.get("rightGripperServo");

        leftGripper.setPosition(Constants.GRIPPER_LEFT_CLOSE_POSITION);
        rightGripper.setPosition(Constants.GRIPPER_RIGHT_CLOSE_POSITION);
        arm = hardwareMap.servo.get("armServo");

        slideMotor1 = hardwareMap.dcMotor.get("leftSlideMotor");
        slideMotor2 = hardwareMap.dcMotor.get("rightSlideMotor");

        slideMotor2.setDirection(DcMotor.Direction.REVERSE);

        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        TrajectorySequence approachSpikeMarks = drive.trajectorySequenceBuilder(new Pose2d(-38.5, -63.75, Math.toRadians(-90)))
                .strafeRight(3)
                .forward(32.5)
                .build();

        drive.followTrajectorySequence(approachSpikeMarks);

        if (leftSensor.getDistance(DistanceUnit.INCH) < 5) {
            purplePos = 1;
            dist = -4;
            TrajectorySequence placePurpleLeft = drive.trajectorySequenceBuilder(approachSpikeMarks.end())
                    .turn(Math.toRadians(90))
                    .forward(3.5)
                    .build();

            drive.followTrajectorySequence(placePurpleLeft);

            leftGripper.setPosition(Constants.GRIPPER_LEFT_OPEN_POSITION);

            TrajectorySequence backLeft = drive.trajectorySequenceBuilder(placePurpleLeft.end())
                    .waitSeconds(1)
                    .back(3.5)
                    .strafeLeft(28)
                    .turn(Math.toRadians(180))
                    .build();

            drive.followTrajectorySequence(backLeft);

            currentPose = backLeft.end();

        } else if (rightSensor.getDistance(DistanceUnit.INCH) < 5) {
            purplePos = 2;
            dist = 4;
            TrajectorySequence placePurpleRight = drive.trajectorySequenceBuilder(approachSpikeMarks.end())
                    .turn(Math.toRadians(-90))
                    .forward(3.5)
                    .build();

            drive.followTrajectorySequence(placePurpleRight);

            leftGripper.setPosition(Constants.GRIPPER_LEFT_OPEN_POSITION);

            TrajectorySequence backRight = drive.trajectorySequenceBuilder(placePurpleRight.end())
                    .waitSeconds(1)
                    .back(3.5)
                    .strafeRight(28)
                    .build();

            drive.followTrajectorySequence(backRight);

            currentPose = backRight.end();

        } else {
            purplePos = 3;
            dist = 0;
            leftGripper.setPosition(Constants.GRIPPER_LEFT_OPEN_POSITION);
            TrajectorySequence back = drive.trajectorySequenceBuilder(approachSpikeMarks.end())
                    .back(28)
                    .turn(Math.toRadians(-90))
                    .build();

            drive.followTrajectorySequence(back);

            currentPose = back.end();
        }


        TrajectorySequence driveToBoard = drive.trajectorySequenceBuilder(currentPose)
                .back(23.5)
                .strafeLeft(50)
                .forward(47)
                .waitSeconds(0.5)
                .forward(47)
                .strafeRight(26 + dist)
                .forward(5)
                .build();

        drive.followTrajectorySequence(driveToBoard);

        slidePos = 600;
        armUp = true;
        while ((slideMotor1.getCurrentPosition() < slidePos - 30 || rampPos < 0.743) && opModeIsActive()) {
            arm.setPosition(Ramp(Constants.ARM_DOWN_POS, Constants.ARM_UP_POS, armUp));
            slidePosError = slidePos - slideMotor1.getCurrentPosition();
            slidePower = slidePosError * 3 / 500;
            slidePower = Math.min(Math.max(slidePower, -0.6), 0.75);

            slideMotor1.setPower(slidePower);
            slideMotor2.setPower(slidePower);
            telemetry.addData("Slide Pos", slideMotor1.getCurrentPosition());
            telemetry.addData("Arm Pos", arm.getPosition());
            telemetry.update();
        }
        slideMotor1.setPower(0);
        slideMotor2.setPower(0);

        TrajectorySequence placeYellow = drive.trajectorySequenceBuilder(driveToBoard.end())
                .forward(9)
                .build();
        drive.followTrajectorySequence(placeYellow);
        currentPose = placeYellow.end();


        rightGripper.setPosition(Constants.GRIPPER_RIGHT_OPEN_POSITION);

        TrajectorySequence backUp = drive.trajectorySequenceBuilder(currentPose)
                .waitSeconds(0.4)
                .back(5)
                .build();

        drive.followTrajectorySequence(backUp);
    }

    public double Ramp(double firstPos, double secondPos, boolean selectPos)
    {
        if(selectPos){
            rampPos += 0.01;
        }
        else{
            rampPos -= 0.01;
        }
        rampPos = Math.min(Math.max(rampPos, firstPos), secondPos);

        return rampPos;
    }
}
