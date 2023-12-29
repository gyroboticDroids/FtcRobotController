package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="TestAutonomous")
public class TestAutonomous extends LinearOpMode {

    DistanceSensor leftSensor;
    DistanceSensor rightSensor;
    Servo leftGripper;
    Servo rightGripper;
    DcMotor slideMotor1;
    DcMotor slideMotor2;
    Pose2d selectPose = new Pose2d(0,0,0);
    double selectTurn = 0;
    double currentHeading = 0;
    double boardOffset = 0;
    boolean armUp = false;
    int slidePos;
    double rampPos = 0.501;
    Servo arm;
    double slidePosError;
    double slidePower;


    @Override
    public void runOpMode() throws InterruptedException
    {
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
        Pose2d startPose = new Pose2d(-38.25, -63.75, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        TrajectorySequence approachSpikeMarks = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-35, -30.25, startPose.getHeading()), startPose.getHeading())
                .addDisplacementMarker(() -> {
                    if (leftSensor.getDistance(DistanceUnit.INCH) < 5) {
                        selectPose = new Pose2d(-39, -31.25, Math.toRadians(180));
                        selectTurn = 90;
                        boardOffset = 6.25;
                    } else if (rightSensor.getDistance(DistanceUnit.INCH) < 5) {
                        selectPose = new Pose2d(-32, -31.25,0);
                        selectTurn = -90;
                        boardOffset = -6.25;
                    } else {
                        selectPose = new Pose2d(-35.5, -31.5, Math.toRadians(90));
                        selectTurn = 0;
                        boardOffset = 0;
                    }
                })
                .build();
        drive.followTrajectorySequence(approachSpikeMarks);

        telemetry.addData("Board Offset", boardOffset);
        telemetry.update();

        TrajectorySequence placePurplePixel = drive.trajectorySequenceBuilder(approachSpikeMarks.end())
                .turn(Math.toRadians(selectTurn))
                .lineToLinearHeading(selectPose)
                .addDisplacementMarker(() -> {
                    leftGripper.setPosition(Constants.GRIPPER_LEFT_OPEN_POSITION);
                })
                .back(4.5)
                .lineToLinearHeading(new Pose2d(-35.5, -48, selectPose.getHeading()))
                .turn(Math.toRadians(179.999)-selectPose.getHeading())
                .lineToLinearHeading(new Pose2d(-58,-48, Math.toRadians(180)))
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(-58,-9, Math.toRadians(90)))
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(-9,-9,0))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(35,-9,Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(43, -30 + boardOffset))
                .build();

        drive.followTrajectorySequence(placePurplePixel);

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

        TrajectorySequence dropOnBoard = drive.trajectorySequenceBuilder(placePurplePixel.end())
                .forward(6.5)
                .build();
        drive.followTrajectorySequence(dropOnBoard);

        rightGripper.setPosition(Constants.GRIPPER_RIGHT_OPEN_POSITION);

        TrajectorySequence backUp = drive.trajectorySequenceBuilder(dropOnBoard.end())
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
