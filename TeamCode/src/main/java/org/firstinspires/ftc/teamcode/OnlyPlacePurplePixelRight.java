package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Red Left Auto Purple Only", group = "Centerstage Autonomous", preselectTeleOp = "RobotController")
public class OnlyPlacePurplePixelRight extends LinearOpMode {
    //Sets up motors and variables
    DistanceSensor leftSensor;
    DistanceSensor rightSensor;
    Servo leftGripper;
    Servo rightGripper;
    DcMotor slideMotor1;
    DcMotor slideMotor2;
    Pose2d selectPose = new Pose2d(0,0,0);
    double selectTurn = 0;
    double boardOffset = 0;
    boolean armUp = false;
    int slidePos;
    double rampPos = 0.501;
    Servo arm;
    double slidePosError;
    double slidePower;
    double waitTime;
    Servo droneLauncher;
    @Override
    public void runOpMode() throws InterruptedException {
        //Sets up sensors
        leftSensor = hardwareMap.get(DistanceSensor.class, "checkLeft");
        rightSensor = hardwareMap.get(DistanceSensor.class, "checkRight");
        //Sets up servos
        leftGripper = hardwareMap.servo.get("leftGripperServo");
        rightGripper = hardwareMap.servo.get("rightGripperServo");
        leftGripper.setPosition(Constants.GRIPPER_LEFT_CLOSE_POSITION);
        rightGripper.setPosition(Constants.GRIPPER_RIGHT_CLOSE_POSITION);
        arm = hardwareMap.servo.get("armServo");
        //Sets up slide motors
        slideMotor1 = hardwareMap.dcMotor.get("leftSlideMotor");
        slideMotor2 = hardwareMap.dcMotor.get("rightSlideMotor");
        slideMotor2.setDirection(DcMotor.Direction.REVERSE);
        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        droneLauncher = hardwareMap.servo.get("droneLaunchServo");
        droneLauncher.setPosition(Constants.DRONE_START_POSITION);
        //Sets up dead wheels
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(14.5, -63.75, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        Trajectory approachSpikeMarks = drive.trajectoryBuilder(startPose)
            //Move into spike mark area
            .splineToLinearHeading(new Pose2d(11.75, -30.25, startPose.getHeading()), startPose.getHeading())
            .build();
        drive.followTrajectory(approachSpikeMarks);

        Trajectory placePurplePixel;

        if (leftSensor.getDistance(DistanceUnit.INCH) < 5) {
            selectPose = new Pose2d(7.75, -31.25, Math.toRadians(180));
            boardOffset = 4.25;
            placePurplePixel = drive.trajectoryBuilder(approachSpikeMarks.end())
                //Drive up to proper spike mark
                .lineToLinearHeading(selectPose)
                //Drop purple pixel
                .addDisplacementMarker(() -> {
                   leftGripper.setPosition(Constants.GRIPPER_LEFT_OPEN_POSITION);
                })
                .back(4.5)
                .lineToLinearHeading(new Pose2d(39, -33.25 + boardOffset, 0))
                .build();
        } else if (rightSensor.getDistance(DistanceUnit.INCH) < 5) {
            selectPose = new Pose2d(15.75, -31.25, 0);
            boardOffset = -4.25;
            placePurplePixel = drive.trajectoryBuilder(approachSpikeMarks.end())
                //Drive up to proper spike mark
                .lineToLinearHeading(selectPose)
                //Drop purple pixel
                .addDisplacementMarker(() -> {
                    leftGripper.setPosition(Constants.GRIPPER_LEFT_OPEN_POSITION);
                })
                .back(4.5)
                .strafeRight(16.75)
                .splineToLinearHeading(new Pose2d(39, -33.25 + boardOffset, 0), Math.toRadians(90))
                .build();

        } else {
            selectPose = new Pose2d(11.75, -31.5, Math.toRadians(90));
            boardOffset = -1.5;
            placePurplePixel = drive.trajectoryBuilder(approachSpikeMarks.end())
                //Drive up to proper spike mark
                .lineToLinearHeading(selectPose)
                //Drop purple pixel
                .addDisplacementMarker(() -> {
                    leftGripper.setPosition(Constants.GRIPPER_LEFT_OPEN_POSITION);
                })
                .back(4.5)
                .lineToLinearHeading(new Pose2d(39, -33.25 + boardOffset, 0))
                .build();
        }
        drive.followTrajectory(placePurplePixel);
    }
}
