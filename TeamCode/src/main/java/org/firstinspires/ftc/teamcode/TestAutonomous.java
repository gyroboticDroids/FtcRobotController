package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="TestAutonomous")
public class TestAutonomous extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor rearLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearRightMotor;
    @Override
    public void runOpMode() throws InterruptedException
    {
//        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
//        rearLeftMotor = hardwareMap.dcMotor.get("rearLeftMotor");
//        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
//        rearRightMotor = hardwareMap.dcMotor.get("rearRightMotor");
//
//        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));

        waitForStart();

        TrajectorySequence approachSpikeMarks = drive.trajectorySequenceBuilder(startPose)
                .forward(32.5)
                .build();

        drive.followTrajectorySequence(approachSpikeMarks);

      //  if(){


     //   }else if(){


      //  }else (){


    //    }


        TrajectorySequence driveToBoard = drive.trajectorySequenceBuilder(startPose)
                .forward(32.5)
                .waitSeconds(1)
                .back(28)
                .strafeRight(23)
                .forward(48)
                .turn(Math.toRadians(-90))
                .waitSeconds(1)
                .build();


        sleep(5000);
    }
}
