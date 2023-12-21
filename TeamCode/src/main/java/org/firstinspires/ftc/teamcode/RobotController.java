package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="RobotController")
public class RobotController extends LinearOpMode{
    double GRIPPER_LEFT_OPEN_POSITION = 0.5;
    double GRIPPER_LEFT_CLOSE_POSITION = 0;
    double GRIPPER_RIGHT_OPEN_POSITION = 0.5;
    double GRIPPER_RIGHT_CLOSE_POSITION = 0;
    double DRONE_START_POSITION = 0.696;
    double DRONE_RELEASE_POSITION = 0.248;

    // Variables used for auto turn
    double turnError = 0;
    double turnSpeed = 0;
    double turnSetpoint = 0;
    double turnFeedback = 0;
    double turnOffset = 0;

    DcMotor frontLeftMotor;
    DcMotor rearLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearRightMotor;
    DcMotor slideMotor1;
    DcMotor slideMotor2;
    Servo arm;
    Servo leftGripper;
    Servo rightGripper;
    Servo droneLauncher;
    DistanceSensor leftSensor;
    DistanceSensor rightSensor;

    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException
    {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        rearLeftMotor = hardwareMap.dcMotor.get("rearLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        rearRightMotor = hardwareMap.dcMotor.get("rearRightMotor");

        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotor1 = hardwareMap.dcMotor.get("leftSlideMotor");
        slideMotor2 = hardwareMap.dcMotor.get("rightSlideMotor");

        slideMotor2.setDirection(DcMotor.Direction.REVERSE);

        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hardwareMap.servo.get("armServo");
        leftGripper = hardwareMap.servo.get("leftGripperServo");
        rightGripper = hardwareMap.servo.get("rightGripperServo");
        droneLauncher = hardwareMap.servo.get("droneLaunchServo");

        leftSensor = hardwareMap.get(DistanceSensor.class, "checkLeft");
        rightSensor = hardwareMap.get(DistanceSensor.class, "checkRight");

        ConfigureGyro();

        droneLauncher.setPosition(DRONE_START_POSITION);
        leftGripper.setPosition(GRIPPER_LEFT_OPEN_POSITION);
        rightGripper.setPosition(GRIPPER_RIGHT_OPEN_POSITION);

        waitForStart();

        if (isStopRequested()) return;

        double slowingDown = 4;

        while (opModeIsActive()) {

            if ((gamepad2.dpad_up)){
                droneLauncher.setPosition(DRONE_RELEASE_POSITION);
            }

            if(gamepad1.b){
                slowingDown = 4;
            }
            else{
                slowingDown = 1.5;
            }

            double y = (-gamepad1.left_stick_y / slowingDown) + (gamepad1.right_stick_y / -2);
            double x = (gamepad1.left_stick_x / slowingDown) + (gamepad1.right_stick_x / 2);
            turnOffset = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.5;

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            AttachmentController();

            double botHeading;

            botHeading = 0;

            if (gamepad1.back){
                imu.resetYaw();
                turnSetpoint = 0;
            }
            else if((botHeading == 14000000)){
                telemetry.addLine("Gyro broken, please reset robot");
            }
            else{

                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turnOffset), 1);
                rotX = rotX * 1.1;



                double frontLeftPower = (rotY + rotX + 0) / denominator;
                double rearLeftPower = (rotY - rotX + 0) / denominator;
                double frontRightPower = (rotY - rotX - 0) / denominator;
                double rearRightPower = (rotY + rotX - 0) / denominator;



                turnFeedback = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                turnFeedback = 0;

                Direction();

                telemetry.addData("Direction", turnSetpoint);

                frontLeftPower -= turnOffset;
                frontRightPower += turnOffset;
                rearLeftPower -= turnOffset;
                rearRightPower += turnOffset;

                if((Math.abs(frontLeftPower) < 0.05) & (Math.abs(rearLeftPower) < 0.05) & (Math.abs(frontRightPower) < 0.05) & (Math.abs(rearRightPower) < 0.05))
                {
                    frontLeftMotor.setPower(0);
                    rearLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    rearRightMotor.setPower(0);
                }
                else
                {
                    frontLeftMotor.setPower(frontLeftPower);
                    rearLeftMotor.setPower(rearLeftPower);
                    frontRightMotor.setPower(frontRightPower);
                    rearRightMotor.setPower(rearRightPower);
                }

                telemetry.addData("Robot current yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

                telemetry.addData("Left Sensor", leftSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("Right Sensor", rightSensor.getDistance(DistanceUnit.CM));

                telemetry.addData("Error", turnError);
                telemetry.addData("Rotation Speed", turnSpeed);
                telemetry.update();
            }
        }
    }
    int slidePos = 0;
    boolean armPos = false;
    public void AttachmentController()
    {
        double slidePosError, slidePower;
        double slidePosError2, slidePower2;

        if(gamepad2.a)
        {
            slidePos = 950;
            armPos = true;
        } else if (gamepad2.b)
        {
            slidePos = 1800;
            armPos = true;
        } else if (gamepad2.x)
        {
            slidePos = 2550;
            armPos = true;
        } else if (gamepad2.y)
        {
            slidePos = 3050;
            armPos = true;
        }

        if(gamepad2.dpad_down)
        {
            slidePos = 0;
            armPos = false;
        }

        slidePos += -gamepad2.left_stick_y * 45;

        slidePos = Math.min(Math.max(slidePos, 0), 3070);

        if(slideMotor1.getCurrentPosition() < 10 && slidePos < 5)
        {
            slideMotor1.setPower(0);
            slideMotor2.setPower(0);
        }
        else{
            if(gamepad2.back){
                slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if(gamepad2.start){
                slidePos = 1000;

                slidePosError2 = slidePos - slideMotor1.getCurrentPosition();
                slidePower2 = slidePosError2;
                slidePower2 = Math.min(Math.max(slidePower2, -0.75), 0.75);

                slideMotor1.setPower(slidePower2);
                slideMotor2.setPower(slidePower2);

            }else {
                slidePosError = slidePos - slideMotor1.getCurrentPosition();
                slidePower = slidePosError * 3 / 500;
                slidePower = Math.min(Math.max(slidePower, -0.6), 0.75);

                slideMotor1.setPower(slidePower);
                slideMotor2.setPower(slidePower);
                telemetry.addData("slidePosError", slidePosError);
            }
        }
        telemetry.addData("slideMotor1 Power", slideMotor1.getPower());
        telemetry.addData("slidePosition", slidePos);

        if(gamepad2.left_bumper){
            leftGripper.setPosition(GRIPPER_LEFT_CLOSE_POSITION);
        }

        if(gamepad2.right_bumper){
            rightGripper.setPosition(GRIPPER_LEFT_OPEN_POSITION);
        }

        if(gamepad2.right_trigger > 0){
            rightGripper.setPosition(GRIPPER_RIGHT_CLOSE_POSITION);
        }

        if(gamepad2.left_trigger > 0){
            leftGripper.setPosition(GRIPPER_RIGHT_OPEN_POSITION);
        }

        arm.setPosition(Ramp(0.501, 0.744, armPos));

        telemetry.addData("Arm pos", armPos);
    }

    double rampPos = 0.501;
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

    public void ConfigureGyro()
    {
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);
    }

    public void GyroTurn()
    {
        if(turnSetpoint > 180){
            turnSetpoint = turnSetpoint - 360;
        }
        else if(turnSetpoint < -180){
            turnSetpoint = turnSetpoint + 360;
        }
        turnError = turnSetpoint - turnFeedback;
        if(turnError > 180){
            turnError = turnError - 360;
        }
        else if(turnError < -180){
            turnError = turnError + 360;
        }

        // Rotation sensitivity
        turnSpeed = 0.04 * turnError;
        // Keeps turning speed in a range
        if(turnSpeed > 0.4){
            turnSpeed = 0.4;
        }

        else if(turnSpeed < -0.4){
            turnSpeed = -0.4;
        }
    }

    public void Direction()
    {
        if(gamepad1.dpad_left){
            turnSetpoint = 90;
        }
        else if(gamepad1.dpad_up){
            turnSetpoint = 0;
        }
        else if(gamepad1.dpad_right){
            turnSetpoint = -90;
        }
        else if(gamepad1.dpad_down){
            turnSetpoint = 180;
        }
        else{
            turnSetpoint += turnOffset;
        }
    }
}
