/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Drive" )
@Configurable

public class Drive extends LinearOpMode {
    // Config variables
    public static int p = 200;
    public static int i = 0;
    public static int d = 15; // 15
    public static int f = 0;
    public static int vel = 1350; // 2100 for far; 1350 for near
    public static double intakeIntakePos = 0.35;
    public static double intakePower = 1;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private long interval = 500;
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx flywheel1 = null;
    private DcMotorEx flywheel2 = null;
    private DcMotorEx intake = null;
    private Servo intakeServol = null;
    private Servo intakeServoR = null;
    private Servo spin = null;
    private CRServo spinner = null;
    private Limelight3A limelight;
    private double distance = 60; // default value if limelight is completely busted
    private ElapsedTime limelightTimer = new ElapsedTime();
    private GoBildaPrismDriver ledStrip;
    PrismAnimations.Solid blueSolid = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.Solid magentaSolid = new PrismAnimations.Solid(Color.MAGENTA);
    private PrismAnimations.Solid ledColor = blueSolid;



    // Carousel Initialization
    private int carouselState = 0;

    private void spinSM() {
        if (carouselState > 2) {
            carouselState = 0;
        } else if (carouselState < 0) {
            carouselState = 2;
        }
        switch (carouselState) {
            case 0:
                spin.setPosition(0.026);
                break;
            case 1:
                spin.setPosition(0.3975);
                break;
            case 2:
                spin.setPosition(0.764);
                break;
        }
    }

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "lf");
        backLeftDrive = hardwareMap.get(DcMotor.class, "lr");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        backRightDrive = hardwareMap.get(DcMotor.class, "rr");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "fOne");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "fTwo");
        intake = hardwareMap.get(DcMotorEx.class, "in");
        intakeServol = hardwareMap.get(Servo.class, "inl");
        intakeServoR = hardwareMap.get(Servo.class, "inr");
        spin = hardwareMap.get(Servo.class, "spin");
        spinner = hardwareMap.get(CRServo.class, "spinner");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        ledStrip = hardwareMap.get(GoBildaPrismDriver.class, "led_strip");

        double P = 20.0;
        double F = 15.0;

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        flywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel1.setDirection(DcMotor.Direction.REVERSE);

        intakeServol.setPosition(0.5);
        intakeServoR.setPosition(0.5);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");

        // Drive Mode Selection
        telemetry.addLine("\nPlease select Drive mode:");
        telemetry.addLine("  |> LEFT BUMPER -> Singleplayer");
        telemetry.addLine("  |> RIGHT BUMPER -> Multiplayer (Default)");
        telemetry.update();
        boolean isSP;
        while (true) {
            if (gamepad1.leftBumperWasPressed()) {
                telemetry.addLine("Singleplayer selected!");
                isSP = true;
                break;
            } else if (gamepad1.rightBumperWasPressed()) {
                telemetry.addLine("Multiplayer selected!");
                isSP = false;
                break;
            }
            if (isStarted()) {
                telemetry.addLine("No Drive mode selected!");
                telemetry.addLine("Defaulting to Multiplayer mode...");
                isSP = false;
                break;
            }
        }
        telemetry.addLine("Ready to start!1!!11");
        telemetry.update();

        waitForStart();
        runtime.reset();
        limelightTimer.reset();

        float pos = 0.5F;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //-------------- Flywheel  --------------
            if ((gamepad1.right_bumper && isSP) || (gamepad2.right_bumper && !isSP)) {
                boolean isVisible = updateDistance();
                telemetry.addData("Distance", distance);
                if (!isVisible) {
                    if (ledColor == magentaSolid) {
                        ledStrip.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, blueSolid);
                        ledColor = blueSolid;
                    }
//                    vel = 1300;
                } else {
                    if (ledColor == blueSolid) {
                        ledStrip.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, magentaSolid);
                        ledColor = magentaSolid;
                    }
                }
                vel = (int) (7.51591 * distance + 753.21775);

                flywheel1.setVelocityPIDFCoefficients(p, i, d, f);
                flywheel2.setVelocityPIDFCoefficients(p, i, d, f);

                flywheel1.setVelocity(vel);
                flywheel2.setVelocity(vel);
            } else {
                flywheel1.setVelocity(0);
                flywheel2.setVelocity(0);
            }
            //-------------- Intake Controls --------------
            if ((gamepad1.left_bumper && isSP) || (gamepad2.left_bumper && !isSP)) {
                intake.setPower(intakePower);
            } else if ((gamepad1.left_trigger > 0.2 && isSP) || gamepad2.left_trigger > 0.2 && !isSP) {
                intake.setPower(-intakePower);
            } else {
                intake.setPower(0);
            }

            //-------------- Intake Servo Controls --------------
            // Intaking
            if ((gamepad1.dpadLeftWasPressed() && isSP) || (gamepad2.dpadLeftWasPressed() && !isSP)) {
                intakeServol.setPosition(intakeIntakePos);
                intakeServoR.setPosition(intakeIntakePos);
            }

            // Shooting
            if ((gamepad1.dpadRightWasPressed() && isSP) || (gamepad2.dpadRightWasPressed() && !isSP)) {
                intakeServol.setPosition(0.6);
                intakeServoR.setPosition(0.6);
            }

            // Neutral
            if ((gamepad1.dpadDownWasPressed() && isSP) || (gamepad2.dpadDownWasPressed() && !isSP)) {
                intakeServol.setPosition(0.5);
                intakeServoR.setPosition(0.5);
            }


            //-------------- Spinner Controls --------------
            if (((gamepad1.xWasPressed() && isSP) || (gamepad2.xWasPressed() && !isSP))) {
                carouselState--;
                spinSM();
            }
            if ((gamepad1.bWasPressed() && isSP) || (gamepad2.bWasPressed() && !isSP)) {
                carouselState++;
                spinSM();
            }


            //-------------- Drive Controls --------------
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.update();
        }
        limelight.stop();
    }

    private boolean updateDistance() {
        limelightTimer.reset();
        LLStatus status = limelight.getStatus();
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            LLResultTypes.FiducialResult target = result.getFiducialResults().get(0);

            if (target != null) {
                double x = (target.getCameraPoseTargetSpace().getPosition().x / DistanceUnit.mPerInch) + 8;
                double z = (target.getCameraPoseTargetSpace().getPosition().z / DistanceUnit.mPerInch) + 8;
               // double x = target.getCameraPoseTargetSpace().getPosition().x;
                //double z = target.getCameraPoseTargetSpace().getPosition().z;
                double dist = Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2));
                telemetry.addData("x, z, distance", x + " " + z + " " + dist);

                distance = dist;
                return true;
            }
        }

        telemetry.addData("Limelight", "No data available");
        return false;
    }
}
