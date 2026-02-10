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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

@TeleOp(name="Drive" )
@Configurable

public class Drive extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "lf");
        backLeftDrive = hardwareMap.get(DcMotor.class, "lr");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        backRightDrive = hardwareMap.get(DcMotor.class, "rr");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        flywheel1 = hardwareMap.get(DcMotorEx.class, "fOne");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "fTwo");
        flywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel1.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "in");
        intakeServoL = hardwareMap.get(Servo.class, "inl");
        intakeServoR = hardwareMap.get(Servo.class, "inr");
        intakeServoL.setPosition(0.5);
        intakeServoR.setPosition(0.5);

        spin = hardwareMap.get(Servo.class, "spin");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");;
        limelight.pipelineSwitch(0);
        limelight.start();

        ledStrip = hardwareMap.get(GoBildaPrismDriver.class, "led_strip");

        telemetry.addData("Status", "Initialized");

        // Drive Mode Selection
        selectDriveMode();

        waitForStart();
        runtime.reset();
        limelightTimer.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            limelightFlywheelPIDF();

            intakeControls();

            intakePositionControls();

            spinControls();

            driveControls();
            telemetry.update();
        }
        limelight.stop();
    }

    private void intakeControls() {
        if ((gamepad1.left_bumper && isSP) || (gamepad2.left_bumper && !isSP)) {
            intake.setPower(intakePower);
        } else if ((gamepad1.left_trigger > 0.2 && isSP) || gamepad2.left_trigger > 0.2 && !isSP) {
            intake.setPower(-intakePower);
        } else {
            intake.setPower(0);
        }
    }
    private void spinControls() {
        if (((gamepad1.xWasPressed() && isSP) || (gamepad2.xWasPressed() && !isSP))) {
            spinState--;
            spinSM();
        }
        if ((gamepad1.bWasPressed() && isSP) || (gamepad2.bWasPressed() && !isSP)) {
            spinState++;
            spinSM();
        }
    }
    private void spinSM() {
        if (spinState > 2) {
            spinState = 0;
        } else if (spinState < 0) {
            spinState = 2;
        }
        switch (spinState) {
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
    private void intakePositionControls() {
        // Intaking
        if ((gamepad1.dpadLeftWasPressed() && isSP) || (gamepad2.dpadLeftWasPressed() && !isSP)) {
            intakeServoL.setPosition(intakeIntakePos);
            intakeServoR.setPosition(intakeIntakePos);
        }

        // Shooting
        if ((gamepad1.dpadRightWasPressed() && isSP) || (gamepad2.dpadRightWasPressed() && !isSP)) {
            intakeServoL.setPosition(intakeShootPos);
            intakeServoR.setPosition(intakeShootPos);
        }

        // Neutral
        if ((gamepad1.dpadDownWasPressed() && isSP) || (gamepad2.dpadDownWasPressed() && !isSP)) {
            intakeServoL.setPosition(0.5);
            intakeServoR.setPosition(0.5);
        }
    }
    private void limelightFlywheelPIDF() {
        if ((gamepad1.right_bumper && isSP) || (gamepad2.right_bumper && !isSP)) {
            boolean isVisible = updateDistance();
            telemetry.addData("Distance", distance);
            if (!isVisible) {
                if (ledColor == magentaSolid) {
                   ledStrip.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, blueSolid);
                    ledColor = blueSolid;
                }
            } else {
                if (ledColor == blueSolid) {
                   ledStrip.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, magentaSolid);
                    ledColor = magentaSolid;
                }
            }
            vel = (int) (7.18563 * distance + 810.62874);

            flywheel1.setVelocityPIDFCoefficients(p, i, d, f);
            flywheel2.setVelocityPIDFCoefficients(p, i, d, f);

            flywheel1.setVelocity(vel);
            flywheel2.setVelocity(vel);
        } else {
            flywheel1.setVelocity(0);
            flywheel2.setVelocity(0);
        }
    }
    private void selectDriveMode() {
        telemetry.addLine("\nPlease select Drive mode:");
        telemetry.addLine("  |> LEFT BUMPER -> Singleplayer");
        telemetry.addLine("  |> RIGHT BUMPER -> Multiplayer (Default)");
        telemetry.update();
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
                // No Drive mode selected
                // Defaulting to Multiplayer mode
                break;
            }
        }
        telemetry.addLine("Ready to start!1!!11");
        telemetry.update();
    }
    private void driveControls() {
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
                double dist = Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2));
                telemetry.addData("x, z, distance", x + " " + z + " " + dist);

                distance = dist;
                return true;
            }
        }

        telemetry.addData("Limelight", "No data available");
        return false;
    }

    // -------- Variable Diarrhea ----------
    // Config variables
    public static int p = 200;
    public static int i = 0;
    public static int d = 15; // 15
    public static int f = 0;
    public static int vel = 1350; // 2100 for far; 1350 for near
    public static float intakeIntakePos = 0.35F;
    public static float intakeShootPos = 0.576F;
    public static double intakePower = 1;

    // Declare OpMode Members
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;

    private DcMotorEx intake;
    private Servo intakeServoL;
    private Servo intakeServoR;

    private Servo spin;
    private int spinState = 0;

    private Limelight3A limelight;
    private final ElapsedTime limelightTimer = new ElapsedTime();
    private double distance = 60; // default value if limelight is completely busted

    private GoBildaPrismDriver ledStrip;
    PrismAnimations.Solid blueSolid = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.Solid magentaSolid = new PrismAnimations.Solid(Color.MAGENTA);
    private PrismAnimations.Solid ledColor = blueSolid;

    boolean isSP = false;
}
