package org.firstinspires.ftc.teamcode.unused;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Disabled
public class PIDFTunerFlywheel extends OpMode {
    public DcMotorEx flywheel1;
    public DcMotorEx flywheel2;


    public double highVelocity = 1500; // Example high velocity
    public double lowVelocity = 900;  // Example low velocity
    double curTargetVelocity = highVelocity;

    double F = 0.0;
    double P = 0.0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01};
    int stepIndex = 1; // Start with medium step size


    @Override
    public void init() {
        flywheel1 = hardwareMap.get(DcMotorEx.class, "fOne");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "fTwo");
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);

        flywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0.0, 0.0, F);
        flywheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addData("init", "complete");
    }

    @Override
    public void loop() {
        //gamepad contols
        // set Target velocity
        //updateTelemetry
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        //set new PIDF
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0.0, 0.0, F);
        flywheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheel1.setVelocity(curTargetVelocity);
        flywheel2.setVelocity(curTargetVelocity);

        double curlVelocity1 = flywheel1.getVelocity();
        double curlVelocity2 = flywheel2.getVelocity();
        double error1 = curTargetVelocity - curlVelocity1;
        double error2 = curTargetVelocity - curlVelocity2;


            telemetry.addData("Target Velocity", curTargetVelocity);
            telemetry.addData("Flywheel1 Velocity", "%.2f", curlVelocity1);
            telemetry.addData("Flywheel2 Velocity", "%.2f", curlVelocity2);
            telemetry.addData("Error1", "%.2f", error1);
            telemetry.addData("Error2", "%.2f", error2);
            telemetry.addLine("_______________________________");
            telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
            telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
            telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
            telemetry.update();



    }
}
