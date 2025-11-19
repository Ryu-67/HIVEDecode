package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.tests.ShotAlgTest.c;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.systems.Drivebase;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Localizer;
import org.firstinspires.ftc.teamcode.systems.Shooter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RedTeleop")
@Config
public class RedTele extends OpMode {

    private Drivebase drivebase; private Intake intake; private Shooter shooter;

    GamepadEx controller;

    Localizer localizer;

    private boolean isDpadDownPressed = false, isDpadRightPressed = false, isLBPressed = false, isDpadUpPressed = false, shootingMode = false, close = true, flapUp = false;

    Limelight3A ll3a;
    private AnalogInput encoder;
    public static double p = 0.025, i, d;
    private PIDController turretPID;
    private DcMotor turret;

    double limelightMountAngleDegrees = 10, limelightLensHeightInches = 13.4, goalHeightInches = 29.5, lastTurretTarget = 0;

    @Override
    public void init() {
        drivebase = new Drivebase(hardwareMap, true);
        intake = new Intake(hardwareMap);

        controller = new GamepadEx(gamepad1);

        turret = hardwareMap.dcMotor.get("turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        localizer = new Localizer(hardwareMap, false);
        localizer.setPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        ll3a = hardwareMap.get(Limelight3A.class, "ll3a");
        ll3a.setPollRateHz(250);
        ll3a.start();

        turretPID = new PIDController(p, i, d);
        encoder = hardwareMap.analogInput.get("encoder");

        shooter = new Shooter(hardwareMap, telemetry);

        intake.setPtoEngaged(false);
    }

    ElapsedTime surveyTimer = new ElapsedTime();

    @Override
    public void loop() {
        localizer.update();
        double heading = localizer.getTruePose().getHeading(AngleUnit.RADIANS);
        drivebase.takeTeleInput(controller.getLeftY(), controller.getLeftX(), controller.getRightX());

        if (gamepad1.right_bumper && !isDpadDownPressed) {
            shootingMode = !shootingMode;
            surveyTimer.reset();
        } isDpadDownPressed = gamepad1.right_bumper;

        if (gamepad1.dpad_right && !isDpadRightPressed) {
            close = !close;
        } isDpadRightPressed = gamepad1.dpad_right;

        if (gamepad1.left_bumper) {
            intake.setFlap(Intake.flapUp);
            intake.setPtoEngaged(true);
            intake.setActive(true);
        } else {
            intake.setFlap(Intake.flapDown);
            intake.setPtoEngaged(false);
            intake.setActive(controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
            intake.setReverse(controller.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        }

        if (shootingMode) {
            LLResult result = ll3a.getLatestResult();
            boolean detected = result.getBotposeTagCount() > 0;
            boolean locked = false;
            double tX = 0, tY = 0;

            if (detected) {
                for (LLResultTypes.FiducialResult result1 : result.getFiducialResults()) {
                    if (result1.getFiducialId() == 24) {
                        locked = true;
                        tX = -result1.getTargetXDegrees();
                        tY = result1.getTargetYDegrees();
                        break;
                    } else {
                        locked = false;
                    }
                }
            }

            telemetry.addData("locked?", locked);

            if (locked) {
                double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + tY);
                double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
                turret.setPower(p*(0-tX));

                double meters = distanceFromLimelightToGoalInches * 0.0254;
                double turretDeg = analogVoltageToDegrees(encoder.getVoltage());

                shooter.setTargetRPM(getRPMForShot(meters) + c);
                shooter.setHoodAngle(getHoodAngle(meters));
                lastTurretTarget = turretDeg;
            } else {
                double turretDeg = analogVoltageToDegrees(encoder.getVoltage());
                double error;
                if (close) {

                    error = AngleUnit.normalizeDegrees(45 - turretDeg);

                } else {

                    error = AngleUnit.normalizeDegrees((Math.toDegrees(heading) - 70) - turretDeg);

                }
                turret.setPower(p * error);
                lastTurretTarget = turretDeg;
            }
            shooter.runShooter();
        } else {
            double turretDeg = analogVoltageToDegrees(encoder.getVoltage());
            double error = AngleUnit.normalizeDegrees(lastTurretTarget - turretDeg);
            turret.setPower(p * error);
            shooter.stopShooter();
        }

        telemetry.addData("is shootingModeOn", shootingMode);
        telemetry.addData("is close", close);

        intake.update();
        drivebase.update();
    }

    private double analogVoltageToDegrees(double voltage) {
        return voltage * (360/3.3);
    }

    public double getRPMForShot(double meters) {
//        return (211.43 * meters) + 1177;
        return (227.87*meters) + 1382.7;
    }

    public double getHoodAngle(double meters) {
//        return (-8.8 * meters) + 76.16;
        return (-4.8701*meters) + 59.754;
    }
}
