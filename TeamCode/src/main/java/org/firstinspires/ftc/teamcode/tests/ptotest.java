package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.systems.Intake;

@Config
@TeleOp
public class ptotest extends OpMode {
    public static double ip = 0;
    private Servo pto;

    private DcMotor intake;

    public static boolean ptoEngaged = false;


    @Override
    public void init() {
        pto = hardwareMap.servo.get("pto");

        intake = hardwareMap.dcMotor.get("intake");
    }

    @Override
    public void loop() {

        if (ptoEngaged) {
            pto.setPosition(Intake.ptoEngaged);
        } else {
            pto.setPosition(Intake.ptoDisengaged);
        }

        intake.setPower(ip);
    }
}