package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Intake {

    private DcMotor intake;
    private Servo flapL, flapR, pto;

    public static double shotDelay = 1.2, shotTransferAllowance = 0.4;
    public static double flapActuationTime = 0.8;

    private boolean intakeActivityFlag = false;
    private boolean shootingFlag = false;

    public static double flapUp = 0.81, flapDown = 0.89, ptoEngaged = 0.36, ptoDisengaged = 0.265;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.dcMotor.get("intake");
        flapL = hardwareMap.servo.get("lFlap");
        flapR = hardwareMap.servo.get("rFlap");
        pto = hardwareMap.servo.get("pto");
        flapR.setDirection(Servo.Direction.REVERSE);
    }

    public void setActive(boolean flag) {
        intakeActivityFlag = flag;
    }

    public void setReverse(boolean flag) {
        shootingFlag = flag;
    }

    ElapsedTime shotTimer = new ElapsedTime();
    private boolean wasShooting = false;

    int shotCount = 0;

    public void update() {
        if (intakeActivityFlag) {
            intake.setPower(1);
        } else if (!shootingFlag && !intakeActivityFlag) {
            intake.setPower(0);
        } else if (shootingFlag && !intakeActivityFlag){
            intake.setPower(-1);
        }
    }

    public void setIntake(double power) {
        intake.setPower(power);
    }

    public void setFlap(double pos) {
        flapR.setPosition(pos);
        flapL.setPosition(pos);
    }

    public void setPtoEngaged(boolean engaged) {
        if (engaged) {
            pto.setPosition(ptoEngaged);
        } else {
            pto.setPosition(ptoDisengaged);
        }
    }

}
