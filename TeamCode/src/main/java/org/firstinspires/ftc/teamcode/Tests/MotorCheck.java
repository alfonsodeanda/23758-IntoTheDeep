package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "Read motors value")
public class MotorCheck extends LinearOpMode {
    private DcMotor delanteIz, delanteDe, atrasIz, atrasDe;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addLine("Prueba motores");
        telemetry.update();

        delanteIz = hardwareMap.get(DcMotor.class, "delanteIz");
        delanteDe = hardwareMap.get(DcMotor.class, "delanteDe");
        atrasIz  = hardwareMap.get(DcMotor.class, "atrasIz");
        atrasDe = hardwareMap.get(DcMotor.class, "atrasDe");

        delanteIz.setDirection(DcMotor.Direction.REVERSE);
        atrasIz.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();

        delanteIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delanteDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        atrasIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        atrasDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()){
            motorsSetPower(0.5, 0.5, 0.5, 0.5);
            telemetry.addData("Encoders", "delanteDe: %d , delanteIz: %d , atrasDe: %d , atrasIz: %d",
                    delanteDe.getCurrentPosition(), delanteIz.getCurrentPosition(),
                    atrasDe.getCurrentPosition(), atrasIz.getCurrentPosition());
            telemetry.update();
        }
    }

    public void resetEncoders() {
        delanteIz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        delanteDe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        atrasIz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        atrasDe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        delanteIz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        delanteDe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        atrasIz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        atrasDe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void motorsSetPower (double powDeIz, double powDeDe, double powAtIz, double powAtDe) {
        delanteIz.setPower(powDeIz);
        delanteDe.setPower(powDeDe);
        atrasIz.setPower(powAtIz);
        atrasDe.setPower(powAtDe);
    }

}
