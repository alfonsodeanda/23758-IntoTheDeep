package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Chasis Test", group = "Linear OpMode")
public class Plesiov1 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor delanteIz, delanteDe, atrasIz, atrasDe;
    private DcMotor Intake, elevador;
    private CRServo garra;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        delanteIz = hardwareMap.get(DcMotor.class, "delanteIz");
        delanteDe = hardwareMap.get(DcMotor.class, "delanteDe");
        atrasIz = hardwareMap.get(DcMotor.class, "atrasIz");
        atrasDe = hardwareMap.get(DcMotor.class, "atrasDe");
        elevador = hardwareMap.get(DcMotor.class, "elevador");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        garra = hardwareMap.get(CRServo.class, "garra");

        delanteIz.setDirection(DcMotor.Direction.REVERSE);
        delanteDe.setDirection(DcMotor.Direction.FORWARD);
        atrasIz.setDirection(DcMotor.Direction.REVERSE);
        atrasDe.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            chasis(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, 0.9);

            Elevador(gamepad2.dpad_up, gamepad2.dpad_down);

            Intake(gamepad2.x, gamepad2.b);

            telemetry.addData("Runtime", runtime.seconds());
            telemetry.addData("Garra Abierta", garraAbierta);
            telemetry.update();
        }
    }

    public void chasis (double x, double y, double rx, double maxPow) {
        double powDelanteIzquierda = (y + x + rx);
        double powAtrasIzquierda = (y - x + rx);
        double powDelanteDerecha = (y - x - rx);
        double powAtrasDerecha = (y + x - rx);

        powDelanteIzquierda = Math.max(-maxPow, Math.min(powDelanteIzquierda, maxPow));
        powAtrasIzquierda = Math.max(-maxPow, Math.min(powAtrasIzquierda, maxPow));
        powDelanteDerecha = Math.max(-maxPow, Math.min(powDelanteDerecha, maxPow));
        powAtrasDerecha = Math.max(-maxPow, Math.min(powAtrasDerecha, maxPow));

        delanteDe.setPower(powDelanteDerecha);
        delanteIz.setPower(powDelanteIzquierda);
        atrasIz.setPower(powAtrasIzquierda);
        atrasDe.setPower(powAtrasDerecha);
    }

    public void Elevador(boolean elevadorUp, boolean elevadorDown) {
        if (elevadorUp) {
            elevador.setPower(-1);
        } else if (elevadorDown) {
            elevador.setPower(1);
        } else {
            elevador.setPower(0);
        }
    }

    public void Intake(boolean dropPiece, boolean takePiece) {
        if (dropPiece) {
            Intake.setPower(-1);
        } else if (takePiece) {
            Intake.setPower(1);
        } else {
            Intake.setPower(0);
        }
    }

}
