package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Chasis Mejorado", group = "Linear OpMode")
public class Plesiov1 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor delanteIz, delanteDe, atrasIz, atrasDe;
    private DcMotor Intake, elevador;
    private CRServo garra;

    private boolean garraAbierta = false;
    private boolean ButtonY = false;
    private boolean ButtonA = false;

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

        delanteIz.setDirection(DcMotor.Direction.FORWARD);
        delanteDe.setDirection(DcMotor.Direction.REVERSE);
        atrasIz.setDirection(DcMotor.Direction.FORWARD);
        atrasDe.setDirection(DcMotor.Direction.REVERSE);

        // Configurar la garra en estado inicial (cerrada)
        garra.setPower(0);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Movimiento del chasis
            double x = gamepad1.left_stick_x * 1.1;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double powDelanteIzquierda = (y + x + rx) / denominator;
            double powAtrasIzquierda = (y - x + rx) / denominator;
            double powDelanteDerecha = (y - x - rx) / denominator;
            double powAtrasDerecha = (y + x - rx) / denominator;

            delanteIz.setPower(powDelanteIzquierda);
            atrasIz.setPower(powAtrasIzquierda);
            delanteDe.setPower(powDelanteDerecha);
            atrasDe.setPower(powAtrasDerecha);

            // Control del elevador
            Elevador(gamepad2.dpad_up, gamepad2.dpad_down);

            // Control del Intake
            Intake(gamepad2.x, gamepad2.b);

            // Control de la garra con lógica toggle
            if (gamepad2.y && !ButtonY) {
                garraAbierta = true;
            }
            if (gamepad2.a && !ButtonA) {
                garraAbierta = false;
            }

            // Actualizar estado de la garra con menor potencia
            if (garraAbierta) {
                garra.setPower(0.01);
            } else {
                garra.setPower(-0.7);
            }

            // Actualizar estados previos de los botones
            ButtonY = gamepad2.y;
            ButtonA = gamepad2.a;

            // Mostrar información en la telemetría
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.addData("Garra Abierta", garraAbierta);
            telemetry.addData("x  ", x);
            telemetry.addData("y  ", y);
            telemetry.addData("rotation  ", rx);
            telemetry.update();
        }
    }

    public void Elevador(boolean flechaUp, boolean flechaDown) {
        if (flechaUp) {
            elevador.setPower(-1);
        } else if (flechaDown) {
            elevador.setPower(1);
        } else {
            elevador.setPower(0);
        }
    }

    public void Intake(boolean buttonX, boolean buttonB) {
        if (buttonX) {
            Intake.setPower(1);
        } else if (buttonB) {
            Intake.setPower(-1);
        } else {
            Intake.setPower(0);
        }
    }
}
