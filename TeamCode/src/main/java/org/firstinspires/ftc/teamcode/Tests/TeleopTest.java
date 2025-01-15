/*Hola. Esta es la primer revision del programa del teleoperado, hecha por mi (alfonso)
* Se cambiaron las funciones de mecanismos para estar acopladas a los botones que nuestro
* querido driver de mecanismos escogio.
*
* se cambio el nombre de las funciones para hacerlo mas facil de entender
*
* ya no se define cada boton como un booleano, porque es redundante. en cambio, cuando se llama cada
* funcion de mecanismos, se pone en los parametros los botones que se usaran, haciendo mas facil el intercambio de ellos
* y que no tengas que inicializarlos cada vez
*
* quisiera cambiar los nombres de los motores y servos de mecanismos nuevos porque se me hicieron confusos, pero habria
* que verlo como equipo para tambien cambiarlos en la driver hub
* */

package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Plesiov1.1")
public class TeleopTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor delanteIz, delanteDe, atrasIz, atrasDe;
    private DcMotor Intake = null;
    private DcMotor Elevador = null;
    private DcMotor Elevador2 = null;
    private DcMotor Outtake = null;
    private CRServo ServoOuttake = null;
    private CRServo Muñeca = null;
    private CRServo Brazo = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Motores chasis
        delanteIz = hardwareMap.get(DcMotor.class, "delanteIz");
        delanteDe = hardwareMap.get(DcMotor.class, "delanteDe");
        atrasIz = hardwareMap.get(DcMotor.class, "atrasIz");
        atrasDe = hardwareMap.get(DcMotor.class, "atrasDe");
        //Motores mecanismos
        Elevador = hardwareMap.get(DcMotor.class, "elevador"); //Uno de los dos motores del elevador
        Elevador2 = hardwareMap.get(DcMotor.class, "Elevador2");
        Intake = hardwareMap.get(DcMotor.class, "Intake"); //Motor del riel
        Outtake = hardwareMap.get(DcMotor.class, "outtake"); //Motor de la muñeca del outtake
        //Servos mecanismos
        ServoOuttake = hardwareMap.get(CRServo.class, "garraO"); //servo de la garra del outtake
        Muñeca = hardwareMap.get(CRServo.class, "muñeca"); //servo de la garra del intake
        Brazo = hardwareMap.get(CRServo.class, "brazo"); //servo de la muneca del intake

        // Direcciones
        delanteIz.setDirection(DcMotor.Direction.REVERSE);
        delanteDe.setDirection(DcMotor.Direction.FORWARD);
        atrasIz.setDirection(DcMotor.Direction.REVERSE);
        atrasDe.setDirection(DcMotor.Direction.FORWARD);

        // Comportamiento al soltar
        zeroPowerBehaviourBrake();

        waitForStart();
        runtime.reset();

        //Programa principal, se repite por siempre
        while (opModeIsActive()) {
            chasis(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, 0.9); //controla motores del chasis

            elevador(gamepad2.dpad_up, gamepad2.dpad_down); //controla motores del elevador(arriba, abajo)
            intakeWrist(gamepad2.left_stick_y); //controla la muneca del intake
            intake(gamepad2.y, gamepad2.a); //controla la garra del intake (tomar, soltar)
            outtake(gamepad2.x, gamepad2.b); //controla garra del outtake (tomar, soltar)
            riel(gamepad2.dpad_left, gamepad2.dpad_right); //controla el riel del intake (adelante, atras)
            outtakeWrist(gamepad2.right_stick_y); //controla la muneca del outtake

            telemetry.addData("Runtime", runtime.seconds());
            telemetry.update();
        }
    }

    //settea todos los motores en brake cuando no se les aplica potencia
    public void zeroPowerBehaviourBrake(){
        delanteIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delanteDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        atrasIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        atrasDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /*-------------AQUI INICIAN FUNCIONES DE MOVIMIENTO EN EL ROBOT-----------------*/

    /*-------------CHASIS-----------------*/
    //funcion de movimiento en chasis.
    //Parametros: joystick del valor en x, en y, y de la rotacion. Valor maximo de potencia que se aplica a motores
    public void chasis(double x, double y, double rx, double maxPow) {
        //formula de movimiento mecanum almacenado en variable
        double powDelanteIzquierda = (y + x + rx);
        double powAtrasIzquierda = (y - x + rx);
        double powDelanteDerecha = (y - x - rx);
        double powAtrasDerecha = (y + x - rx);

        //Se limita el valor maximo que puede dar la variable de la formula al maximo escogido en el parametro
        powDelanteIzquierda = Math.max(-maxPow, Math.min(powDelanteIzquierda, maxPow));
        powAtrasIzquierda = Math.max(-maxPow, Math.min(powAtrasIzquierda, maxPow));
        powDelanteDerecha = Math.max(-maxPow, Math.min(powDelanteDerecha, maxPow));
        powAtrasDerecha = Math.max(-maxPow, Math.min(powAtrasDerecha, maxPow));

        //se aplica la potencia normalizada
        delanteDe.setPower(powDelanteDerecha);
        delanteIz.setPower(powDelanteIzquierda);
        atrasIz.setPower(powAtrasIzquierda);
        atrasDe.setPower(powAtrasDerecha);
    }

    /*-------------MECANISMOS-----------------*/

    //funcion de control del elevador. con cada boton se prenden dos motores
    public void elevador(boolean up, boolean down) {
        if (up) {
            Elevador.setPower(-1);
            Elevador2.setPower(-1);
        } else if (down) {
            Elevador.setPower(1);
            Elevador2.setPower(1);
        } else {
            Elevador.setPower(0);
            Elevador2.setPower(0);
        }
    }

    //funcion de garra de outtake con dos botones
    public void outtake(boolean open, boolean close) {
        if (open) {
            ServoOuttake.setPower(-1);
        } else if (close) {
            ServoOuttake.setPower(1);
        } else {
            ServoOuttake.setPower(0);
        }
    }

    //funcion de la muneca del intake. se controla con un joystick
    public void intakeWrist(double y) {
        Brazo.setPower(y);
    }

    //control de la garra del intake con dos botones
    public void intake(boolean open, boolean close) {
        if (open) {
            Muñeca.setPower(-1);
        } else if (close) {
            Muñeca.setPower(1);
        } else {
            Muñeca.setPower(0);
        }
    }

    //funcion de movimiento del intake, con dos botones
    public void riel(boolean forward, boolean backward) {
        if (forward) {
            Intake.setPower(1);
        } else if (backward) {
            Intake.setPower(-1);
        } else {
            Intake.setPower(0);
        }
    }

    //funcion de la muneca del outtake(motor core hex). se controla con el joystick
    public void outtakeWrist(double y) {
        Outtake.setPower(y);
    }

}
