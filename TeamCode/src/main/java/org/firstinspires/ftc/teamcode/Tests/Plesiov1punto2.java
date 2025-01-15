package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Plesiov1.2")
public class Plesiov1punto2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor DelanteIz, DelanteDe, AtrasIz, AtrasDe;
    private DcMotor Intake = null;
    private DcMotor ElevadorIz = null;
    private DcMotor ElevadorDe = null;
    private DcMotor Outtake = null;
    private CRServo ServoOuttake = null;
    private CRServo Muñeca = null;
    private CRServo Brazo = null;

    //variables para el Toggle en las garras de intake y outtake
    private boolean outtakeMode = false; //para saber el modo en el que esta el outtake (abrir/cerrar)
    private boolean outtakeButtonState = false; //para saber si el boton ha sido presionado
    private boolean intakeMode = false; //para saber el modo en el que esta el intake(abrir/cerrar)
    private boolean intakeButtonState = false; //para saber si el boton ha sido presionado

    private static final int maxEncoderRiel = -1700; //constante de el numero maximo de ticks que se puede recorrer el riel
    private static final int maxOuttake = -100;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Programa de Teleoperado version 1.2");
        telemetry.addLine("Controles:" +
                "GAMEPAD 1 (CHASIS)" +
                "Joystick izquierdo: Movmiento en x, y. Joystick derecho: Rotacion sobre su propio eje" +
                "GAMEPAD 2 (MECANISMOS)" +
                "Joystick izquierdo: muñeca intake. Joystick derecho: muñeca outtake" +
                "Boton Y: Garra Intake. Boton X: Garra outtake. Boton A: Transfer de piezas" +
                "DPAD Arriba: Elevador hacia arriba. DPAD Abajo: Elevador hacia abajo" +
                "DPAD izquierda: Extender riel. DPAD Derecha: Contraer Riel");
        telemetry.update();

        //Motores chasis
        DelanteIz = hardwareMap.get(DcMotor.class, "DelanteIz");
        DelanteDe = hardwareMap.get(DcMotor.class, "DelanteDe");
        AtrasIz = hardwareMap.get(DcMotor.class, "AtrasIz");
        AtrasDe = hardwareMap.get(DcMotor.class, "AtrasDe");
        //Motores mecanismos
        ElevadorIz = hardwareMap.get(DcMotor.class, "elevadorIz"); //Uno de los dos motores del elevador
        ElevadorDe = hardwareMap.get(DcMotor.class, "elevadorDe");
        Intake = hardwareMap.get(DcMotor.class, "Intake"); //Motor del riel
        Outtake = hardwareMap.get(DcMotor.class, "outtake"); //Motor de la muñeca del outtake
        //Servos mecanismos
        ServoOuttake = hardwareMap.get(CRServo.class, "garraO"); //servo de la garra del outtake
        Muñeca = hardwareMap.get(CRServo.class, "muñeca"); //servo de la garra del intake
        Brazo = hardwareMap.get(CRServo.class, "brazo"); //servo de la muneca del intake

        // Direcciones
        DelanteIz.setDirection(DcMotor.Direction.REVERSE);
        DelanteDe.setDirection(DcMotor.Direction.FORWARD);
        AtrasIz.setDirection(DcMotor.Direction.REVERSE);
        AtrasDe.setDirection(DcMotor.Direction.FORWARD);

        // Comportamiento al soltar
        zeroPowerBehaviorBrake();

        resetEncoders();

        waitForStart();
        runtime.reset();

        //Programa principal, se repite por siempre
        while (opModeIsActive()) {
            chasis(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, 0.9);

            intakeWrist(gamepad2.left_stick_y);
            intakeToggle(gamepad2.y);
            outtakeToggle(gamepad2.x);
            riel(gamepad2.dpad_right, gamepad2.dpad_left);
            outtakeWrist(gamepad2.right_stick_y, 0.2);
            transfer(gamepad2.a);
            elevador(gamepad2.dpad_up, gamepad2.dpad_down);
            //outtake(gamepad2.x, gamepad2.b);
            //intake(gamepad2.y, gamepad2.a);

            //imprimir ciertos valores para debuggear
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.addData("Riel", Intake.getCurrentPosition());
            telemetry.addData("Muñeca outtake:", Outtake.getCurrentPosition());
            telemetry.update();
        }
    }

    //settea todos los motores en brake cuando no se les aplica potencia
    public void zeroPowerBehaviorBrake(){
        DelanteIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DelanteDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AtrasIz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AtrasDe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //se configuran los encoders para poder acceder a las lecturas y configurar limites
    public void resetEncoders() {
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevadorIz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevadorDe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElevadorIz.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElevadorDe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        //se aplica la potencia con limites
        DelanteDe.setPower(powDelanteDerecha);
        DelanteIz.setPower(powDelanteIzquierda);
        AtrasIz.setPower(powAtrasIzquierda);
        AtrasDe.setPower(powAtrasDerecha);
    }

    /*-------------MECANISMOS-----------------*/

    //funcion de control del elevador. con cada boton se prenden dos motores
    public void elevador(boolean up, boolean down) {
        if (up) {
            ElevadorIz.setPower(-1);
            ElevadorDe.setPower(-1);
        } else if (down) {
            ElevadorIz.setPower(1);
            ElevadorDe.setPower(1);
        } else {
            ElevadorIz.setPower(0);
            ElevadorDe.setPower(0);
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

    //funcion para abrir/cerrar la garra del intake con un solo boton
    public void outtakeToggle(boolean button) { //parametro del boton que se usara
        if (button && !outtakeButtonState) { //si el boton es presionado una vez
            outtakeMode = !outtakeMode; //cambia el modo (abrir a cerrar, o cerrar a abrir)
            outtakeButtonState = true; //guarda que esta presionado
        }

        if(!button){ //si no es presionado
            outtakeButtonState=false;  //guardar que no esta siendo presionado
        }

        double power=0;
        if (!outtakeMode) power = -0.5; //si el modo es cerrar, aplicar power negativo
        else power = 0.5; //si es abrir, aplicar power positivo
        ServoOuttake.setPower(power);
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

    //funcion para abrir/cerrar la garra del intake con un solo boton
    public void intakeToggle(boolean button) { //parametro del boton que se usara
        if (button && !intakeButtonState) { //si el boton es presionado una vez
            intakeMode = !intakeMode; //cambia el modo (abrir a cerrar, o cerrar a abrir)
            intakeButtonState = true; //guarda que esta presionado
        }

        if(!button){ //si no es presionado
            intakeButtonState=false;  //guardar que no esta siendo presionado
        }

        double power=0;
        if (!intakeMode) power = -0.5; //si el modo es cerrar, aplicar power negativo
        else power = 0.5; //si es abrir, aplicar power positivo
        Muñeca.setPower(power);
    }

    //funcion de movimiento del intake, con dos botones
    public void riel(boolean forward, boolean backward) {
        int rielCurrentPos = Intake.getCurrentPosition();

        //si presionas el boton de adelante pero se pasa de la distancia maxima
        if(forward && rielCurrentPos < maxEncoderRiel){
            Intake.setPower(0);
            return; //finalizar la funcion aqui
        }

        if (forward) {
            Intake.setPower(-1);
        } else if (backward) {
            Intake.setPower(1);
        } else {
            Intake.setPower(0);
        }
    }

    public void transfer(boolean button) {
        if(button){
            Intake.setTargetPosition(0);
            Intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Intake.setPower(0.8);

            while (opModeIsActive() && Intake.isBusy()) {
                telemetry.addData("Encoder Riel:", Intake.getCurrentPosition());
                telemetry.update();
            }

            Intake.setPower(0);
            Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            Brazo.setPower(-1);
            sleep(600);
            Brazo.setPower(0);

        }
    }

    //funcion de la muneca del outtake(motor core hex). se controla con el joystick
    public void outtakeWrist(double y, double maxPow) {
        int outtakeCurrentPos = Outtake.getCurrentPosition();

        //si presionas el boton de adelante pero se pasa de la distancia maxima
        if(y < 0.0 && outtakeCurrentPos < maxOuttake){
            Outtake.setPower(0);
            return; //finalizar la funcion aqui
        }

        double outtakePower = Math.max(-maxPow, Math.min(y, maxPow));

        Outtake.setPower(y);
    }

}
