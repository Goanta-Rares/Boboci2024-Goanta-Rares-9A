package org.firstinspires.ftc.teamcode.drive.opmodetele;

import static java.lang.Math.abs;//Este importata functia abs pentru a se calcula valoarea absoluta

import com.acmerobotics.dashboard.FtcDashboard;//FtcDashboard este folosit pentru afisarea in timp real
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;//Sunt afisate informatii despre robot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;//Clasa importanta pentru definirea unui mod de operare teleop
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;//Teleop este un mod de operare ce permite controlul manual al robotului  
import com.acmerobotics.roadrunner.geometry.Pose2d;//Pose2d este folosita pentru a controla miscarea robotului


import org.firstinspires.ftc.teamcode.drive.robot.Robot;//este importata clasa robot care defineste modul de functionare al robotului 

@TeleOp(name="MecanumDriveMode", group="Linear OpMode")

public class LinearDriveMode extends LinearOpMode {
    private Robot robot = null;
    int direction = 1;//variabila folosita pentru controlarea directiei miscarii 
    double servoPosSlides = 0.5;//variabila folosita pentru pozitia si directia de miscare a motoarelor
    double servoPosGrippy = 0;
    public double calculateThrottle(float x) {
        int sign = -1;//semnul este initializat ca a fiind negativ
        if (x > 0) sign = 1;//daca valoatea x este pozitiva, atunci semnul devine pozitiv
        return sign * 3 * abs(x);//throttle-ul ia valoare de trei ori mai mare ca a lui x
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Initializing...");//se adauga un mesaj referitor la starea robotului pe board
        telemetry.update();

        robot = new Robot(hardwareMap);//obiectul robot este initializat cu hardware
        while (robot.isInitialize() && opModeIsActive()) {//acest ciclu while continua pana cand robotul este initializat si modul este activ
            idle();
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData(">", "Initialized");
        telemetry.update();



        waitForStart();
        if (isStopRequested()) return;//asteapta ca utilizatorul sa inceapa modul de operare


        while (opModeIsActive()) {//functioneaza cat timp modul este activ



            if (gamepad2.left_bumper) {//daca este folosit butonul L1 de pe controler, glisiera robotului se extinde
                robot.crane.slidesDirection = 1;
                robot.crane.setSlides(5);
                if(robot.crane.slideEncoderLastPosition > robot.crane.slideEncoder.getVoltage()){//valoarea tensiunii este folosita pentru a determina pozitia bratului
                    robot.crane.slideExtension -= 3.3;
                }
            } else if (gamepad2.right_bumper) {//Daca se apasa butonul de R1 de pe controler, atunci glisiera se va misca in directia opusa
                robot.crane.slidesDirection = -1;
                robot.crane.setSlides(5);
                if(robot.crane.slideEncoderLastPosition < robot.crane.slideEncoder.getVoltage()){//valoarea tensiunii este folosita pentru a determina pozitia bratului
                    robot.crane.slideExtension += 3.3;
                }
            } else {
               robot.crane.setSlides(0);
            }
            robot.crane.slideEncoderLastPosition = robot.crane.slideEncoder.getVoltage();//valoarea tensiunii este folosita pentru a determina pozitia bratului


            if(gamepad2.left_trigger > 0.1){
                robot.crane.craneTarget -= (int) calculateThrottle(gamepad2.left_trigger);//Prin apasarea butoanelor L2 si R2 este ajustata pozitia gripperului, prin setarea craneTarget
            }
            else if(gamepad2.right_trigger > 0.1){
                robot.crane.craneTarget += (int) calculateThrottle(gamepad2.right_trigger);//Prin apasarea butoanelor L2 si R2 este ajustata pozitia gripperului, prin setarea craneTarget
            }
            robot.crane.motorCrane1.setPower(robot.crane.cranePower(robot.crane.craneTarget));//Puterea motoarelor este ajustata de craneTarget 
            robot.crane.motorCrane2.setPower(robot.crane.cranePower(robot.crane.craneTarget));

            if (gamepad2.a) {
                robot.crane.gripperDirection = 1;//prin apasarea butonului a, gripperul se deschide
                robot.crane.setGripper(1);
            }
            else if (gamepad2.b) {
                robot.crane.gripperDirection = -1;//prin apasarea butonului b, gripperul se inchide
                robot.crane.setGripper(1);
            }
            else robot.crane.setGripper(0);

            robot.drive.setWeightedDrivePower(new Pose2d((-gamepad1.left_stick_y),(-gamepad1.left_stick_x),(-gamepad1.right_stick_x)));//prin left stick y se controleaza miscarea inainte si inapoi, prin left stick x se controleaza miscarea laterala iar prin right stick x se controleaza rotatia 








            telemetry.addData("crane target: ", robot.crane.craneTarget);//sunt afisate informatii referitoare la pozitia bratelor, la senzori si la starea robotului
                telemetry.addData("right trigger: ", gamepad2.right_trigger);
                telemetry.addData("encoder value: ", robot.crane.slideEncoder.getVoltage());
                telemetry.addData("last position ", robot.crane.slideEncoderLastPosition);
                telemetry.addData("slide extension ", robot.crane.slideExtension);
                telemetry.addData("sensor touch: ", robot.crane.slideSensor.isPressed());
//                telemetry.addData("CRANE TICKS LEFT: ", robot.crane.motorCraneLeft.getCurrentPosition());
//                telemetry.addData("CRANE TICKS RIGHT: ", robot.crane.motorCraneRight.getCurrentPosition());
//                telemetry.addData("DIRECTION: ", direction);
//                telemetry.addData("SERVO GRIPPER: ", robot.crane.servoGrippy1.getPosition());
                telemetry.update();
            }

        }

    }



