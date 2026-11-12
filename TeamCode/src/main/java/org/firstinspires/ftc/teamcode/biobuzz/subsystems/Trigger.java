package org.firstinspires.ftc.teamcode.biobuzz.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Trigger {

    /*for nectar
    nectar open position: 0.0
    nectar close position: 0.21
      for pollen
    pollen open position: 0.03
    pollen close position: 0.27
    */
    public final static double kNectarTriggerOpenPosition = 0.23;
    public final static double kNectarTriggerClosePosition = 0;

    public final static double kPollenTriggerOpenPosition = 0.2;
    public final static double kPollenTriggerClosePosition = 0.;


    Servo trigger;
    double openPosition;
    double closePosition;

    /**
     *
     * @param hardwareMap
     * @param servoName
     * @param initOpenPosition
     * @param initClosePositon
     */
    public Trigger(HardwareMap hardwareMap,
                   String servoName,
                   double initOpenPosition,
                   double initClosePositon,
                   boolean reverseDirection){
        trigger = hardwareMap.get(Servo.class, servoName);

        if(reverseDirection)
            trigger.setDirection(Servo.Direction.REVERSE);

        openPosition = initOpenPosition;
        closePosition = initClosePositon;
    }

    public double getOpenPosition() {
        return openPosition;
    }
    public double getClosePosition(){
        return closePosition;
    }
    public void setOpenPosition(double newOpenPosition){
        openPosition = newOpenPosition;
    }
    public void setClosePosition(double newClosePosition){
        closePosition = newClosePosition;
    }
    public void setServoPosition(double newPosition){
        trigger.setPosition(newPosition);
    }
    public double getServoPosition(){
        return trigger.getPosition();
    }
    public void close(){
        trigger.setPosition(closePosition);
    }
    public void open(){
        trigger.setPosition(openPosition);
    }
}
