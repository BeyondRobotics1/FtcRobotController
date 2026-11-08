package org.firstinspires.ftc.teamcode.biobuzz.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Trigger {
    Servo trigger;
    double openPosition;
    double closePosition;
    /*for nectar
    nectar open position: 0.0
    nectar close position: 0.21
      for pollen
    pollen open position: 0.03
    pollen close position: 0.27
    */
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
                   double initClosePositon){
        trigger = hardwareMap.get(Servo.class, servoName);
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
