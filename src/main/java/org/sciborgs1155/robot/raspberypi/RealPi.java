package org.sciborgs1155.robot.raspberypi;

import java.lang.reflect.Parameter;
import java.util.ArrayList;
import java.util.List;

import org.sciborgs1155.robot.shooter.Cache.NoteTrajectory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;

public class RealPi implements PiIO {
    private static final double tillTimeout = 0.5;

    private final int currentInstanceNum;
    private final StringPublisher configPublisher;
    private final StringPublisher requestPublisher;
    private final List<DoubleArraySubscriber> resultSubscribers = new ArrayList();
    private final List<IntegerSubscriber> pingSubscribers = new ArrayList();
    private final double[] lastPing;
    
    public RealPi(int currentInstanceNum){
        this.currentInstanceNum = currentInstanceNum;
        lastPing = new double[currentInstanceNum];

        var AiosTable = NetworkTableInstance.getDefault().getTable("Aios");
        configPublisher = AiosTable.getStringTopic("config").publish(PubSubOption.periodic(0.0));
        requestPublisher = 
            AiosTable
                .getStringTopic("request")
                .publish(PubSubOption.periodic(0.0),PubSubOption.keepDuplicates(true));
        


    }
    @Override
    public void updateModelState(NoteTrajectory currentShooter, Translation2d currentPosition) {
       
    }

    @Override
    public void config(String jsonConfig) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'config'");
    }

    @Override
    public void request(Parameter parameter) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'request'");
    }
    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'close'");
    }
}
