package org.sciborgs1155.robot.raspberypi;



import java.lang.reflect.Parameter;

import org.sciborgs1155.robot.shooter.Cache.NoteTrajectory;

import edu.wpi.first.math.geometry.Translation2d;
/*
 * Generalized way the Pi works on a simulated and real robot
 */
public interface PiIO extends AutoCloseable{
    /*
     * Updates the state of the Pi
     * 
     * @param currentShooter The state of the shooter at a given time.
     * @param currentPosition The position the robot is at the same time.
     */
    public void updateModelState(NoteTrajectory currentShooter, Translation2d currentPosition);



    public void config(String jsonConfig);

    public void request(Parameter parameter);

}
