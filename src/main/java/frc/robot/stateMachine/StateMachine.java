package frc.robot.stateMachine;

import java.util.ArrayList;
import java.util.Arrays;

public class StateMachine {
    private ArrayList<Path> paths;
    private String desiredState;
    private String currentState;
    private int desiredPathIndex = -1;
    public StateMachine(String c) {
        currentState = c;
        desiredState = "";
    }
    public StateMachine(String c, String d, ArrayList<Path> p) {
        currentState = c;
        desiredState = d;
        paths = p;
    }
    public void setCurrentState(String c) {
        currentState = c;
        setPathIndex();
    }
    private void setPathIndex () {
        for(int i = 0; i < paths.size(); i++) {
            if(paths.get(i).checkForValidPath(currentState, desiredState)) {
                desiredPathIndex = i;
            }
       }
       if(desiredPathIndex == -1) {
            // no path possible
       }
    }
    public void setDesiredState(String desired) {
        desiredState = desired;
        setPathIndex();
    }
    public String getNextState() {
        return paths.get(desiredPathIndex).nextEdge();
    }
    public void addPath(String base, String[] p) {
        Path temp = new Path(base, p);
        paths.add(temp);
    }
}
