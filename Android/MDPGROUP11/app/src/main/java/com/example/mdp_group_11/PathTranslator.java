package com.example.mdp_group_11;

import android.util.Log;
import android.widget.Toast;

// To translate the relayed path given from algo to rpi and then to android to update robot position in "real-time"
// Basically will involve converting stm commands into its 'equivalent' on the 20x20 grid map
public class PathTranslator {
    private static final String TAG = "PathTranslator";
    private static GridMap gridMap;
    private static final int CELL_LENGTH = 10; //length of each cell in cm
    private static final int MILLI_DELAY = 200;    // delay between movement commands

    private int curX, curY;
    private String dir;

    public PathTranslator(GridMap gridMap) {
        this.gridMap = gridMap;
        this.curX = 2;
        this.curY = 1;
        this.dir = "up";    // up,down,left,right
    }

    public void translatePath(String stmCommand) {
        showLog("Entered translatePath");
        char commandType = 'z'; //commandType is a single character for switch case
        int commandValue = 0; //commandValue represents the value to move forwards or backwards.

        //Case 1: is a MOVE command. Expects a syntax of eg. MOVE,<DISTANCE IN CM>,<DIRECTION>.
        if(stmCommand.contains("MOVE")){

            try { //set commandValue to <DISTANCE IN CM>
                commandValue = Integer.parseInt(stmCommand.split(",")[1]);

            } catch(Exception e) {}

            //set commandType for <DIRECTION>
            String direction = stmCommand.split(",")[1];
            showLog("directions"+direction);

            commandValue = Integer.valueOf(stmCommand.split(",")[2].replace("\n",""));

            if(direction.equals("FORWARD")){
                commandType = 'f';
            }
            else if (direction.equals("BACKWARD")){
                commandType = 'b';
            }
        }
        //Case 2: is a TURN command. Expects a syntax of eg. TURN,<DIRECTION>.
        else if (stmCommand.contains("TURN")){
            String direction = stmCommand.split(",")[1];
            //set commandType for <DIRECTION> for Turns
            if(direction.equals("FORWARD_RIGHT")){
                commandType = 'd';
            }
            if(direction.equals("FORWARD_LEFT")){
                commandType = 'a';
            }
            if (direction.equals("BACKWARD_RIGHT")){
                commandType = 'e';
            }
            if (direction.equals("BACKWARD_LEFT")){
                commandType = 'q';
            }
        }

        int moves = 0;
        switch(commandType) {
            case 'f':   // forward
                moves = commandValue / CELL_LENGTH;
                for(int i = 0; i < moves; i++) {
                    gridMap.moveRobot("forward");

                    Home.refreshLabel();    // update x and y coordinate displayed
                    // display different statuses depending on validity of robot action
                    if (gridMap.getValidPosition()){
                        showLog("moving forward");}
                    else {
//                        Home.printMessage("obstacle");
                        showLog("Unable to move forward");
                    }

                    Home.printMessage("f");

                    try {
                        Thread.sleep(MILLI_DELAY);
                    } catch(InterruptedException e) {
                        showLog("InterruptedException occurred when calling Thread.sleep()!");
                        e.printStackTrace();
                    }
                }
                break;
            case 'b':   // backwards
                moves = commandValue / CELL_LENGTH;
                for(int i = 0; i < moves; i++) {
                    gridMap.moveRobot("back");
                    Home.refreshLabel();
                    try {
                        Thread.sleep(MILLI_DELAY);
                    } catch(InterruptedException e) {
                        showLog("InterruptedException occurred when calling Thread.sleep()!");
                        e.printStackTrace();
                    }
                }
                break;
            case 'd':   // 90 deg right
                gridMap.moveRobot("right");
                Home.refreshLabel();
                break;
            case 'a':   // 90 deg left
                gridMap.moveRobot("left");
                break;
            case 'q':   // 90 deg back-left
                gridMap.moveRobot("backleft");
                break;
            case 'e':   // 90 deg back-right
                gridMap.moveRobot("backright");
                break;
            case 's':   // stop to scan
                Toast.makeText(gridMap.getContext(), "Scanning image...",Toast.LENGTH_SHORT).show();
                try {
                    Thread.sleep(1000);
                } catch(InterruptedException e) {
                    showLog("InterruptedException occurred when calling Thread.sleep()!");
                    e.printStackTrace();
                }
                break;
            default:
                showLog("Invalid commandType!");
        }
        showLog("Exited translatePath");
    }

    private static void showLog(String message) {
        Log.d(TAG, message);
    }
}
