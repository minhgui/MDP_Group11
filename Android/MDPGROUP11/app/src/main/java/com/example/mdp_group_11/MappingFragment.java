package com.example.mdp_group_11;

import android.content.Context;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.CompoundButton;
import android.widget.ImageButton;
import android.widget.Switch;
import android.widget.Toast;
import android.widget.ToggleButton;

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;

public class MappingFragment extends Fragment {
    private static final String TAG = "MapFragment";

    SharedPreferences mapPref;
    private static SharedPreferences.Editor editor;

    ImageButton resetMapBtn, saveMapObstacle, loadMapObstacle;
    ImageButton directionChangeImageBtn, obstacleImageBtn;
    ToggleButton setStartPointToggleBtn;
    GridMap gridMap;
    Switch dragSwitch;
    Switch changeObstacleSwitch;

    static String imageID="";
    static String imageBearing="North";
    static boolean dragStatus;
    static boolean changeObstacleStatus;

    String direction = "";

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
    }

    @Override
    public View onCreateView(
            @NonNull LayoutInflater inflater, ViewGroup container,
            Bundle savedInstanceState) {
        View root = inflater.inflate(R.layout.activity_map_config, container,  false);
        SharedPreferences sharedPreferences = getActivity().getSharedPreferences("Shared Preferences", Context.MODE_PRIVATE);
        editor = sharedPreferences.edit();
        direction = sharedPreferences.getString("direction","");

        if (savedInstanceState != null)
            direction = savedInstanceState.getString("direction");
        gridMap = Home.getGridMap();

        resetMapBtn = root.findViewById(R.id.resetBtn);
        setStartPointToggleBtn = root.findViewById(R.id.startpointToggleBtn);
        directionChangeImageBtn = root.findViewById(R.id.changeDirectionBtn);
        obstacleImageBtn = root.findViewById(R.id.addObstacleBtn);
        saveMapObstacle = root.findViewById(R.id.saveBtn);
        loadMapObstacle = root.findViewById(R.id.loadBtn);
        dragSwitch = root.findViewById(R.id.dragSwitch);
        changeObstacleSwitch = root.findViewById(R.id.changeObstacleSwitch);

        resetMapBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                showToast("Reseting map...");
                Home.printMessage("CLEAR");
                gridMap.resetMap();

            }
        });

        // switch for dragging
        dragSwitch.setOnCheckedChangeListener( new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton toggleButton, boolean isChecked) {
                showToast("Dragging is " + (isChecked ? "on" : "off"));
                dragStatus = isChecked;
                if (dragStatus) {
                    gridMap.setSetObstacleStatus(false);
                    changeObstacleSwitch.setChecked(false);
                }
            }
        });

        // switch for changing obstacle
        changeObstacleSwitch.setOnCheckedChangeListener( new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton toggleButton, boolean isChecked) {
                showToast("Changing Obstacle is " + (isChecked ? "on" : "off"));
                changeObstacleStatus = isChecked;
                if (changeObstacleStatus) {
                    gridMap.setSetObstacleStatus(false);
                    dragSwitch.setChecked(false);
                }
            }
        });

        setStartPointToggleBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                // 2nd consecutive tap on the toggle btn (logic to handle other buttons being tapped is in gridmap.toggleCheckedBtn())
                if (setStartPointToggleBtn.getText().equals("SET START POINT")) {
                    showToast("Cancelled select starting point");
                    setStartPointToggleBtn.setBackgroundResource(R.drawable.border_black);
                }
                else {  // 1st tap on the toggle btn
                    showToast("Please select starting point");
                    gridMap.setStartCoordStatus(true);
                    gridMap.toggleCheckedBtn("setStartPointToggleBtn");
                    setStartPointToggleBtn.setBackgroundResource(R.drawable.border_black_pressed);
                }
            }
        });

        saveMapObstacle.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                String getObsPos = "";
                mapPref = getContext().getSharedPreferences("Shared Preferences", Context.MODE_PRIVATE);
                editor = mapPref.edit();
                if(!mapPref.getString("maps", "").equals("")){
                    editor.putString("maps", "");
                    editor.commit();
                }
                getObsPos = GridMap.saveObstacleList();
                editor.putString("maps",getObsPos);

                // Save Robot's position
                int[] robotPos = gridMap.getCurCoord(); // Get current robot coordinates
                String robotDir = gridMap.getRobotDirection(); // Get current robot direction

                if (robotPos != null && robotPos.length == 2) {
                    int robotX = robotPos[0] - 1; // Subtract 1 from X
                    int robotY = robotPos[1] - 1; // Subtract 1 from Y

                    editor.putInt("robot_x", robotX); // Save adjusted X position
                    editor.putInt("robot_y", robotY); // Save adjusted Y position
                }

                // Save robot direction outside the if block (always saved)
                editor.putString("robot_direction", robotDir);
                editor.commit(); // Save all data

                showToast("Saved map");

            }
        });

        loadMapObstacle.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                mapPref = getContext().getSharedPreferences("Shared Preferences", Context.MODE_PRIVATE);
                String obsPos = mapPref.getString("maps","");
                if(!obsPos.equals("")){

                    String[] obstaclePosition = obsPos.split("\n");
                    for (String s : obstaclePosition) {

                        String[] coords = s.split(",");

                        String direction = "";
                        switch (coords[2]) {
                            case "N":
                                direction = "North";
                                break;
                            case "E":
                                direction = "East";
                                break;
                            case "W":
                                direction = "West";
                                break;
                            case "S":
                                direction = "South";
                                break;
                            default:
                                direction = "";
                        }
                        gridMap.imageBearings.get(Integer.parseInt(coords[1]))[Integer.parseInt(coords[0])] = direction;
                        gridMap.setObstacleCoord(Integer.parseInt(coords[0]) + 1, Integer.parseInt(coords[1]) + 1, true);
                        try {
                            Thread.sleep(50);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    }

                    // Load Robot’s saved position
                    int savedX = mapPref.getInt("robot_x", -1);
                    int savedY = mapPref.getInt("robot_y", -1);
                    String savedDirection = mapPref.getString("robot_direction", "None");

                    if (savedX != -1 && savedY != -1) {
                        // ✅ Ensure the robot is drawn
                        gridMap.setCurCoord(savedX + 1, savedY + 1, savedDirection); // Restore robot position
                        GridMap.canDrawRobot = true; // ✅ Explicitly allow robot drawing

                        // ✅ Send robot data to AMDTool/RPi
                        String dir = savedDirection.equals("up") ? "NORTH" :
                                savedDirection.equals("down") ? "SOUTH" :
                                        savedDirection.equals("left") ? "WEST" : "EAST";
                        // Center center
                        Home.printMessage("ROBOT," + (savedX - 1) + "," + (savedY + 1) + "," + dir.toUpperCase() + "," + "Z");
                    }

                    gridMap.invalidate();
                    showToast("Loaded saved map");
                }
                showToast("Empty saved map!");
            }
        });


        directionChangeImageBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                switch(direction)
                {
                    case "None":
                    case "up":
                        direction="right";
                        break;
                    case "right":
                        direction="down";
                        break;
                    case "down":
                        direction="left";
                        break;
                    case "left":
                        direction="up";
                        break;
                }
                editor.putString("direction",direction);
                Home.refreshDirection(direction);
                Toast.makeText(getActivity(), "Saving direction...", Toast.LENGTH_SHORT).show();
                editor.commit();
            }
        });

        // To place obstacles
        obstacleImageBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                if (!gridMap.getSetObstacleStatus()) {  // if setObstacleStatus is false
                    showToast("Please plot obstacles");
                    gridMap.setSetObstacleStatus(true);
                    gridMap.toggleCheckedBtn("obstacleImageBtn");
                    obstacleImageBtn.setBackgroundResource(R.drawable.border_black_pressed);
                }
                else if (gridMap.getSetObstacleStatus()) {  // if setObstacleStatus is true
                    gridMap.setSetObstacleStatus(false);
                    obstacleImageBtn.setBackgroundResource(R.drawable.border_black);
                }
                // disable the other on touch functions
                changeObstacleSwitch.setChecked(false);
                dragSwitch.setChecked(false);

            }
        });

        return root;
    }

    private void showToast(String message) {
        Toast.makeText(getContext(), message, Toast.LENGTH_SHORT).show();
    }
}
