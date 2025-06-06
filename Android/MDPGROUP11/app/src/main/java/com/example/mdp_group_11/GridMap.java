package com.example.mdp_group_11;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.ClipData;
import android.content.Context;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Point;
import android.util.AttributeSet;
import android.util.Log;
import android.view.DragEvent;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.view.Window;
import android.view.WindowManager;
import android.widget.ArrayAdapter;
import android.widget.ImageButton;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import androidx.annotation.Nullable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.HashMap;
import java.util.UUID;

public class GridMap extends View {
    public GridMap(Context c) {
        super(c);
        initMap();
        setWillNotDraw(false);
    }

    SharedPreferences sharedPrefs;

    private final Paint blackPaint = new Paint();
    private final Paint whitePaint = new Paint();
    private final Paint maroonPaint = new Paint();
    private final Paint obstacleColor = new Paint();
    private final Paint imageColor = new Paint();
    private final Paint robotColor = new Paint();
    private final Paint endColor = new Paint();
    private final Paint startColor = new Paint();
    private final Paint waypointColor = new Paint();
    private final Paint unexploredColor = new Paint();
    private final Paint exploredColor = new Paint();
    private final Paint arrowColor = new Paint();
    private final Paint fastestPathColor = new Paint();

    private static String robotDirection = "None";
    private static int[] startCoord = new int[]{-1, -1};
    private static int[] curCoord = new int[]{-1, -1};
    private static int[] oldCoord = new int[]{-1, -1};
    private static ArrayList<int[]> obstacleCoord = new ArrayList<>();
    public static boolean canDrawRobot = false;
    private static boolean startCoordStatus = false;
    private static boolean setObstacleStatus = false;
    private static final boolean unSetCellStatus = false;
    private static final boolean setExploredStatus = false;
    private static boolean validPosition = false;
    private static final String TAG = "GridMap";
    private static final int COL = 20;
    private static final int ROW = 20;
    private static float cellSize;
    private static Cell[][] cells;
    Map<String, String> val2IdxMap;

    private boolean mapDrawn = false;

    public String getRobotDirection() {
        return robotDirection;
    }
    private void setValidPosition(boolean status) {
        validPosition = status;
    }
    public boolean getValidPosition() {
        return validPosition;
    }
    public void setSetObstacleStatus(boolean status) {
        setObstacleStatus = status;
    }
    public boolean getSetObstacleStatus() {
        return setObstacleStatus;
    }
    public void setStartCoordStatus(boolean status) {
        startCoordStatus = status;
    }
    private boolean getStartCoordStatus() {
        return startCoordStatus;
    }
    public boolean getCanDrawRobot() {
        return canDrawRobot;
    }
    private int[] getStartCoord() {
        return startCoord;
    }
    public int[] getCurCoord() {
        return curCoord;
    }
    private void calculateDimension() {
        this.setCellSize(getWidth() / (COL + 1));
    }
    private int convertRow(int row) {
        return (20 - row);
    }
    private void setCellSize(float cellSize) {
        GridMap.cellSize = cellSize;
    }
    private float getCellSize() {
        return cellSize;
    }

    public ArrayList<String[]> ITEM_LIST = new ArrayList<>(Arrays.asList(
            new String[20], new String[20], new String[20], new String[20], new String[20],
            new String[20], new String[20], new String[20], new String[20], new String[20],
            new String[20], new String[20], new String[20], new String[20], new String[20],
            new String[20], new String[20], new String[20], new String[20], new String[20]
    ));
    public static ArrayList<String[]> imageBearings = new ArrayList<>(Arrays.asList(
            new String[20], new String[20], new String[20], new String[20], new String[20],
            new String[20], new String[20], new String[20], new String[20], new String[20],
            new String[20], new String[20], new String[20], new String[20], new String[20],
            new String[20], new String[20], new String[20], new String[20], new String[20]
    ));

    static ClipData clipData;
    static Object localState;
    int initialColumn, initialRow;

    public GridMap(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        initMap();
        blackPaint.setStyle(Paint.Style.FILL_AND_STROKE);
        whitePaint.setColor(Color.WHITE);
        whitePaint.setTextSize(17);
        whitePaint.setTextAlign(Paint.Align.CENTER);
        maroonPaint.setColor(getResources().getColor(R.color.brightRed));
        maroonPaint.setStrokeWidth(8);
        obstacleColor.setColor(getResources().getColor(R.color.black));
        imageColor.setColor(getResources().getColor(R.color.rockColor));
        robotColor.setColor(getResources().getColor(R.color.pikaYellow));
        robotColor.setStrokeWidth(2);
        endColor.setColor(Color.RED);
        startColor.setColor(Color.CYAN);
        waypointColor.setColor(Color.GREEN);
        unexploredColor.setColor(getResources().getColor(R.color.skyBlue));
        exploredColor.setColor(getResources().getColor(R.color.grassColor));
        arrowColor.setColor(Color.BLACK);
        fastestPathColor.setColor(Color.MAGENTA);
        Paint newpaint = new Paint();
        newpaint.setColor(Color.TRANSPARENT);

        // get shared preferences
        sharedPrefs = getContext().getSharedPreferences("Shared Preferences",
                Context.MODE_PRIVATE);

        this.val2IdxMap = new HashMap<>();
    }

    private void initMap() {
        setWillNotDraw(false);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        Log.d(TAG, "Creating Cell");

        if (!mapDrawn) {
            mapDrawn = true;
            this.createCell();
        }

        drawIndividualCell(canvas);
        drawHorizontalLines(canvas);
        drawVerticalLines(canvas);
        drawGridNumber(canvas);
        if (getCanDrawRobot())
            drawRobot(canvas, curCoord);
        drawObstacles(canvas);

    }

    // draws obstacle cells whenever map refreshes
    private void drawObstacles(Canvas canvas) {
        for (int i = 0; i < obstacleCoord.size(); i++) { // for each recorded obstacle
            // get col and row (zero-indexed)
            int col = obstacleCoord.get(i)[0];
            int row = obstacleCoord.get(i)[1];

            if (ITEM_LIST.get(row)[col] == null || ITEM_LIST.get(row)[col].equals("")
                    || ITEM_LIST.get(row)[col].equals("Nil")) {
                whitePaint.setTextSize(15);
                canvas.drawText(
                        String.valueOf(i + 1),
                        cells[col + 1][19 - row].startX + ((cells[1][1].endX - cells[1][1].startX) / 2),
                        cells[col + 1][19 - row].startY + ((cells[1][1].endY - cells[1][1].startY) / 2) + 5,
                        whitePaint
                );
            } else {
                whitePaint.setTextSize(17);
                canvas.drawText(
                        ITEM_LIST.get(row)[col],
                        cells[col + 1][19 - row].startX + ((cells[1][1].endX - cells[1][1].startX) / 2),
                        cells[col + 1][19 - row].startY + ((cells[1][1].endY - cells[1][1].startY) / 2) + 10,
                        whitePaint
                );
            }

            switch (imageBearings.get(row)[col]) {
                case "North":
                    canvas.drawLine(
                            cells[col + 1][19 - row].startX,
                            cells[col + 1][19 - row].startY,
                            cells[col + 1][19 - row].endX,
                            cells[col + 1][19 - row].startY,
                            maroonPaint
                    );
                    break;
                case "South":
                    canvas.drawLine(
                            cells[col + 1][19 - row].startX,
                            cells[col + 1][19 - row].startY + cellSize,
                            cells[col + 1][19 - row].endX,
                            cells[col + 1][19 - row].startY + cellSize,
                            maroonPaint
                    );
                    break;
                case "East":
                    canvas.drawLine(
                            cells[col + 1][19 - row].startX + cellSize,
                            cells[col + 1][19 - row].startY,
                            cells[col + 1][19 - row].startX + cellSize,
                            cells[col + 1][19 - row].endY,
                            maroonPaint
                    );
                    break;
                case "West":
                    canvas.drawLine(
                            cells[col + 1][19 - row].startX,
                            cells[col + 1][19 - row].startY,
                            cells[col + 1][19 - row].startX,
                            cells[col + 1][19 - row].endY,
                            maroonPaint
                    );
                    break;
            }
        }
    }

    private void drawIndividualCell(Canvas canvas) {
        for (int x = 1; x <= COL; x++)
            for (int y = 0; y < ROW; y++)
                // if cell[x][y] is a non-image cell
                if (!cells[x][y].type.equals("image") && cells[x][y].getId() == -1) {
                    canvas.drawRect(
                            cells[x][y].startX,
                            cells[x][y].startY,
                            cells[x][y].endX,
                            cells[x][y].endY,
                            cells[x][y].paint
                    );
                } else {
                    canvas.drawRect(
                            cells[x][y].startX,
                            cells[x][y].startY,
                            cells[x][y].endX,
                            cells[x][y].endY,
                            cells[x][y].paint
                    );
                }
    }

    private void drawHorizontalLines(Canvas canvas) {
        for (int y = 0; y <= ROW; y++)
            canvas.drawLine(
                    cells[1][y].startX,
                    cells[1][y].startY - (cellSize / 30),
                    cells[20][y].endX,
                    cells[20][y].startY - (cellSize / 30),
                    whitePaint
            );
    }

    private void drawVerticalLines(Canvas canvas) {
        for (int x = 0; x <= COL; x++)
            canvas.drawLine(
                    cells[x][0].startX - (cellSize / 30) + cellSize,
                    cells[x][0].startY - (cellSize / 30),
                    cells[x][0].startX - (cellSize / 30) + cellSize,
                    cells[x][19].endY + (cellSize / 30),
                    whitePaint
            );
    }

    // Draw the axis numbers
    private void drawGridNumber(Canvas canvas) {
        for (int x = 1; x <= COL; x++) {
            if (x > 10)
                canvas.drawText(
                        Integer.toString(x - 1),
                        cells[x][20].startX + (cellSize / 5),
                        cells[x][20].startY + (cellSize / 1.5f),
                        blackPaint
                );
            else
                canvas.drawText(
                        Integer.toString(x - 1),
                        cells[x][20].startX + (cellSize / 3),
                        cells[x][20].startY + (cellSize / 1.5f),
                        blackPaint
                );
        }
        for (int y = 0; y < ROW; y++) {
            if ((20 - y) > 10)
                canvas.drawText(
                        Integer.toString(ROW - 1 - y),
                        cells[0][y].startX + (cellSize / 4),
                        cells[0][y].startY + (cellSize / 1.5f),
                        blackPaint
                );
            else
                canvas.drawText(
                        Integer.toString(ROW - 1 - y),
                        cells[0][y].startX + (cellSize / 2.5f),
                        cells[0][y].startY + (cellSize / 1.5f),
                        blackPaint
                );
        }
    }

    public ArrayList<int[]> getObstaclesList() {
        return obstacleCoord;
    }

    private void drawRobot(Canvas canvas, int[] curCoord) {

        float xCoord, yCoord;
        BitmapFactory.Options op = new BitmapFactory.Options();
        Bitmap bm, mapscalable;

        int androidRowCoord = curCoord[1];

        try{
            for (int y = androidRowCoord - 2; y <= androidRowCoord; y++) {
                canvas.drawLine(
                        cells[curCoord[0] - 2][19 - y].startX,
                        cells[curCoord[0] - 2][19 - y].startY,
                        cells[curCoord[0]][19 - y].endX,
                        cells[curCoord[0] - 2][19 - y].startY,
                        robotColor
                );
            }
            // vertical lines
            for (int x = curCoord[0] - 2; x <= curCoord[0]; x++) {
                canvas.drawLine(
                        cells[x][19 - androidRowCoord].startX,
                        cells[x][19 - androidRowCoord].startY,
                        cells[x][19 - (androidRowCoord - 2)].endX,
                        cells[x][19 - (androidRowCoord - 2)].startY,
                        robotColor
                );

            // use cells[initialCol][20 - initialRow] as ref
            switch (this.getRobotDirection()) {
                case "up":
                    //This makes the coordinates adjustable instead of static
                    op.inMutable = true;
                    //change icon pic
                    bm = BitmapFactory.decodeResource(getResources(), R.drawable.move_forward, op);

                    mapscalable = Bitmap.createScaledBitmap(bm, 51, 51, true);
                    xCoord = cells[curCoord[0] - 1][20 - androidRowCoord].startX;
                    yCoord = cells[curCoord[0]][20 - androidRowCoord - 1].startY;
                    canvas.drawBitmap(mapscalable, xCoord, yCoord, null);
                    break;
                case "down":
                    op.inMutable = true;
                    bm = BitmapFactory.decodeResource(getResources(), R.drawable.move_back, op);
                    mapscalable = Bitmap.createScaledBitmap(bm, 51, 51, true);
                    xCoord = cells[curCoord[0] - 1][20 - androidRowCoord].startX;
                    yCoord = cells[curCoord[0]][20 - androidRowCoord - 1].startY;
                    canvas.drawBitmap(mapscalable, xCoord, yCoord, null);
                    break;
                case "right":
                    op.inMutable = true;
                    bm = BitmapFactory.decodeResource(getResources(), R.drawable.move_right, op);
                    mapscalable = Bitmap.createScaledBitmap(bm, 51, 51, true);
                    xCoord = cells[curCoord[0] - 1][20 - androidRowCoord].startX;
                    yCoord = cells[curCoord[0]][20 - androidRowCoord - 1].startY;
                    canvas.drawBitmap(mapscalable, xCoord, yCoord, null);

                    break;
                case "left":
                    op.inMutable = true;
                    bm = BitmapFactory.decodeResource(getResources(), R.drawable.move_left, op);
                    mapscalable = Bitmap.createScaledBitmap(bm, 51, 51, true);
                    xCoord = cells[curCoord[0] - 1][20 - androidRowCoord].startX;
                    yCoord = cells[curCoord[0]][20 - androidRowCoord - 1].startY;
                    canvas.drawBitmap(mapscalable, xCoord, yCoord, null);
                    break;
                default:
                    Toast.makeText(
                            this.getContext(),
                            "Error with drawing robot (unknown direction)",
                            Toast.LENGTH_SHORT
                    ).show();
                    break;
            }
        }}
            catch (ArrayIndexOutOfBoundsException e){
                showLog("ArrayIndexOutOfBoundsException");
            }
    }

    private void createCell() {
        cells = new Cell[COL + 1][ROW + 1];
        this.calculateDimension();
        cellSize = this.getCellSize();

        for (int x = 0; x <= COL; x++)
            for (int y = 0; y <= ROW; y++)
                cells[x][y] = new Cell(
                        x * cellSize + (cellSize / 30),
                        y * cellSize + (cellSize / 30),
                        (x + 1) * cellSize,
                        (y + 1) * cellSize,
                        unexploredColor,
                        "unexplored"
                );
    }

    // receives col and row values that are just +1 of the visual col and row value (x & y)
    public void setStartCoord(int col, int row) {
        String dir;
        int x, y;
        startCoord[0] = col;
        startCoord[1] = row;
        String direction = getRobotDirection();
        if (direction.equals("None")) {
            direction = "up";
        }
        if (this.getStartCoordStatus())
            this.setCurCoord(col, row, direction);

        dir = (direction.equals("up")) ? "NORTH" : (direction.equals("down")) ? "SOUTH" : (direction.equals("left")) ? "WEST" : "EAST";

        if ((col - 2) >= 0 && (row - 1) >= 0) {
            Home.printMessage("ROBOT" + "," + (col - 2) + "," + (row - 1) + "," + dir.toUpperCase());
        } else {
            showLog("out of grid");
        }
    }



    public void setCurCoord(int col, int row, String direction) {

        // although rows are from 0 to 19, if the row value given is 0 or > 19, the robot will have to be in an invalid position
        if (row < 1 || row > 19) {
            showLog("y is out of bounds");
            return;
        }
        // although cols are from 1 to 20, if the col value given is 1 or > 20, the robot will have to be in an invalid position
        if (col < 2 || col > 20) {
            showLog("x is out of bounds");
            return;
        }

        curCoord[0] = col;
        curCoord[1] = row;
        this.setRobotDirection(direction);
        this.updateRobotAxis(col, row, direction);

        row = this.convertRow(row);

        try {

            for (int x = col - 2; x <= col; x++)
                for (int y = row - 2; y <= row; y++)
                    cells[x][y].setType("robot");

        } catch (ArrayIndexOutOfBoundsException e) {
            showLog("Array out of bounds");

        }
    }

    public void setCurCoord2(int col, int row) {

        // although rows are from 0 to 19, if the row value given is 0 or > 19, the robot will have to be in an invalid position
        if (row < 1 || row > 19) {
            showLog("y is out of bounds");
            return;
        }
        // although cols are from 1 to 20, if the col value given is 1 or > 20, the robot will have to be in an invalid position
        if (col < 2 || col > 20) {
            showLog("x is out of bounds");
            return;
        }

        curCoord[0] = col;
        curCoord[1] = row;

        row = this.convertRow(row);
        for (int x = col - 2; x <= col; x++)
            for (int y = row - 2; y <= row; y++) {
                cells[x][y].setType("robot");

            }
    }

    private void setOldRobotCoord(int oldCol, int oldRow) {
        showLog("Entering setOldRobotCoord");
        oldCoord[0] = oldCol;
        oldCoord[1] = oldRow;
        oldRow = this.convertRow(oldRow);

        if (oldRow == 0) {
            showLog("oldRow has gone out of grid.");
            return;
        }
        try{
            for (int x = oldCol - 2; x <= oldCol; x++)
                for (int y = oldRow - 2; y <= oldRow; y++)
                    cells[x][y].setType("explored");
        }
        catch (ArrayIndexOutOfBoundsException e)
        {
            showLog("ArrayOutOfBounds");
        }

    }

    private int[] getOldRobotCoord() {
        return oldCoord;
    }

    public void setRobotDirection(String direction) {
        sharedPrefs = getContext().getSharedPreferences("Shared Preferences",
                Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = sharedPrefs.edit();
        robotDirection = direction;
        editor.putString("direction", direction);
        editor.apply();
        this.invalidate();
    }

    public void updateRobotAxis(int col, int row, String direction) {
        TextView xAxisTextView = ((Activity) this.getContext()).findViewById(R.id.xAxisTextView);
        TextView yAxisTextView = ((Activity) this.getContext()).findViewById(R.id.yAxisTextView);
        TextView directionAxisTextView = ((Activity) this.getContext())
                .findViewById(R.id.directionAxisTextView);

        xAxisTextView.setText(String.valueOf(col - 1));
        yAxisTextView.setText(String.valueOf(row - 1));
        directionAxisTextView.setText(direction);
    }

    public void setObstacleCoord(int col, int row, boolean isLoaded) {

        int[] obstacleCoord = new int[]{col - 1, row - 1};

        GridMap.obstacleCoord.add(obstacleCoord);

        row = this.convertRow(row);
        cells[col][row].setType("obstacle");

        int obstacleNumber = GridMap.obstacleCoord.size();

        if (((col - 1)) >= 0 && row >= 0) {
            String message = "OBSTACLE" + "," + obstacleNumber + "," + (col - 1) + "," + (19 - row) + "," + (imageBearings.get(19 - row)[col - 1]).toUpperCase();

            if (isLoaded) {
                message += ",Z";
            }

            Home.printMessage(message + "\n");
        } else {
            showLog("out of grid");
        }
    }

    private ArrayList<int[]> getObstacleCoord() {
        return obstacleCoord;
    }

    private static void showLog(String message) {
        Log.d(TAG, message);
    }

    private class Cell {
        float startX, startY, endX, endY;
        Paint paint;
        String type;
        int id = -1;

        private Cell(float startX, float startY, float endX, float endY, Paint paint, String type) {
            this.startX = startX;
            this.startY = startY;
            this.endX = endX;
            this.endY = endY;
            this.paint = paint;
            this.type = type;
        }

        public void setType(String type) {
            this.type = type;
            switch (type) {
                case "image":
                    this.paint = imageColor;
                    break;
                case "obstacle":
                    this.paint = obstacleColor;
                    break;
                case "robot":
                    this.paint = robotColor;
                    break;
                case "end":
                    this.paint = endColor;
                    break;
                case "start":
                    this.paint = startColor;
                    break;
                case "waypoint":
                    this.paint = waypointColor;
                    break;
                case "unexplored":
                    this.paint = unexploredColor;
                    break;
                case "explored":
                    this.paint = exploredColor;
                    break;
                case "arrow":
                    this.paint = arrowColor;
                    break;
                case "fastestPath":
                    this.paint = fastestPathColor;
                    break;
                default:
                    showLog("setType default: " + type);
                    break;
            }
        }

        public void setId(int id) {
            this.id = id;
        }

        public int getId() {
            return this.id;
        }
    }

    int endColumn, endRow;
    String oldItem = "";

    // drag event to move obstacle
    @Override
    public boolean onDragEvent(DragEvent dragEvent) {
        clipData = dragEvent.getClipData();
        localState = dragEvent.getLocalState();

        String tempID, tempBearing, testID;
        endColumn = endRow = -999;

        int obstacleid = -1;

        // drag and drop out of gridmap
        if ((dragEvent.getAction() == DragEvent.ACTION_DRAG_ENDED)
                && (endColumn == -999 || endRow == -999) && !dragEvent.getResult()) {
            // check if 2 arrays are same, then remove
            int obstacleid3 = -1;
            for (int i = 0; i < obstacleCoord.size(); i++) {
                if (Arrays.equals(obstacleCoord.get(i), new int[]{initialColumn - 1, initialRow - 1})) {
                    obstacleCoord.remove(i);
                    obstacleid3 = i;
                }
            }
            cells[initialColumn][20 - initialRow].setType("unexplored");
            ITEM_LIST.get(initialRow - 1)[initialColumn - 1] = "";
            imageBearings.get(initialRow - 1)[initialColumn - 1] = "";

            if (((initialColumn - 1)) >= 0 && ((initialRow - 1)) >= 0) {
                Home.printMessage("OBSTACLE" + "," + (obstacleid3 + 1) + "," + (initialColumn) + "," + (initialRow) + "," + "-1");
            } else {
                showLog("out of grid");
            }

        }
        // drop within gridmap
        else if (dragEvent.getAction() == DragEvent.ACTION_DROP) {
            endColumn = (int) (dragEvent.getX() / cellSize);
            endRow = this.convertRow((int) (dragEvent.getY() / cellSize));

            // if the currently dragged cell is empty, do nothing
            if (ITEM_LIST.get(initialRow - 1)[initialColumn - 1].equals("")
                    && imageBearings.get(initialRow - 1)[initialColumn - 1].equals("")) {
                showLog("Cell is empty");
            }

            // if dropped within mapview but outside drawn grids, remove obstacle from lists
            // drag to left side of grid
            else if (endColumn <= 0 || endRow <= 0) {
                int obstacleid2 = -1;
                for (int i = 0; i < obstacleCoord.size(); i++) {
                    if (Arrays.equals(obstacleCoord.get(i),
                            new int[]{initialColumn - 1, initialRow - 1})) {
                        obstacleCoord.remove(i);
                        obstacleid2 = i;
                    }

                }
                cells[initialColumn][20 - initialRow].setType("unexplored");
                ITEM_LIST.get(initialRow - 1)[initialColumn - 1] = "";
                imageBearings.get(initialRow - 1)[initialColumn - 1] = "";

                if (((initialColumn - 1)) >= 0 && ((initialRow - 1)) >= 0) {
                    Home.printMessage("OBSTACLE" + "," + (obstacleid2 + 1) + "," + (initialColumn) + "," + (initialRow) + "," + "-1");
                }
                else {
                    showLog("out of grid");
                }

            }
            // if dropped within gridmap, shift it to new position unless already got existing
            else if ((1 <= initialColumn && initialColumn <= 20)
                    && (1 <= initialRow && initialRow <= 20)
                    && (1 <= endColumn && endColumn <= 20)
                    && (1 <= endRow && endRow <= 20)) {
                tempID = ITEM_LIST.get(initialRow - 1)[initialColumn - 1];
                tempBearing = imageBearings.get(initialRow - 1)[initialColumn - 1];

                // check if got existing obstacle at drop location
                if (!ITEM_LIST.get(endRow - 1)[endColumn - 1].equals("")
                        || !imageBearings.get(endRow - 1)[endColumn - 1].equals("")) {
                    showLog("An obstacle is already at drop location");
                }
                else {
                    ITEM_LIST.get(initialRow - 1)[initialColumn - 1] = "";
                    imageBearings.get(initialRow - 1)[initialColumn - 1] = "";
                    ITEM_LIST.get(endRow - 1)[endColumn - 1] = tempID;
                    imageBearings.get(endRow - 1)[endColumn - 1] = tempBearing;
                    for (int i = 0; i < obstacleCoord.size(); i++) {
                        if (Arrays.equals(obstacleCoord.get(i), new int[]{initialColumn - 1, initialRow - 1})) {
                            obstacleCoord.set(i, new int[]{endColumn - 1, endRow - 1});
                            obstacleid = i;
                        }
                    }
                    cells[endColumn][20 - endRow].setType(cells[initialColumn][20 - initialRow].type);
                    cells[initialColumn][20 - initialRow].setType("unexplored");

                    if (((endColumn - 1)) >= 0 && ((endRow - 1)) >= 0) {
                        Home.printMessage("OBSTACLE" + "," + (obstacleid + 1) + "," + (endColumn - 1) + "," + (endRow - 1) + "," + tempBearing.toUpperCase());
                    } else {
                        showLog("out of grid");
                    }

                }
            } else {
                showLog("Drag event failed.");
            }
        }
        this.invalidate();
        return true;
    }

    public void callInvalidate() {
        this.invalidate();
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        if (event.getAction() == MotionEvent.ACTION_DOWN) {
            // column and row values are BASED ON THE DISPLAYED MAP (but also +1 as it is not 0-indexed)
            int column = (int) (event.getX() / cellSize);
            int row = this.convertRow((int) (event.getY() / cellSize));

            initialColumn = column;
            initialRow = row;

            ToggleButton setStartPointToggleBtn = ((Activity) this.getContext())
                    .findViewById(R.id.startpointToggleBtn);

            try {
                oldItem = ITEM_LIST.get(initialRow - 1)[initialColumn - 1];
            } catch (IndexOutOfBoundsException e) {
                System.out.println("Invalid index!");
                e.printStackTrace();
            }

            // start drag
            if (MappingFragment.dragStatus) {
                if (!((1 <= initialColumn && initialColumn <= 20)
                        && (1 <= initialRow && initialRow <= 20))) {
                    return false;
                } else if (ITEM_LIST.get(row - 1)[column - 1].equals("")
                        && imageBearings.get(row - 1)[column - 1].equals("")) {
                    return false;
                }
                View.DragShadowBuilder dragShadowBuilder = new MyDragShadowBuilder(this);
                this.startDrag(null, dragShadowBuilder, null, 0);
            }

            // start change obstacle
            if (MappingFragment.changeObstacleStatus) {
                if (!((1 <= initialColumn && initialColumn <= 20)
                        && (1 <= initialRow && initialRow <= 20))) {
                    return false;
                } else if (ITEM_LIST.get(row - 1)[column - 1].equals("")
                        && imageBearings.get(row - 1)[column - 1].equals("")) {
                    return false;
                } else {
                    String imageId = ITEM_LIST.get(row - 1)[column - 1];
                    String imageBearing = imageBearings.get(row - 1)[column - 1];
                    final int tRow = row;
                    final int tCol = column;

                    AlertDialog.Builder mBuilder = new AlertDialog.Builder(this.getContext());
                    View mView = ((Activity) this.getContext()).getLayoutInflater()
                            .inflate(R.layout.activity_dialog,
                                    null);
                    mBuilder.setTitle("Change Existing Bearing");
                    final Spinner mBearingSpinner = mView.findViewById(R.id.bearingSpinner2);


                    ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(
                            this.getContext(), R.array.imageID_array,
                            android.R.layout.simple_spinner_item);
                    adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
                    ArrayAdapter<CharSequence> adapter2 = ArrayAdapter.createFromResource(
                            this.getContext(), R.array.imageBearing_array,
                            android.R.layout.simple_spinner_item);
                    adapter2.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
                    mBearingSpinner.setAdapter(adapter2);

                    switch (imageBearing) {
                        case "North":
                            mBearingSpinner.setSelection(0);
                            break;
                        case "South":
                            mBearingSpinner.setSelection(1);
                            break;
                        case "East":
                            mBearingSpinner.setSelection(2);
                            break;
                        case "West":
                            mBearingSpinner.setSelection(3);
                    }

                    // do what when user presses ok
                    mBuilder.setPositiveButton("OK", new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialogInterface, int i) {
                            String newBearing = mBearingSpinner.getSelectedItem().toString();
                            imageBearings.get(tRow - 1)[tCol - 1] = newBearing;

                            int obstacleid = -1;
                            for (int m = 0; m < obstacleCoord.size(); m++) {
                                if (Arrays.equals(obstacleCoord.get(m), new int[]{tCol - 1, tRow - 1})) {
                                    obstacleid = m;
                                }
                            }

                            String oldObstacleId = UUID.randomUUID().toString();
                            if (val2IdxMap.containsKey(oldItem)) {
                                oldObstacleId = val2IdxMap.get(oldItem);
                            }

                            else cells[tCol][20 - tRow].setType("obstacle");
                            int obstacleNumber = GridMap.obstacleCoord.size();

                            if (((tCol - 1)) >= 0 && ((tRow - 1)) >= 0) {
                                Home.printMessage("OBSTACLE" + "," + (obstacleid + 1) + "," + (tCol - 1) + "," + (tRow - 1) + "," + newBearing.toUpperCase());
                            }
                            else {
                                showLog("out of grid");
                            }
                            callInvalidate();
                        }
                    });

                    // dismiss
                    mBuilder.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialogInterface, int i) {
                            dialogInterface.dismiss();
                        }
                    });

                    mBuilder.setView(mView);
                    AlertDialog dialog = mBuilder.create();
                    dialog.show();
                    Window window = dialog.getWindow();
                    WindowManager.LayoutParams layoutParams = new WindowManager.LayoutParams();
                    layoutParams.width = 150;
                    window.setLayout(ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
                }
            }

            // change robot size and make sure its within the grid
            if (startCoordStatus) {
                if (canDrawRobot) {
                    // removes green grids when user changes robot startpoint
                    for (int i = 0; i < 21; i++) {
                        for (int j = 0; j < 21; j++) {
                            if (cells[i][j].type.equals("robot")) {
                                cells[i][j].setType("unexplored");
                            }
                        }
                    }
                    // don't set robot if obstacles are there
                    int[] startCoord = this.getStartCoord();

                    if (startCoord[0] >= 2 && startCoord[1] >= 2) {
                        showLog("startCoord = " + startCoord[0] + " " + startCoord[1]);
                        for (int x = startCoord[0] - 1; x <= startCoord[0]; x++)
                            for (int y = startCoord[1] - 1; y <= startCoord[1]; y++)
                                cells[x][y].setType("unexplored");
                    }
                } else
                    canDrawRobot = true;

                this.setStartCoord(column, row);
                startCoordStatus = false;
                String direction = getRobotDirection();
                if (direction.equals("None")) {
                    direction = "up";
                }
                try {
                    int directionInt = 0;
                    switch (direction) {
                        case "up":
                            directionInt = 0;
                            break;
                        case "left":
                            directionInt = 3;
                            break;
                        case "right":
                            directionInt = 1;
                            break;
                        case "down":
                            directionInt = 2;
                            break;
                    }

                } catch (Exception e) {
                    e.printStackTrace();
                }
                updateRobotAxis(column, row, direction);
                if (setStartPointToggleBtn.isChecked()) {
                    setStartPointToggleBtn.toggle();
                    setStartPointToggleBtn.setBackgroundResource(R.drawable.border_black);
                }
                this.invalidate();
                return true;
            }

            // place obstacles on map
            if (setObstacleStatus) {
                if ((1 <= row && row <= 20) && (1 <= column && column <= 20)) { // if touch is within the grid

                    if (!ITEM_LIST.get(row - 1)[column - 1].equals("")
                            || !imageBearings.get(row - 1)[column - 1].equals("")) {
                        showLog("An obstacle is already at drop location");
                    } else {
                        // get user input from spinners in MapTabFragment static values
                        // imageID is "", imageBearing is "North"
                        String imageID = (MappingFragment.imageID).equals("Nil") ?
                                "" : MappingFragment.imageID;
                        String imageBearing = MappingFragment.imageBearing;

                        // after init, at stated col and row, add the id to use as ref to update grid
                        ITEM_LIST.get(row - 1)[column - 1] = imageID;
                        imageBearings.get(row - 1)[column - 1] = imageBearing;

                        // this function affects obstacle turning too
                        this.setObstacleCoord(column, row, false);
                    }
                }
                this.invalidate();
                return true;
            }
            if (setExploredStatus) {
                cells[column][20 - row].setType("explored");
                this.invalidate();
                return true;
            }

            // added removing imageID and imageBearing
            if (unSetCellStatus) {
                ArrayList<int[]> obstacleCoord = this.getObstacleCoord();
                cells[column][20 - row].setType("unexplored");
                for (int i = 0; i < obstacleCoord.size(); i++)
                    if (obstacleCoord.get(i)[0] == column && obstacleCoord.get(i)[1] == row)
                        obstacleCoord.remove(i);
                ITEM_LIST.get(row)[column - 1] = "";  // remove imageID
                imageBearings.get(row)[column - 1] = "";  // remove bearing
                this.invalidate();
                return true;
            }
        }
        return false;
    }

    public void toggleCheckedBtn(String buttonName) {
        ToggleButton setStartPointToggleBtn = ((Activity) this.getContext())
                .findViewById(R.id.startpointToggleBtn);
        ImageButton obstacleImageBtn = ((Activity) this.getContext())
                .findViewById(R.id.addObstacleBtn);

        if (!buttonName.equals("setStartPointToggleBtn"))
            if (setStartPointToggleBtn.isChecked()) {
                this.setStartCoordStatus(false);
                setStartPointToggleBtn.toggle();
                setStartPointToggleBtn.setBackgroundResource(R.drawable.border_black);
            }
        if (!buttonName.equals("obstacleImageBtn"))
            if (obstacleImageBtn.isEnabled()) {
                this.setSetObstacleStatus(false);
                obstacleImageBtn.setBackgroundResource(R.drawable.border_black);
            }
    }

    public void resetMap() {
        TextView robotStatusTextView = ((Activity) this.getContext())
                .findViewById(R.id.robotStatus);
        updateRobotAxis(1, 1, "None");
        robotStatusTextView.setText("Not Available");


        this.toggleCheckedBtn("None");

        startCoord = new int[]{-1, -1};
        curCoord = new int[]{-1, -1};
        oldCoord = new int[]{-1, -1};
        robotDirection = "None";
        obstacleCoord = new ArrayList<>();
        mapDrawn = false;
        canDrawRobot = false;
        validPosition = false;

        for (int i = 0; i < 20; i++) {
            for (int j = 0; j < 20; j++) {
                ITEM_LIST.get(i)[j] = "";
                imageBearings.get(i)[j] = "";
            }
        }
        this.invalidate();
    }

    // e.g obstacle is on right side of 2x2 and can turn left and vice versa
    public void moveRobot(String direction) {
        setValidPosition(false);
        int[] curCoord = this.getCurCoord();

        ArrayList<int[]> obstacleCoord = this.getObstacleCoord();
        this.setOldRobotCoord(curCoord[0], curCoord[1]);
        int[] oldCoord = this.getOldRobotCoord();
        String robotDirection = getRobotDirection();
        String backupDirection = robotDirection;

        Integer[] newCoords = Arrays.stream(this.getCurCoord()).boxed().toArray( Integer[]::new );

        Map.Entry<String, ArrayList<Integer[]>> entry;
        switch (direction) {
            case "forward":
            case "back": {
                Integer[] last = Straight.straight(newCoords, robotDirection, direction);
                if (last[0] < 2 || last[1] < 1 || 21 <= last[0] || 19 <= last[1]) {
                    break;
                }

                curCoord[0] = last[0];
                curCoord[1] = last[1];
                showLog("Forward backward coords x: " + curCoord[0] + " y: " + curCoord[1]);
                cells[curCoord[0]][20 - curCoord[1]].setType("explored");
                validPosition = true;
            }
                break;

            case "left": {
                entry = Turn.turn(newCoords,robotDirection,"left");
                Integer[] last = entry.getValue().get(entry.getValue().size() - 1);
                if (last[0] < 2 || last[1] < 1 || 20 <= last[0] || 19 <= last[1]) {
                    break;
                }
                curCoord[0] = last[0];
                curCoord[1] = last[1];
                cells[curCoord[0]][20 - curCoord[1]].setType("explored");
                robotDirection = entry.getKey();
                validPosition = true;
            }
                break;

            case "right": {
                entry = Turn.turn(newCoords,robotDirection,"right");
                Integer[] last = entry.getValue().get(entry.getValue().size() - 1);
                if (last[0] < 2 || last[1] < 1 || 20 <= last[0] || 20 <= last[1]) {
                    break;
                }
                curCoord[0] = last[0];
                curCoord[1] = last[1];
                cells[curCoord[0]][20 - curCoord[1]].setType("explored");
                robotDirection = entry.getKey();
                validPosition = true;
            }
                break;

            // testing new direction of movement (facing forward)
            case "backleft": {
                entry = Turn.turn(newCoords,robotDirection,"backleft");
                Integer[] last = entry.getValue().get(entry.getValue().size() - 1);
                if (last[0] < 2 || last[1] < 1 || 20 <= last[0] || 20 <= last[1]) {
                    break;
                }
                curCoord[0] = last[0];
                curCoord[1] = last[1];
                cells[curCoord[0]][20 - curCoord[1]].setType("explored");
                robotDirection = entry.getKey();
                validPosition = true;
            }
                break;

            case "backright": {
                entry = Turn.turn(newCoords,robotDirection,"backright");
                Integer[] last = entry.getValue().get(entry.getValue().size() - 1);
                if (last[0] < 2 || last[1] < 1 || 20 <= last[0] || 20 <= last[1]) {
                    break;
                }
                curCoord[0] = last[0];
                curCoord[1] = last[1];
                cells[curCoord[0]][20 - curCoord[1]].setType("explored");
                robotDirection = entry.getKey();
                validPosition = true;
            }
                break;
            default:
                robotDirection = "error up";
                break;
        }

        if (getValidPosition())
            // check obstacle for new position
            for (int x = curCoord[0] - 1; x <= curCoord[0] + 1; x++) {
                for (int y = curCoord[1] - 1; y <= curCoord[1] + 1; y++) {
                    for (int i = 0; i < obstacleCoord.size(); i++) {
                        if (obstacleCoord.get(i)[0] == (x - 2)  && obstacleCoord.get(i)[1] == y) { // HERE x
                            setValidPosition(false);
                            robotDirection = backupDirection;
                            break;
                        }
                    }
                    if (!getValidPosition())
                        break;
                }
                if (!getValidPosition())
                    break;
            }
        if (getValidPosition())
            this.setCurCoord(curCoord[0], curCoord[1], robotDirection);
        else {
            if (direction.equals("forward") || direction.equals("back"))
                robotDirection = backupDirection;
            this.setCurCoord(oldCoord[0], oldCoord[1], robotDirection);
        }
        this.invalidate();
    }


    public void moveRobotCoords(int x, int y) {

        // Set valid position to false initially
        setValidPosition(false);

        // Get current coordinates
        int[] curCoord = this.getCurCoord();

        // Get obstacle coordinates
        ArrayList<int[]> obstacleCoord = this.getObstacleCoord();

        // Save the old coordinates
        this.setOldRobotCoord(curCoord[0], curCoord[1]);
        int[] oldCoord = this.getOldRobotCoord();

        // Check if the new position (x, y) is valid
        if (isValidMove(x, y, obstacleCoord)) {
            // Update the robot's coordinates if valid
            curCoord[0] = x;
            curCoord[1] = y;

            this.setCurCoord2(curCoord[0], curCoord[1]);
            setValidPosition(true); // Mark the position as valid
            this.invalidate();
        }
        else {
            // If the move is invalid, revert to the old coordinates
            this.setCurCoord2(oldCoord[0], oldCoord[1]);
        }
    }

    // Helper method to check if the move is valid (no obstacle)
    private boolean isValidMove(int x, int y, ArrayList<int[]> obstacleCoord) {
        // Check if the position is within bounds
        if (x < 0 || y < 0 || x >= 21 || y >= 21) {
            showLog("Move out of bounds.");
            return false;
        }

        // Check if there's an obstacle at the new position
        for (int[] obstacle : obstacleCoord) {
            if (obstacle[0] == x && obstacle[1] == y) {
                showLog("Obstacle detected at: (" + x + ", " + y + ")");
                return false;
            }
        }

        return true;
    }

    private static class MyDragShadowBuilder extends View.DragShadowBuilder {
        private Point mScaleFactor;

        // Defines the constructor for myDragShadowBuilder
        public MyDragShadowBuilder(View v) {
            super(v);
        }

        // Defines a callback that sends the drag shadow dimensions and touch point back to the system.
        @Override
        public void onProvideShadowMetrics (Point size, Point touch) {
            // Defines local variables
            int width;
            int height;

            // Sets the width of the shadow to half the width of the original View
            width = (int) (cells[1][1].endX - cells[1][1].startX);

            // Sets the height of the shadow to half the height of the original View
            height = (int) (cells[1][1].endY - cells[1][1].startY);

            // Sets the size parameter's width and height values. These get back to the system through the size parameter.
            size.set(width, height);
            // Sets size parameter to member that will be used for scaling shadow image.
            mScaleFactor = size;

            // Sets the touch point's position to be in the middle of the drag shadow
            touch.set(width / 2, height / 2);
        }

        @Override
        public void onDrawShadow(Canvas canvas) {
            // Draws the ColorDrawable in the Canvas passed in from the system.
            canvas.scale(mScaleFactor.x/(float)getView().getWidth(),
                    mScaleFactor.y/(float)getView().getHeight());
            getView().draw(canvas);
        }
    }

    // Modified for algo side to denote each obstacle as: (algoX, algoY, algoDirection, algoObsId)
    // 2nd half of getObstacles()
    public String translateCoord(String msg){
        String translatedMsg = "";
        // split msg by '|'
        String[] msgSections = msg.split("\\|");
        for(int i = 1; i < msgSections.length; i++) {   // ignore 1st sub string since its "ALG"
            String[] msgSubSections = msgSections[i].split(",");
            // algoX and algoY are 'related' to (x, y) coordinates on a 0-indexed grid, e.g. (10, 7) in (x, y) = (105, 75) in (algoX, algoY)
            int algoX = Integer.parseInt(msgSubSections[0]) * 10 + 5;
            int algoY = Integer.parseInt(msgSubSections[1]) * 10 + 5;
            // algoDirection is a mapping of 4 values for each direction: North = 90, East = 0, South = -90, West = 180
            int algoDirection;
            switch(msgSubSections[2].charAt(0)) {
                case 'N':
                    algoDirection = 90;
                    break;
                case 'S':
                    algoDirection = -90;
                    break;
                case 'E':
                    algoDirection = 0;
                    break;
                case 'W':
                    algoDirection = 180;
                    break;
                default:
                    algoDirection = -1;
            }
            // algo_obs_id is zero-index obstacle id number, probably can just use a for loop w/ i < obstacleCoord.size()? Assuming that it doesn't affect the algo
            int algoObsId = Integer.parseInt(msgSubSections[3]);
            translatedMsg += algoX + "," + algoY + "," + algoDirection + "," + algoObsId;
            if(i < msgSections.length - 1) translatedMsg += "|";  // add separator for all obstacles except the last
        }
        // The '_' is just a special character to denote the position to split this resulting string later on
        return msg + "_ALG|" + translatedMsg;
    }

    public static String saveObstacleList(){    // used for the save/load map functionality
        String message ="";
        for (int i = 0; i < obstacleCoord.size(); i++) {
            message += ((obstacleCoord.get(i)[0]) + "," // add x coordinate of obstacle
                    + (obstacleCoord.get(i)[1]) + ","   // add y coordinate of obstacle
                    + imageBearings.get(obstacleCoord.get(i)[1])[obstacleCoord.get(i)[0]].charAt(0));  // add the 1st letter of the direction


            if(i < obstacleCoord.size() - 1) message += "\n";    // add a "|" to the end of each obstacle's info (except for the last)
        }
        BluetoothCommunications.getMessageReceivedTextView().append(message+"\n");

        return message;

    }

    // Returns a string that contains 2 'substrings' of the obstacle information, 1 untranslated & 1 translated, separated by '_'
    // Currently it will send all the coordinates regardless of whether the 'obstacle' is an obstacle or image - but images have '-1' as obstacle id
    public String getObstacles() {
        String msg = "ALG|";
        int obstId = 0;
        for (int i = 0; i < obstacleCoord.size(); i++) {
            // check if its an obstacle or an image
            int col = obstacleCoord.get(i)[0];
            int row = obstacleCoord.get(i)[1];
            // Additional redundant logic to allow for position of obstacles w/o images to also be sent over
            msg += (col + ","     // x
                    + row + ","   // y
                    + imageBearings.get(obstacleCoord.get(i)[1])[obstacleCoord.get(i)[0]].charAt(0) + ",");   // direction
            if(ITEM_LIST.get(row)[col] == null || ITEM_LIST.get(row)[col].equals("") || ITEM_LIST.get(row)[col].equals("Nil")) { // ITEM_LIST value is null, but in obstacleCoord => obstacle
                msg += obstId++;   // obstacle id
            } else { // ITEM_LIST not empty, but in obstacleCoord => image (or blank obstacle)
                msg += (-1);    // non-obstacle id
            }
            if (i < obstacleCoord.size() - 1) msg += "|";
        }

        // Add the translated message to msg
        msg = translateCoord(msg);

        return msg;
    }

    // Updating the obstacle image id (sent over by RPi)
    public boolean updateIDFromRpi(String obstacleID, String imageID) {
        int x = obstacleCoord.get(Integer.parseInt(obstacleID))[0];
        int y = obstacleCoord.get(Integer.parseInt(obstacleID))[1];
        ITEM_LIST.get(y)[x] = (imageID.equals("-1")) ? "NA" : imageID;
        this.invalidate();
        return true;
    }

}