package com.example.mdp_group_11;

import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.graphics.Color;
import android.util.Log;
import android.widget.TextView;

import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.Charset;
import java.util.UUID;

public class BluetoothConnectionService {
    private static final String TAG = "Debugging Tag";
    private static final String appName = "MDP Group 11";
    private static final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    private final BluetoothAdapter mBluetoothAdapter;
    Context mContext;

    private AcceptThread mAcceptThread;

    private ConnectThread mConnectThread;
    private BluetoothDevice mmDevice;
    private UUID deviceUUID;
    ProgressDialog mProgressDialog;
    Intent connectionStatus;
    public static boolean BluetoothConnectionStatus=false;
    private static ConnectedThread mConnectedThread;

    public BluetoothConnectionService(Context context) {
        this.mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        this.mContext = context;
        startAcceptThread();
    }

    //This thread will be running while listening for an incoming connection.
    private class AcceptThread extends Thread {
        private final BluetoothServerSocket ServerSocket;

        public AcceptThread() {
            BluetoothServerSocket tmp = null;

            try {
                tmp = mBluetoothAdapter.listenUsingInsecureRfcommWithServiceRecord(appName, MY_UUID);
                Log.d(TAG, "Accept Thread: Setting up Server using: " + MY_UUID);
            } catch (IOException e) {
                Log.e(TAG, "Accept Thread: IOException: " + e.getMessage());
            }
            ServerSocket = tmp;
        }
        public void run(){
            Log.d(TAG, "run: AcceptThread Running. ");
            BluetoothSocket socket =null;
            try {
                Log.d(TAG, "run: RFCOM server socket start here...");

                socket = ServerSocket.accept();
            }catch (IOException e){
                Log.e(TAG, "run: IOException: " + e.getMessage());
            }
            if(socket!=null){
                connected(socket, socket.getRemoteDevice());
            }
            Log.i(TAG, "END AcceptThread");
        }
        public void cancel(){
            Log.d(TAG, "cancel: Cancelling AcceptThread");
            try{
                ServerSocket.close();
            } catch(IOException e){
                Log.e(TAG, "cancel: Failed to close AcceptThread ServerSocket " + e.getMessage());
            }
        }
    }

    private class ConnectThread extends Thread {
        private BluetoothSocket mmSocket;

        public ConnectThread(BluetoothDevice device, UUID uuid) {
            Log.d(TAG, "ConnectThread: started.");
            mmDevice = device;
            deviceUUID = uuid;
        }

        public void run() {
            BluetoothSocket tmp = null;
            Log.d(TAG, "RUN: mConnectThread");

            try {
                Log.d(TAG, "ConnectThread: Trying to create InsecureRfcommSocket using UUID: " + MY_UUID);
                tmp = mmDevice.createRfcommSocketToServiceRecord(deviceUUID);
            } catch (IOException e) {
                Log.e(TAG, "ConnectThread: Could not create InsecureRfcommSocket " + e.getMessage());
            }
            mmSocket = tmp;

            try {
                mmSocket.connect();

                Log.d(TAG, "RUN: ConnectThread connected.");

                connected(mmSocket, mmDevice);

            } catch (IOException e) {
                try {
                    mmSocket.close();
                    Log.d(TAG, "RUN: ConnectThread socket closed.");
                } catch (IOException e1) {
                    Log.e(TAG, "RUN: ConnectThread: Unable to close connection in socket." + e1.getMessage());
                }
                Log.d(TAG, "RUN: ConnectThread: could not connect to UUID." + MY_UUID);
                try {

                } catch (Exception z) {
                    z.printStackTrace();
                    Log.e(TAG,"error here");
                }

            }
            try {
                mProgressDialog.dismiss();
            } catch (NullPointerException e) {
                e.printStackTrace();
            }
        }

        public void cancel(){
            Log.d(TAG, "cancel: Closing Client Socket");
            try{
                mmSocket.close();
            } catch(IOException e){
                Log.e(TAG, "cancel: Failed to close ConnectThread mSocket " + e.getMessage());
            }
        }
    }

    public synchronized void startAcceptThread(){
        Log.d(TAG, "start");

        //Cancel any thread attempting to make a connection
        if(mConnectThread!=null){
            mConnectThread.cancel();
            mConnectThread=null;
        }

        //If accept thread is null we want to start a new one
        if(mAcceptThread == null){
            mAcceptThread = new AcceptThread();
            mAcceptThread.start();
        }
    }

    public void startClientThread(BluetoothDevice device, UUID uuid){
        Log.d(TAG, "startClient: Started.");
        try {
            mProgressDialog = ProgressDialog.show(mContext, "Connecting Bluetooth", "Please Wait...", true);
        } catch (Exception e) {
            Log.d(TAG, "StartClientThread Dialog show failure");
        }
        mConnectThread = new ConnectThread(device, uuid);
        mConnectThread.start();
    }

    private class ConnectedThread extends Thread{
        private final BluetoothSocket mSocket;
        private final InputStream inStream;
        private final OutputStream outStream;

        public ConnectedThread(BluetoothSocket socket) {
            Log.d(TAG, "ConnectedThread: Starting.");

            this.mSocket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            connectionStatus = new Intent("ConnectionStatus");
            connectionStatus.putExtra("Status", "connected");
            connectionStatus.putExtra("Device", mmDevice);
            LocalBroadcastManager.getInstance(mContext).sendBroadcast(connectionStatus);
            BluetoothConnectionStatus = true;

            TextView status = Home.getBluetoothStatus();
            status.setText("Connected");
            status.setTextColor(Color.GREEN);

            TextView device = Home.getConnectedDevice();
            device.setText(mmDevice.getName());

            try {
                tmpIn = mSocket.getInputStream();
                tmpOut = mSocket.getOutputStream();
            } catch (IOException e) {
                e.printStackTrace();
            }

            inStream = tmpIn;
            outStream = tmpOut;
        }

        public void run() {
            byte[] buffer = new byte[1024];
            int bytes;

            while (true) {
                try {
                    bytes = inStream.read(buffer);
                    String incomingMessage = new String(buffer, 0, bytes);
                    Log.d(TAG, "InputStream: " + incomingMessage);

                    Intent incomingMessageIntent = new Intent("incomingMessage");
                    incomingMessageIntent.putExtra("receivedMessage", incomingMessage);

                    LocalBroadcastManager.getInstance(mContext).sendBroadcast(incomingMessageIntent);
                } catch (IOException e) {
                    Log.e(TAG, "Error reading input stream. " + e.getMessage());

                    connectionStatus = new Intent("ConnectionStatus");
                    connectionStatus.putExtra("Status", "disconnected");
                    TextView status = Home.getBluetoothStatus();
                    status.setText("Disconnected");
                    status.setTextColor(Color.RED);
                    connectionStatus.putExtra("Device", mmDevice);
                    LocalBroadcastManager.getInstance(mContext).sendBroadcast(connectionStatus);
                    BluetoothConnectionStatus = false;

                    break;
                }
            }
        }
        public void write(byte[] bytes){
            String text = new String(bytes, Charset.defaultCharset());
            Log.d(TAG, "write: Writing to output stream: "+text);
            try {
                outStream.write(bytes);
            } catch (IOException e) {
                Log.e(TAG, "Error writing to output stream. "+e.getMessage());
            }
        }
    }

    private void connected(BluetoothSocket mSocket, BluetoothDevice device) {
        Log.d(TAG, "connected: Starting.");
        mmDevice =  device;
        if (mAcceptThread != null) {
            mAcceptThread.cancel();
            mAcceptThread = null;
        }

        mConnectedThread = new ConnectedThread(mSocket);
        mConnectedThread.start();
    }

    public static void write(byte[] out){
        ConnectedThread tmp;

        Log.d(TAG, "write: Write is called." );
        mConnectedThread.write(out);
    }
}
