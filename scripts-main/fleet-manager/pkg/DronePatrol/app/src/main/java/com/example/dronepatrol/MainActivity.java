package com.example.dronepatrol;

import java.util.Timer;
import java.util.TimerTask;

import android.Manifest;
import android.animation.AnimatorSet;
import android.animation.ObjectAnimator;
import android.annotation.SuppressLint;
import android.app.AlertDialog;
import android.content.ContentResolver;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.provider.Settings;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import org.eclipse.paho.android.service.MqttAndroidClient;
import org.eclipse.paho.client.mqttv3.IMqttActionListener;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.json.JSONObject;

import cn.pedant.SweetAlert.SweetAlertDialog;

public class MainActivity extends AppCompatActivity {

    public Location currentLocation = null;
    public String phoneID;
    public String droneAssigned;

    private LocationManager locationManager = null;
    private static final int REQUEST_GPS_PERMISSION = 201;
    private static final int REQUEST_READ_PHONE_STATE = 202;
    private static final int REQUEST_INTERNET = 203;
    private static final int REQUEST_NETWORK_STATE = 204;
    private static final int REQUEST_WAKE_LOCK = 205;


    private Button btnGetLocation = null;
    private ImageView droneImg = null;
    private TextView helpText;
    private TextView dronePText;
    private Button btnCancel;

    private Boolean GPS_ON = false;
    private Intent i;

    private Timer timer;

    private TimerTask sendCoordinates;

    private MqttAndroidClient client;
    private String ServerURI = "tcp://192.168.1.184:1883";

    public static int globalStop = 0;

    @SuppressLint("MissingPermission")
    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Check if the application has all necessary permissions
        checkPermissions();


        // MQTT Client initialization
        String clientID = MqttClient.generateClientId();
        client = new MqttAndroidClient(MainActivity.this.getApplicationContext(), ServerURI, clientID);

        //if you want to lock screen for always Portrait mode
        setRequestedOrientation(ActivityInfo
                .SCREEN_ORIENTATION_PORTRAIT);


        btnGetLocation = (Button) findViewById(R.id.button);
        btnCancel = (Button) findViewById(R.id.Stop);
        btnCancel.setClickable(false);
        btnCancel.setAlpha(0.0f);
        btnCancel.setScaleX(0.01f);
        btnCancel.setScaleY(0.01f);

        droneImg = (ImageView) findViewById(R.id.DroneImg);
        helpText = (TextView) findViewById(R.id.help);
        helpText.setAlpha(0.0f);

        dronePText = (TextView) findViewById(R.id.drone_patrol);


        locationManager = (LocationManager)
                getSystemService(Context.LOCATION_SERVICE);

        // NEW
        // Get last known coordinates
        currentLocation = locationManager.getLastKnownLocation(locationManager.GPS_PROVIDER);

        locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0,
                0, locationListener);

    }

    // Listen to location changes
    private final LocationListener locationListener = new LocationListener() {
        @Override
        public void onLocationChanged(final Location location) {
            // Update current location
            currentLocation = location;
        }

        @Override
        public void onStatusChanged(String s, int i, Bundle bundle) {

        }

        @Override
        public void onProviderEnabled(String s) {

        }

        @Override
        public void onProviderDisabled(String s) {

        }
    };

    @Override
    protected void onDestroy() {
        super.onDestroy();
        stopService(i);
        stopPanicRequest(null);
    }

    protected void moveDownDrone(){
        ObjectAnimator scaleUPX = ObjectAnimator.ofFloat(droneImg, "scaleX", 1.7f);
        ObjectAnimator scaleUPY = ObjectAnimator.ofFloat(droneImg, "scaleY", 1.7f);
        scaleUPX.setDuration(2500);
        scaleUPY.setDuration(2500);

        ObjectAnimator moveDownY = ObjectAnimator.ofFloat(droneImg, "translationY", +450);
        moveDownY.setDuration(2500);

        AnimatorSet scaleDown = new AnimatorSet();
        AnimatorSet moveDown = new AnimatorSet();

        scaleDown.play(scaleUPX).with(scaleUPY);
        moveDown.play(moveDownY);

        scaleDown.start();
        moveDown.start();
    }

    protected void moveUpDrone(){
        ObjectAnimator scaleDOWNX = ObjectAnimator.ofFloat(droneImg, "scaleX", 1f);
        ObjectAnimator scaleDOWNY = ObjectAnimator.ofFloat(droneImg, "scaleY", 1f);
        scaleDOWNX.setDuration(2500);
        scaleDOWNY.setDuration(2500);

        ObjectAnimator moveUpY = ObjectAnimator.ofFloat(droneImg, "translationY", 0);
        moveUpY.setDuration(2500);

        AnimatorSet scaleDown = new AnimatorSet();
        AnimatorSet moveUp = new AnimatorSet();

        scaleDown.play(scaleDOWNX).with(scaleDOWNY);
        moveUp.play(moveUpY);

        scaleDown.start();
        moveUp.start();
    }

    @SuppressLint("MissingPermission")
    public void getGPS(View v) throws InterruptedException, MqttException {

        // Get ServerURI
        TextView URI = (TextView) findViewById(R.id.ServerURI);
        ServerURI = "tcp://" + URI.getText().toString() + ":1883";

        // Get phone's ID
        TextView tmpPhoneID = (TextView) findViewById(R.id.phoneID);
        phoneID = tmpPhoneID.getText().toString();

        // MQTT Client initialization
        String clientID = MqttClient.generateClientId();
        client = new MqttAndroidClient(MainActivity.this.getApplicationContext(), ServerURI, clientID);

        // MQTT Callback Handler
        client.setCallback(new MqttCallback() {
            @Override
            public void connectionLost(Throwable cause) {
                MainActivity.this.runOnUiThread(new Runnable() {
                    public void run() {
                        Toast.makeText(MainActivity.this,"Connection Failed!", Toast.LENGTH_LONG).show();
                        cancelRequest();
                    }
                });
            }

            @Override
            public void messageArrived(String topic, MqttMessage message) throws Exception {
                if (topic.equals("sensors"))
                {
                    //System.out.println("topic: " + topic + ", msg: " + new String(message.getPayload()));
                    String messageReceived = new String(message.getPayload());
                    JSONObject reader = new JSONObject(messageReceived);

                    System.out.println("reader = " + reader);

                    // Check if it is a message with sender ('FROM') and receiver ('TO')
                    if (!reader.isNull("FROM") && !reader.isNull("TO"))
                    {
                        // Check if the message is meant for this phone
                        if (reader.getString("TO").equals(phoneID))
                        {
                            if (reader.getString("FROM").equals("GroundStation"))
                            {
                                String type = reader.getString("type");
                                // Check the type of message
                                // In case the there are no drones available and the mission cannot be accomplished
                                if (type.equals("chaseDenied"))
                                {
                                    MainActivity.this.runOnUiThread(new Runnable() {
                                        public void run() {
                                            Toast.makeText(MainActivity.this, "There are no available drones! Try again later!", Toast.LENGTH_SHORT).show();
                                            cancelRequest();
                                        }
                                    });
                                }
                                // In case the mission is accepted
                                else if (type.equals("chaseAccepted"))
                                {
                                    droneAssigned = reader.getString("droneAssigned");
                                    MainActivity.this.runOnUiThread(new Runnable() {
                                        public void run() {
                                            Toast.makeText(MainActivity.this, "Request accepted! Drone " + droneAssigned + " has been sent!", Toast.LENGTH_SHORT).show();
                                        }
                                    });
                                }
                            }
                            else if (reader.getString("FROM").equals(droneAssigned))
                            {
                                // In case the mission is canceled while on the run due to insufficient drones to perform the relay
                                if (reader.getString("type").equals("chaseCanceled"))
                                {
                                    MainActivity.this.runOnUiThread(new Runnable() {
                                        public void run() {
                                            Toast.makeText(MainActivity.this, "The mission had to be canceled due to lack of drones!", Toast.LENGTH_SHORT).show();
                                            cancelRequest();
                                        }
                                    });
                                }
                            }
                        }
                    }
                }
            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken token) {
            }
        });

        GPS_ON = displayGpsStatus();
        if (GPS_ON) {

            //ANIMATIONS
            MainActivity.globalStop = 0;
            moveDownDrone();

            btnGetLocation.setClickable(false);
            btnGetLocation.animate().scaleY(0.01f).setDuration(750);
            btnGetLocation.animate().scaleX(0.01f).setDuration(750);
            btnGetLocation.animate().alpha(0.0f);

            btnCancel.animate().alpha(1.0f);
            btnCancel.animate().scaleY(1.0f).setDuration(750);
            btnCancel.animate().scaleX(1.0f).setDuration(750);
            btnCancel.setClickable(true);

            dronePText.animate().alpha(0.0f).setDuration(1250);
            helpText.animate().alpha(1.0f).setDuration(1250);

            i = new Intent(getApplicationContext(),GPS_Service.class);

            // Subscribe to sensors' topic and send locatorRequest message
            subscribeTopic("sensors");

            // Send coordinates each 5 seconds
            timer = new Timer();
            sendCoordinates = new TimerTask(){
                public void run(){
                    //Check if there is no information about the current location
                    if (currentLocation!=null){
                         try {
                            IMqttToken token = client.connect();
                            token.setActionCallback(new IMqttActionListener() {
                                @Override
                                public void onSuccess(IMqttToken asyncActionToken) {
                                    try {

                                        Long tsLong = System.currentTimeMillis();
                                        String ts = tsLong.toString();

                                        String message = "{\"sensorId\" :\"" + phoneID + "\", "
                                        + "\"type\" : \"locator\", "
                                        + "\"timestamp\" : " + ts + ", "
                                        + "\"value\" :  {"
                                        + "\"lat\" : " + currentLocation.getLatitude() + ", "
                                        + "\"lon\" : " + currentLocation.getLongitude() + ", "
                                        + "\"alt\" : " + currentLocation.getAltitude() + " }}";

                                        client.publish("sensors", message.getBytes(), 0, false);
                                    }
                                    catch(MqttException e){
                                        e.printStackTrace();
                                    }
                                }

                                @Override
                                public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                                    System.out.println("Connection failed!");
                                    Toast.makeText(MainActivity.this,"Connection Failed!", Toast.LENGTH_LONG).show();
                                }
                            });
                        } catch (MqttException e) {
                            e.printStackTrace();
                        }
                    }
                    else
                    {
                        // Display error message if it was unable to fetch the location
                        MainActivity.this.runOnUiThread(new Runnable() {
                            public void run() {
                                Toast.makeText(MainActivity.this, "Unable to get a location!", Toast.LENGTH_SHORT).show();
                            }
                        });
                    }
                }
            };
            timer.schedule(sendCoordinates, 5000, 5000);

        } else {
            alertbox("Gps Status!!", "Your GPS is: OFF");
        }

    }

    public void publishMessage(final String payload, final String topic) {
        try {
            IMqttToken token = client.connect();
            System.out.println("publishMessage()");
            token.setActionCallback(new IMqttActionListener() {
                @Override
                public void onSuccess(IMqttToken asyncActionToken) {
                    // after this is called you can publish messages
                    try {
                        System.out.println("onSuccess()");
                        MqttMessage message = new MqttMessage();
                        message.setPayload(payload.getBytes());
                        message.setQos(0);
                        System.out.println("client = " + client);
                        client.publish(topic, message,null, new IMqttActionListener() {
                            @Override
                            public void onSuccess(IMqttToken asyncActionToken) {
                                System.out.println("onSuccess()");
                            }

                            @Override
                            public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                                MainActivity.this.runOnUiThread(new Runnable() {
                                    public void run() {
                                        Toast.makeText(MainActivity.this, "Connection Failed!", Toast.LENGTH_SHORT).show();
                                    }
                                });
                            }
                        });
                    } catch (Exception e) {
                        e.printStackTrace();
                        System.out.println("catch()");
                    }
                }
                @Override
                public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                    MainActivity.this.runOnUiThread(new Runnable() {
                        public void run() {
                            Toast.makeText(MainActivity.this, "Connection Failed!", Toast.LENGTH_SHORT).show();
                        }
                    });
                }
            });


        } catch (MqttException e) {
            System.out.println("catch()");
            e.printStackTrace();
        }
    }

    public void unsubscribeTopic(String topic) {
        try {
            client.unsubscribe(topic, 0, null);
        } catch (MqttException e) {
            e.printStackTrace();
        }
    }

    // Subscribe to sensors' topic and send locatorRequest message
    public void subscribeTopic(final String topic) {
        try {
            IMqttToken token = client.connect();
            System.out.println("subscribeTopic()");
            token.setActionCallback(new IMqttActionListener() {
                @Override
                public void onSuccess(IMqttToken asyncActionToken) {
                    try {
                        try {
                            client.subscribe(topic, 0, null, new IMqttActionListener() {
                                @Override
                                public void onSuccess(IMqttToken asyncActionToken) {
                                    // Send locatorRequest message
                                    publishMessage("{\"sensorId\" :\"" + phoneID + "\", "
                                            + "\"type\" : \"locatorRequest\"}","sensors");
                                }

                                @Override
                                public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                                    MainActivity.this.runOnUiThread(new Runnable() {
                                        public void run() {
                                            Toast.makeText(MainActivity.this, "Connection Failed!", Toast.LENGTH_SHORT).show();
                                        }
                                    });
                                }
                            });

                        } catch (MqttException e) {
                            e.printStackTrace();
                        }
                    } catch (Exception e) {
                        e.printStackTrace();
                        System.out.println("catch()");
                    }
                }
                @Override
                public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                    MainActivity.this.runOnUiThread(new Runnable() {
                        public void run() {
                            Toast.makeText(MainActivity.this, "Connection Failed!", Toast.LENGTH_SHORT).show();
                        }
                    });
                }
            });


        } catch (MqttException e) {
            System.out.println("catch()");
            e.printStackTrace();
        }
    }

    public void cancelRequest(){
        // Cancel timers
        sendCoordinates.cancel();
        timer.cancel();

        droneAssigned = null;

        unsubscribeTopic("sensors");

        MainActivity.globalStop = 1;
        moveUpDrone();

        btnCancel.setClickable(false);
        btnCancel.animate().scaleY(0.01f).setDuration(750);
        btnCancel.animate().scaleX(0.01f).setDuration(750);
        btnCancel.animate().alpha(0.0f);;

        btnGetLocation.animate().alpha(1.0f);;
        btnGetLocation.animate().scaleY(1.0f).setDuration(750);
        btnGetLocation.animate().scaleX(1.0f).setDuration(750);
        btnGetLocation.setClickable(true);


        helpText.animate().alpha(0.0f).setDuration(1250);
        dronePText.animate().alpha(1.0f).setDuration(1250);

    }

    /*----Method to Check GPS is enable or disable ----- */
    private Boolean displayGpsStatus() {
        ContentResolver contentResolver = getBaseContext()
                .getContentResolver();
        boolean gpsStatus = Settings.Secure
                .isLocationProviderEnabled(contentResolver,
                        LocationManager.GPS_PROVIDER);
        if (gpsStatus) {
            return true;

        } else {
            return false;
        }
    }

    /*----------Method to create an AlertBox ------------- */
    protected void alertbox(String title, String mymessage) {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setMessage("Your Device's GPS is Disabled")
                .setCancelable(false)
                .setTitle("*** GPS Status ***")
                .setPositiveButton("Enable GPS",
                        new DialogInterface.OnClickListener() {
                            public void onClick(DialogInterface dialog, int id) {
                                // finish the current activity
                                Intent myIntent = new Intent(
                                        Settings.ACTION_SECURITY_SETTINGS);
                                startActivity(myIntent);
                                dialog.cancel();
                            }
                        })
                .setNegativeButton("Cancel",
                        new DialogInterface.OnClickListener() {
                            public void onClick(DialogInterface dialog, int id) {
                                // cancel the dialog box
                                dialog.cancel();
                            }
                        });
        AlertDialog alert = builder.create();
        alert.show();
    }

    public void stopPanicRequest(View v) {

        new SweetAlertDialog(this, SweetAlertDialog.WARNING_TYPE)
                .setTitleText("Cancel Help Request?")
                .setCancelText("I need help!")
                .setConfirmText("Yes, I'm fine!")
                .showCancelButton(true)
                .setConfirmClickListener(new SweetAlertDialog.OnSweetClickListener() {
                    @Override
                    public void onClick(SweetAlertDialog sDialog) {
                        sDialog.cancel();

                        // Cancel message
                        publishMessage("{\"sensorId\" :\"" + phoneID + "\", "
                                + "\"type\" : \"locatorCancel\"}","sensors");

                        Toast.makeText(MainActivity.this, "Request has been canceled! Drone " + droneAssigned + " has been sent back!", Toast.LENGTH_SHORT).show();
                        cancelRequest();
                    }
                })
                .show();
    }

    public void checkPermissions(){
        // Adding Permission Request
        if (ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_DENIED)
        {
            ActivityCompat.requestPermissions(MainActivity.this, new String[] {Manifest.permission.ACCESS_FINE_LOCATION}, REQUEST_GPS_PERMISSION);
        }
        if (ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.READ_PHONE_STATE) == PackageManager.PERMISSION_DENIED)
        {
            ActivityCompat.requestPermissions(MainActivity.this, new String[] {Manifest.permission.READ_PHONE_STATE}, REQUEST_READ_PHONE_STATE);
        }
        if (ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.INTERNET) == PackageManager.PERMISSION_DENIED)
        {
            ActivityCompat.requestPermissions(MainActivity.this, new String[] {Manifest.permission.INTERNET}, REQUEST_INTERNET);
        }
        if (ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.ACCESS_NETWORK_STATE) == PackageManager.PERMISSION_DENIED)
        {
            ActivityCompat.requestPermissions(MainActivity.this, new String[] {Manifest.permission.ACCESS_NETWORK_STATE}, REQUEST_NETWORK_STATE);
        }
        if (ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.WAKE_LOCK) == PackageManager.PERMISSION_DENIED)
        {
            ActivityCompat.requestPermissions(MainActivity.this, new String[] {Manifest.permission.WAKE_LOCK}, REQUEST_WAKE_LOCK);
        }
    }
}


