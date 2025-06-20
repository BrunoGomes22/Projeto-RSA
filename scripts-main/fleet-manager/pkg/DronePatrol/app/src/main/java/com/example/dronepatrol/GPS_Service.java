package com.example.dronepatrol;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.util.Log;

import com.android.volley.DefaultRetryPolicy;
import com.android.volley.Request;
import com.android.volley.RequestQueue;
import com.android.volley.Response;
import com.android.volley.RetryPolicy;
import com.android.volley.VolleyError;
import com.android.volley.toolbox.JsonObjectRequest;
import com.android.volley.toolbox.Volley;

import org.json.JSONException;
import org.json.JSONObject;

import androidx.annotation.Nullable;
import androidx.core.app.NotificationCompat;

public class GPS_Service extends Service {
    private LocationManager locationManager;
    private LocationListener locationListener;
    private String uuid;
    @Nullable
    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        this.uuid=intent.getStringExtra("uuid");
        Log.d("Uuid","Uuid in Service "+this.uuid);

        locationListener = new LocationListener() {
            @Override
            public void onLocationChanged(final Location loc) {

                Handler mainHandler = new Handler(getMainLooper());
                mainHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        //Toast.makeText(getApplicationContext(), "Latitude: "+loc.getLatitude()+" Longitude: "+loc.getLongitude(), Toast.LENGTH_SHORT).show();
                        RequestQueue queue = Volley.newRequestQueue(getApplicationContext());

                        String url ="http://motamiguel.pythonanywhere.com/panic/"+uuid+"/";

                        JSONObject jsonReq = new JSONObject();
                        try{
                            jsonReq.put("uuid",uuid);
                            jsonReq.put("longitude",loc.getLongitude());
                            jsonReq.put("latitude",loc.getLatitude());
                        }catch (JSONException e){
                            e.printStackTrace();
                        }


                        JsonObjectRequest objectRequest = new JsonObjectRequest(Request.Method.PUT, url,jsonReq,
                                new Response.Listener<JSONObject>() {
                                    @Override
                                    public void onResponse(JSONObject response) {
                                        Log.d("Coordinates Updated","Latitude: "+loc.getLatitude()+" Longitude: "+loc.getLongitude());
                                    }
                                },
                                new Response.ErrorListener() {
                                    @Override
                                    public void onErrorResponse(VolleyError error) {
                                        Log.d("Error",error.toString());
                                    }
                                }
                        );
                        // Add the request to the RequestQueue.
                        int socketTimeout = 15000;//30 seconds - change to what you want
                        RetryPolicy policy = new DefaultRetryPolicy(socketTimeout, DefaultRetryPolicy.DEFAULT_MAX_RETRIES, DefaultRetryPolicy.DEFAULT_BACKOFF_MULT);
                        objectRequest.setRetryPolicy(policy);
                        queue.add(objectRequest);

                        if (MainActivity.globalStop == 1){
                            onDestroy();
                        }
                    }
                });



            }

            @Override
            public void onProviderDisabled(String provider) {
                // TODO Auto-generated method stub
            }

            @Override
            public void onProviderEnabled(String provider) {
                // TODO Auto-generated method stub
            }

            @Override
            public void onStatusChanged(String provider,
                                        int status, Bundle extras) {
                // TODO Auto-generated method stub
            }

        };

        locationManager= (LocationManager) getApplicationContext().getSystemService(Context.LOCATION_SERVICE);
        locationManager.requestLocationUpdates(locationManager
                .GPS_PROVIDER, 3000, (float) 0.0,locationListener);

        return super.onStartCommand(intent, flags, startId);
    }

    @Override
    public void onCreate() {
        super.onCreate();
        if (Build.VERSION.SDK_INT >= 26) {
            String CHANNEL_ID = "my_channel_01";
            NotificationChannel channel = new NotificationChannel(CHANNEL_ID,
                    "My Channel",
                    NotificationManager.IMPORTANCE_DEFAULT);

            ((NotificationManager) getSystemService(Context.NOTIFICATION_SERVICE)).createNotificationChannel(channel);

            Notification notification = new NotificationCompat.Builder(this, CHANNEL_ID)
                    .setContentTitle("")
                    .setContentText("").build();

            startForeground(1, notification);
        }
    }
    @Override
    public void onDestroy() {
        super.onDestroy();
        if(locationManager != null){
            //noinspection MissingPermission
            locationManager.removeUpdates(locationListener);
            locationManager = null;
        }
    }
}

