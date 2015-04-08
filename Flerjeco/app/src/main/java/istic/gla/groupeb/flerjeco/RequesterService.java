package istic.gla.groupeb.flerjeco;

import android.app.Service;
import android.content.Intent;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.os.Messenger;
import android.os.RemoteException;
import android.util.Pair;

import org.apache.http.HttpResponse;
import org.apache.http.HttpStatus;
import org.apache.http.StatusLine;
import org.apache.http.client.HttpClient;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.client.methods.HttpUriRequest;
import org.apache.http.entity.ByteArrayEntity;
import org.apache.http.impl.client.DefaultHttpClient;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.UnsupportedEncodingException;

public class RequesterService extends Service {

    public static final int MSG_ZONE = 1;
    public static final int MSG_POINT = 2;
    public static final int MSG_IMG = 3;
    public static final int MSG_AGENTS = 4;
    public static final int MSG_DRONES = 5;
    public static final int MSG_DRONE = 6;
    public static final int MSG_LOGIN = 7;


    private static final String URL = "http://localhost:8080/";

    final Messenger mMessenger = new Messenger(new IncomingHandler());

    @Override
    public IBinder onBind(Intent intent) {
        return mMessenger.getBinder();
    }

    private String formatZone(Coordinates coordinates) {
        String stringCoordinates = "{\"coordinates\":[{\"positions\" : [";
        for(int i = 0; i < coordinates.getCoordinates().size() - 1; i++) {
            stringCoordinates += "{\"latitude\":" + coordinates.getCoordinates().get(i).first + ",\"longitude\":" + coordinates.getCoordinates().get(i).second + "},";
        }
        stringCoordinates += "{\"latitude\":" + coordinates.getCoordinates().get(coordinates.getCoordinates().size() - 1).first + ",\"longitude\":" + coordinates.getCoordinates().get(coordinates.getCoordinates().size() - 1).second + "}";
        stringCoordinates += "]}]}";
        System.out.println(stringCoordinates);
        return stringCoordinates;
    }

    class IncomingHandler extends Handler {
        @Override
        public void handleMessage(Message msg) {
            Coordinates coordinates;
            HttpUriRequest request;
            switch (msg.what) {
                case MSG_POINT:
                    //send point to the server
                    coordinates = msg.getData().getParcelable("coord");
                    request = new HttpGet("geoposition/point/" + coordinates.getCoordinates().get(0).first + "/" + coordinates.getCoordinates().get(0).second);
                    break;
                case MSG_ZONE:
                    //send zone to the server
                    coordinates = msg.getData().getParcelable("coord");
                    request = new HttpPost(URL + "geoposition/setzone");
                    try {
                        ((HttpPost)request).setEntity(new ByteArrayEntity(formatZone(coordinates).getBytes("UTF-8")));
                    } catch (UnsupportedEncodingException e) {
                        e.printStackTrace();
                    }
                    break;
                case MSG_IMG:
                    //ask server for images
                    request = new HttpGet("images");
                    break;
                case MSG_AGENTS:
                    //ask server for agents
                    request = new HttpGet("agents");
                    break;
                case MSG_DRONES:
                    //ask server for the list of drones
                    request = new HttpGet("drones");
                    break;
                case MSG_DRONE:
                    //ask server for one special drone
                    int drone = (Integer)msg.obj;
                    request = new HttpGet("drone/" + drone);
                    break;
                case MSG_LOGIN:
                    //ask server for log in
                    String login = msg.getData().getString("login");
                    String password = msg.getData().getString("password");
                    request = new HttpGet("login/"+login+"password/"+password);
                    break;
                default:
                    super.handleMessage(msg);
                    return;
            }
            new RequestTask(request, msg.what, msg.replyTo).execute();
        }
    }
}
