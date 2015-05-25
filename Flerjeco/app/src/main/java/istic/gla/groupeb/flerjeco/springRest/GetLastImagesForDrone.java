package istic.gla.groupeb.flerjeco.springRest;

import android.os.AsyncTask;
import android.util.Log;
import android.widget.Toast;

import com.google.android.gms.maps.model.LatLng;

import org.springframework.http.ResponseEntity;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;

import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.agent.droneVisualisation.ImageRefresher;
import istic.gla.groupeb.flerjeco.agent.droneVisualisation.VideoRefresher;

/**
 * Represents an asynchronous call for getting drone position and showing them on the map
 */
public class GetLastImagesForDrone extends AsyncTask<Object, Void, ResponseEntity<Image>> {

    private final String TAG = GetLastImagesForDrone.class.getSimpleName();
    private final VideoRefresher refresher;
    private final String droneLabel;
    private SpringService service = new SpringService();

    /**
     * constructor
     * @param refresher the refresher for callbacks
     */
    public GetLastImagesForDrone(VideoRefresher refresher, String droneLabel){
        this.refresher = refresher;
        this.droneLabel = droneLabel;
    }

    @Override
    protected ResponseEntity<Image> doInBackground(Object... params) {
        try {
            Log.d(TAG, "Get the image for drone " + droneLabel);
            return service.getLastImageForDrone(droneLabel);
        } catch (Exception e) {
            Log.e(TAG, e.getMessage(), e);
        }
        return null;
    }

    @Override
    protected void onPostExecute(ResponseEntity<Image> response) {
        if (response != null) {
            switch(response.getStatusCode()){
                case OK:
                    Log.d(TAG, "got response with image and will update resfresher with it");
                    refresher.setLastImage(response.getBody());
                    return;
                default:
                    Log.w(TAG, "failed to refresh images : " + response.getStatusCode());
            }
        } else{
            Log.e(TAG, "got null response");
        }
    }
}