package istic.gla.groupeb.flerjeco.springRest;

import android.os.AsyncTask;
import android.util.Log;

import org.springframework.http.ResponseEntity;

import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupeb.flerjeco.agent.droneVisualisation.VideoRefresher;

/**
 * Represents an asynchronous call for getting drone position and showing them on the map
 */
public class GetLastImageForDroneTask extends AsyncTask<Object, Void, ResponseEntity<Image>> {

    private final String TAG = GetLastImageForDroneTask.class.getSimpleName();
    private final VideoRefresher refresher;
    private final String droneLabel;
    private SpringService service = new SpringService();

    /**
     * constructor
     * @param refresher the refresher for callbacks
     */
    public GetLastImageForDroneTask(VideoRefresher refresher, String droneLabel){
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