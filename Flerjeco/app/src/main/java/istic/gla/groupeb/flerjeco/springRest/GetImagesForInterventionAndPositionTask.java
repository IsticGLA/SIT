package istic.gla.groupeb.flerjeco.springRest;

import android.os.AsyncTask;
import android.util.Log;

import com.google.android.gms.maps.model.LatLng;

import org.springframework.http.ResponseEntity;

import java.util.Arrays;
import java.util.Date;

import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupeb.flerjeco.agent.droneVisualisation.ImageRefresher;

/**
 * Represents an asynchronous call for getting drone position and showing them on the map
 */
public class GetImagesForInterventionAndPositionTask extends AsyncTask<Object, Void, ResponseEntity<Image[]>> {

    private final String TAG = GetImagesForInterventionAndPositionTask.class.getSimpleName();
    private final ImageRefresher refresher;
    private final long interventionId;
    private final LatLng position;
    private Date start;
    private SpringService service = new SpringService();

    /**
     * constructor
     * @param refresher the refresher for callbacks
     */
    public GetImagesForInterventionAndPositionTask(ImageRefresher refresher, Long interventionId, LatLng position){
        this.refresher = refresher;
        this.interventionId = interventionId;
        this.position = position;
    }

    @Override
    protected ResponseEntity<Image[]> doInBackground(Object... params) {
        try {
            Log.v(TAG, "Get the position of the drone for the intervention with id : " + interventionId);
            start = new Date();
            return service.getAllImageForInterventionAndPosition(interventionId, position);
        } catch (Exception e) {
            Log.e(TAG, e.getMessage(), e);
        }

        return null;
    }

    @Override
    protected void onPostExecute(ResponseEntity<Image[]> response) {
        if (response != null) {
            switch(response.getStatusCode()){
                case OK:
                    refresher.updateWithImages(Arrays.asList(response.getBody()));
                    break;
                default:
                    Log.w(TAG, "failed to refresh images : " + response.getStatusCode());
            }
        } else{
            Log.e(TAG, "got null response");
        }
    }
}