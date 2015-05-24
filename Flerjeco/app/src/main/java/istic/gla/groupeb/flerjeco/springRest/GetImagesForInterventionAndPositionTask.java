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

/**
 * Represents an asynchronous call for getting drone position and showing them on the map
 */
public class GetImagesForInterventionAndPositionTask extends AsyncTask<Object, Void, ResponseEntity<Image[]>> {

    private final String TAG = GetImagesForInterventionAndPositionTask.class.getSimpleName();
    private final ImageRefresher refresher;
    private final long interventionId;
    private final long timestamp;
    private final LatLng position;
    private Date start;
    private SpringService service = new SpringService();

    /**
     * constructor
     * @param refresher the refresher for callbacks
     */
    public GetImagesForInterventionAndPositionTask(ImageRefresher refresher, Long interventionId, LatLng position, long timeStamp){
        this.refresher = refresher;
        this.interventionId = interventionId;
        this.position = position;
        this.timestamp = timeStamp;
    }

    @Override
    protected ResponseEntity<Image[]> doInBackground(Object... params) {
        try {
            Log.d(TAG, "Get the images intervention with id : " + interventionId + " and position : " + position + " timestamp:" + timestamp);
            start = new Date();
            return service.getAllImageForInterventionAndPosition(interventionId, position, timestamp);
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
                    Log.d(TAG, "got response with image and will update resfresher with it");
                    refresher.loadImages(Arrays.asList(response.getBody()));
                    return;
                default:
                    Log.w(TAG, "failed to refresh images : " + response.getStatusCode());
            }
        } else{
            Log.e(TAG, "got null response");
        }
        Toast.makeText(refresher.getContext(), R.string.fail_get_images, Toast.LENGTH_SHORT).show();
        refresher.loadImages(new ArrayList<Image>());
    }
}