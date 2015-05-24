package istic.gla.groupeb.flerjeco.springRest;

import android.os.AsyncTask;
import android.util.Log;

import org.springframework.http.ResponseEntity;

import java.util.Date;

import istic.gla.groupb.nivimoju.entity.Drone;
import istic.gla.groupeb.flerjeco.agent.DronesMapFragment;

/**
 * Represents an asynchronous call for getting drone position and showing them on the map
 */
public class GetPositionDroneTask extends AsyncTask<Object, Void, ResponseEntity<Drone[]>> {

    private final String TAG = GetPositionDroneTask.class.getSimpleName();
    private final DronesMapFragment fragment;
    private final long interventionId;
    private Date start;
    private SpringService service = new SpringService();

    /**
     * constructor
     * @param fragment the fragment for callbacks
     */
    public GetPositionDroneTask(DronesMapFragment fragment, Long interventionId){
        this.fragment = fragment;
        this.interventionId = interventionId;
    }

    @Override
    protected ResponseEntity<Drone[]> doInBackground(Object... params) {
        try {
            Log.v(TAG, "Get the position of the drone for the intervention with id : " + interventionId);
            start = new Date();
            return service.getAllDroneByIntervention(interventionId);
        } catch (Exception e) {
            Log.e(TAG, e.getMessage(), e);
        }

        return null;
    }

    @Override
    protected void onPostExecute(ResponseEntity<Drone[]> response) {
        Date responseDate = new Date();
        if (response != null) {
            switch(response.getStatusCode()){
                case OK:
                    fragment.showDrones(response.getBody());
                    break;
                default:
                    Log.w(TAG, "failed to refresh drone position : " + response.getStatusCode());
            }
        } else{
            Log.e(TAG, "got null response");
        }
        Date end = new Date();
        Log.v(TAG, "refreshed drones in " + (end.getTime() - start.getTime()) + "ms");
        fragment.refreshDrones();
    }
}