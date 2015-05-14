package istic.gla.groupeb.flerjeco.agent.planZone;

import android.os.AsyncTask;
import android.util.Log;

import org.springframework.http.ResponseEntity;

import java.util.Date;

import entity.Drone;
import istic.gla.groupeb.flerjeco.springRest.SpringService;

/**
 * Represents an asynchronous call for add new path for drone in the current intervention
 * the user.
 */
public class GetPositionDroneTask extends AsyncTask<Long, Void, ResponseEntity<Drone[]>> {

    private final String TAG = GetPositionDroneTask.class.getSimpleName();
    private final PlanZoneMapFragment fragment;
    private final Date start;
    private SpringService service = new SpringService();

    /**
     * constructor
     * @param fragment the fragment for callbacks
     */
    public GetPositionDroneTask(PlanZoneMapFragment fragment){
        this.fragment = fragment;
        start = new Date();
    }

    @Override
    protected ResponseEntity<Drone[]> doInBackground(Long... params) {
        try {
            Log.v(TAG, "Get the position of the drone for the intervention with id : " + params[0]);
            return service.getAllDroneByIntervention(params);
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
                    fragment.showDrones(response.getBody(), (responseDate.getTime() - start.getTime()));
                    break;
                default:
                    Log.w(TAG, "failed to refresh drone position : " + response.getStatusCode());
            }
        } else{
            Log.e(TAG, "got null response");
        }
        Date end = new Date();
        Log.v(TAG, "refreshed drones in " + (end.getTime() - start.getTime()) + "ms");
    }
}