package istic.gla.groupeb.flerjeco.springRest;

import android.hardware.Camera;
import android.os.AsyncTask;
import android.util.Log;
import android.widget.Toast;

import org.springframework.http.ResponseEntity;

import java.util.Collections;
import java.util.Comparator;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Path;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.agent.planZone.EPathOperation;
import istic.gla.groupeb.flerjeco.agent.planZone.PlanZoneActivity;
import istic.gla.groupeb.flerjeco.agent.planZone.PlanZoneMapFragment;

/**
 * Created by jules on 27/05/15.
 */
public class UpdateAreaTask extends AsyncTask<Object[], Void, ResponseEntity<Intervention>> {

    private final String TAG = UpdateAreaTask.class.getSimpleName();
    private final PlanZoneMapFragment fragment;
    private final EPathOperation operation;
    private SpringService service = new SpringService();

    public UpdateAreaTask(PlanZoneMapFragment fragment, EPathOperation operation) {
        this.fragment = fragment;
        this.operation = operation;
    }

    @Override
    protected ResponseEntity<Intervention> doInBackground(Object[]... params) {
        try {
            Log.i(TAG, "Update paths of the intervention with id : " + params[0][0]);
            return service.updateInterventionArea(params[0], this.operation);
        } catch (Exception e) {
            Log.e(TAG, e.getMessage(), e);
        }
        return null;
    }

    @Override
    protected void onPostExecute(ResponseEntity<Intervention> response) {
        //boolean revert = true;
        if(response != null){
            switch(response.getStatusCode()){
                case OK:
                    Intervention intervention = response.getBody();
                    PlanZoneActivity p = ((PlanZoneActivity) fragment.getActivity());
                    Collections.sort(intervention.getWatchArea(), new Comparator<Area>() {
                        @Override
                        public int compare(Area lhs, Camera.Area rhs) {
                            return Long.valueOf(lhs.getId()).compareTo(Long.valueOf(rhs.getId()));
                        }
                    });
                    p.refreshList(intervention);
                    Toast.makeText(p.getApplicationContext(),
                            fragment.getActivity().getResources().getString(R.string.drone_update_path_success),
                            Toast.LENGTH_LONG).show();
                    //revert = false;
                    fragment.applyUpdateAfterOperation(intervention);
                    break;
                case SERVICE_UNAVAILABLE:
                    ((PlanZoneActivity)fragment.getActivity()).showProgress(false);
                    Log.i(TAG, "No drones were available, cannot add area");
                    Toast.makeText(fragment.getActivity().getApplicationContext(),
                            fragment.getActivity().getResources().getString(R.string.drone_update_area_error_not_available),
                            Toast.LENGTH_LONG).show();
                    break;
                default:
                    ((PlanZoneActivity)fragment.getActivity()).showProgress(false);
                    Log.i(TAG, "path update failed");
                    Toast.makeText(fragment.getActivity().getApplicationContext(),
                            fragment.getActivity().getResources().getString(R.string.drone_update_path_error_generic),
                            Toast.LENGTH_LONG).show();
            }
        } else {
            ((PlanZoneActivity)fragment.getActivity()).showProgress(false);
            Log.w(TAG, "got null response");
            Toast.makeText(fragment.getActivity().getApplicationContext(),
                    fragment.getActivity().getResources().getString(R.string.drone_update_path_error_generic),
                    Toast.LENGTH_LONG).show();
        }
    }
}
