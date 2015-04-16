package istic.gla.groupeb.flerjeco.springRest;

import android.content.Intent;
import android.os.AsyncTask;
import android.os.Bundle;
import android.util.Log;

import entity.Intervention;
import istic.gla.groupeb.flerjeco.agent.interventionsList.ListInterventionsActivity;
import istic.gla.groupeb.flerjeco.codis.intervention.InterventionActivity;

/**
 * Represents an asynchronous task used to get interventions
 */
public class GetAllInterventionsTask extends AsyncTask<Void, Void, Boolean> {

    private int count = 0;
    private Intervention[] interventionTab;
    private IInterventionsActivity activity;

    public GetAllInterventionsTask(IInterventionsActivity activity) {
        this.activity = activity;
    }

    public GetAllInterventionsTask(IInterventionsActivity activity, int count) {
        this.count = count;
        this.activity = activity;
    }

    @Override
    protected Boolean doInBackground(Void... params) {
        SpringService service = new SpringService();
        interventionTab = service.getAllInterventions();
        if(interventionTab ==  null) {
            return false;
        }
        Log.i("GetAllInterventionsTask", "interventionTab size : " + interventionTab.length);
        Log.i("GetAllInterventionsTask", "doInBackground end");
        return true;
    }

    @Override
    protected void onPostExecute(final Boolean success) {
        if(success)
            activity.updateInterventions(interventionTab);
        else {
            count++;
            if(count < 4)
                new GetAllInterventionsTask(activity, count);
        }
    }
}