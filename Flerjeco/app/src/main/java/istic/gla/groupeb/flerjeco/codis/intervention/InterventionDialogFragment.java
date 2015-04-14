package istic.gla.groupeb.flerjeco.codis.intervention;

import android.animation.Animator;
import android.animation.AnimatorListenerAdapter;
import android.annotation.TargetApi;
import android.os.Build;
import android.support.v4.app.DialogFragment;
import android.os.AsyncTask;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Spinner;
import android.widget.Toast;

import org.springframework.web.client.HttpStatusCodeException;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import entity.IncidentCode;
import entity.Intervention;
import entity.Resource;
import entity.ResourceType;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.springRest.SpringService;
import util.State;


public class InterventionDialogFragment extends DialogFragment implements OnTaskCompleted{


    Spinner codeSinistreSpinner;

    //Button of intervention creation
    Button intervention_creation_button;

    //Text fields
    EditText nameIntervetionEditText;
    EditText latitudeEditText;
    EditText longitudeEditText;

    private View mProgressView;
    private View mCreateFormView;

    SpringService springService = new SpringService();
    String[] spinnerArray;
    ArrayAdapter<String> spinnerAdapter;

    private HashMap<String, Long> spinnerMap;
    private HashMap<String, List<Long>> resourceTypeMap;


    boolean data_local = false;
    private HttpRequestTask httpRequestTask;
    private AsyncTask<Intervention, Void, Intervention> at;
    private ResourceGetTask resourceGetTask;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View v = inflater.inflate(R.layout.activity_intervention, container, false);

        mCreateFormView = v.findViewById(R.id.intervention_scroll);
        mProgressView = v.findViewById(R.id.create_progress);

        //Set up code sinistre list
        codeSinistreSpinner = (Spinner) v.findViewById(R.id.CodeSinistreSpinner);
        nameIntervetionEditText = (EditText) v.findViewById(R.id.nameInterventionEditText);
        latitudeEditText = (EditText) v.findViewById(R.id.lat);
        longitudeEditText = (EditText) v.findViewById(R.id.lng);

        // Set up Button of intervention creation
        intervention_creation_button = (Button) v.findViewById(R.id.intervention_button);

        httpRequestTask = new HttpRequestTask();
        httpRequestTask.execute();


        // add button listener
        intervention_creation_button.setOnClickListener(new View.OnClickListener() {

            public boolean validate() {

                if (latitudeEditText.getText().toString().length() == 0) {
                    latitudeEditText.setError("latitude est obligatoire!");
                    return false;
                }

                if (longitudeEditText.getText().toString().length() == 0) {
                    longitudeEditText.setError("longitude est obligatoire!");
                    return false;
                }
                showProgress(true);
                return true;
            }


            @Override
            public void onClick(View v) {
                if (validate()) {
                    //String idIncidentCode, double latitude, double longitude, String name
                    Log.i("MAMH", spinnerMap.get(codeSinistreSpinner.getSelectedItem().toString()) + "");


                   /* //Intervetion
                    entity.Intervention intervention;

                    intervention = new entity.Intervention(nameIntervetionEditText.getText().toString(), spinnerMap.get(codeSinistreSpinner.getSelectedItem().toString()).intValue(), Double.valueOf(latitudeEditText.getText().toString()), Double.valueOf(longitudeEditText.getText().toString()));

                    Log.i("MAMH", "Lat : " + intervention.getLatitude() + ", Lng : " + intervention.getLongitude());

                    List<Resource> resources;

                    AsyncTask at = new InterventionPostTask().execute(intervention);*/

                    resourceGetTask = new ResourceGetTask(InterventionDialogFragment.this);
                    resourceGetTask.execute(resourceTypeMap.get(codeSinistreSpinner.getSelectedItem().toString()));

                }
            }
        });

        return v;
    }

    /**
     * Shows the progress UI and hides the login form.
     */
    @TargetApi(Build.VERSION_CODES.HONEYCOMB_MR2)
    public void showProgress(final boolean show) {
        // On Honeycomb MR2 we have the ViewPropertyAnimator APIs, which allow
        // for very easy animations. If available, use these APIs to fade-in
        // the progress spinner.
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB_MR2) {
            int shortAnimTime = getResources().getInteger(android.R.integer.config_shortAnimTime);

            mCreateFormView.setVisibility(show ? View.GONE : View.VISIBLE);
            mCreateFormView.animate().setDuration(shortAnimTime).alpha(
                    show ? 0 : 1).setListener(new AnimatorListenerAdapter() {
                @Override
                public void onAnimationEnd(Animator animation) {
                    mCreateFormView.setVisibility(show ? View.GONE : View.VISIBLE);
                }
            });

            mProgressView.setVisibility(show ? View.VISIBLE : View.GONE);
            mProgressView.animate().setDuration(shortAnimTime).alpha(
                    show ? 1 : 0).setListener(new AnimatorListenerAdapter() {
                @Override
                public void onAnimationEnd(Animator animation) {
                    mProgressView.setVisibility(show ? View.VISIBLE : View.GONE);
                }
            });
        } else {
            // The ViewPropertyAnimator APIs are not available, so simply show
            // and hide the relevant UI components.
            mProgressView.setVisibility(show ? View.VISIBLE : View.GONE);
            mCreateFormView.setVisibility(show ? View.GONE : View.VISIBLE);
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        if(at != null)
            at.cancel(true);
        else if (httpRequestTask != null)
            httpRequestTask.cancel(true);
        else if (resourceGetTask != null)
            resourceGetTask.cancel(true);
    }

    // Backgroud task to get sistre codes
    private class HttpRequestTask extends AsyncTask<Void, Void, IncidentCode[]> {

        @Override
        protected IncidentCode[] doInBackground(Void... params) {
            try {
                IncidentCode[] codes = springService.codeSinistreClient();
                return  codes;

            } catch (HttpStatusCodeException e) {
                Log.e("InterventionActivity", e.getMessage(), e);
            }

            return null;
        }

        @Override
        protected void onPostExecute(IncidentCode[] codes) {

            if(codes != null && codes.length > 0 ) {
                int i = 0;
                spinnerArray = new String[codes.length];
                resourceTypeMap = new HashMap<>();
                spinnerMap = new HashMap();
                for (IncidentCode code : codes) {
                    if(code != null) {

                        spinnerArray[i] = code.getCode();
                        spinnerMap.put(code.getCode(), code.getId());
                        resourceTypeMap.put(code.getCode(), code.getresourceType());
                        i++;
                    }
                    }
                spinnerAdapter = new ArrayAdapter<String>(InterventionDialogFragment.this.getActivity(), android.R.layout.simple_spinner_item,spinnerArray);
                spinnerAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
                codeSinistreSpinner.setAdapter(spinnerAdapter);
            }
        }

    }


    // Backgroud task to post intervention
    private class InterventionPostTask extends AsyncTask<entity.Intervention, Void, entity.Intervention> {

        @Override
        protected entity.Intervention doInBackground(entity.Intervention... params) {
            try {
                return springService.postIntervention(params[0]);
            } catch (HttpStatusCodeException e) {
                Log.e("InterventionActivity", e.getMessage(), e);
                return null;
            }

        }

        @Override
        protected void onPostExecute(entity.Intervention resultPost) {
            Toast.makeText(InterventionDialogFragment.this.getActivity(), "Intervention N°"+resultPost.getId()+" est ajoutée ", Toast.LENGTH_LONG).show();
            showProgress(false);
            ((InterventionActivity)getActivity()).addIntervention(resultPost);
            ((InterventionActivity)getActivity()).updateInterventions();
            dismiss();
        }

    }

    // Backgroud task to post intervention
    private class ResourceGetTask extends AsyncTask<List<Long>, Void, List<ResourceType>> {

        private OnTaskCompleted listener;

        public ResourceGetTask(OnTaskCompleted listener){
            this.listener=listener;
        }

        @Override
        protected List<ResourceType> doInBackground(List<Long>... params) {
            try {

                List<ResourceType> resourcesType;
                resourcesType = new ArrayList<ResourceType>();

                List<Long> idResourcesTypes = params[0];
                Log.i("MAMH", "Size idResourcesTypes = "+idResourcesTypes.size());
                for (Long id : idResourcesTypes){
                    Log.i("MAMH", "ID = "+id);
                }
                Log.i("MAMH", "Fin Size idResourcesTypes");
                Log.i("MAMH", "getResourceTypeById");
                for(Long idRes : idResourcesTypes){

                    ResourceType rt = springService.getResourceTypeById(idRes);
                    resourcesType.add(rt);
                    Log.i("MAMH", "Label :  "+rt.getLabel());
                }
                Log.i("MAMH", "Fin getResourceTypeById ");
                return  resourcesType;

            } catch (HttpStatusCodeException e) {
                Log.e("InterventionActivity", e.getMessage(), e);

            }

            return  null;
        }

        @Override
        protected void onPostExecute(List<ResourceType> resultPost) {
            Toast.makeText(InterventionDialogFragment.this.getActivity(), "Intervention N°"+resultPost+" est ajoutée ", Toast.LENGTH_LONG).show();
            Log.i("MAMH", "Size resultPost = "+resultPost.size());
            listener.onTaskCompleted(resultPost);
        }

    }


    public void onTaskCompleted(List<ResourceType> resourcesType){
        //Intervetion
        entity.Intervention intervention;

        //Les champs text sont toujours vérifié
        intervention = new entity.Intervention(nameIntervetionEditText.getText().toString(), spinnerMap.get(codeSinistreSpinner.getSelectedItem().toString()).intValue(), Double.valueOf(latitudeEditText.getText().toString()), Double.valueOf(longitudeEditText.getText().toString()));

        Log.i("MAMH", "Lat : " + intervention.getLatitude() + ", Lng : " + intervention.getLongitude());


        List<Resource> resources = covertResourcesTypeToResources(resourcesType);

        Log.i("MAMH", "Resource");
        for (Resource res : resources){
            Log.i("MAMH", "Resource : "+res.getLabel());
        }
        Log.i("MAMH", "Fin Resource");
        //intervention.set
        intervention.setResources(resources);
        at = new InterventionPostTask().execute(intervention);
    }

    public List<Resource> covertResourcesTypeToResources(List<ResourceType> resourcesType){

        List<Resource> resourcesResult;
        resourcesResult = new ArrayList<Resource>();

        for(ResourceType rt : resourcesType){
            if(rt != null) {
                Resource rs = new Resource();
                rs.setLabel(rt.getLabel());
                rs.setState(State.validated);
                resourcesResult.add(rs);
            }
        }
        return  resourcesResult;
    }
}
