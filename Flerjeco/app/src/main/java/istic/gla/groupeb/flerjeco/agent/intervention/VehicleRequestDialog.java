package istic.gla.groupeb.flerjeco.agent.intervention;


import android.animation.Animator;
import android.animation.AnimatorListenerAdapter;
import android.annotation.TargetApi;
import android.content.Context;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.support.v4.app.DialogFragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.Spinner;
import android.widget.Toast;

import org.springframework.web.client.HttpStatusCodeException;

import java.util.HashMap;

import entity.Intervention;
import entity.Resource;
import entity.ResourceType;
import istic.gla.groupeb.flerjeco.FlerjecoApplication;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.springRest.GetResourceTypesTask;
import istic.gla.groupeb.flerjeco.springRest.IResourceTypesActivity;
import istic.gla.groupeb.flerjeco.springRest.SpringService;
import util.ResourceCategory;

/**
 * Created by jules on 09/04/15.
 *
 * How to use :
 *
 * //An instance of this fragment can be created and shown as a dialog:
 *
 * void showDialogRequest() {
 *     // Create the fragment and show it as a dialog.
 *     DialogFragment vehicleDialog = new VehicleRequestDialog();
 *     vehicleDialog.show(getSupportFragmentManager(), "vehicle_dialog");
 * }
 *
 * //It can also be added as content in a view hierarchy:
 *
 * FragmentTransaction ft = getSupportFragmentManager().beginTransaction();
 * DialogFragment vehicleFragment = new VehicleRequestDialog();
 * ft.add(R.id.embedded, vehicleFragment);
 * ft.commit();
 */
public class VehicleRequestDialog extends DialogFragment implements IResourceTypesActivity {

    private static final String TAG = VehicleRequestDialog.class.getSimpleName();
    private Spinner spinner;
    private HashMap<String, Long> spinnerMap;
    private Long interventionId;
    public final static String INTERVENTION = "intervention";
    private View mProgressView;
    private View mVehicleFormView;

    private GetResourceTypesTask resourceTypesTask;
    private ResourceRequestTask resourceRequestTask;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View v = inflater.inflate(R.layout.fragment_resource_request, container, false);
        interventionId = getArguments().getLong(INTERVENTION);
        spinner = (Spinner)v.findViewById(R.id.vehicle_fragment_spinner);

        mVehicleFormView = v.findViewById(R.id.vehicle_form);
        mProgressView = v.findViewById(R.id.vehicle_progress);
        getDialog().setTitle(R.string.resource_request_dialog_title);

        showProgress(true);

        if(FlerjecoApplication.getInstance().getResourceTypes() != null && FlerjecoApplication.getInstance().getResourceTypes().length > 0) {
            updateResourceTypes(FlerjecoApplication.getInstance().getResourceTypes());
        } else {
            resourceTypesTask = new GetResourceTypesTask(this);
            resourceTypesTask.execute();
        }

        //ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(getActivity(),
        //        R.array.vehicles_array, android.R.layout.simple_spinner_item);
        //adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        //spinner.setAdapter(adapter);

        final Spinner spinner1 = spinner;
        Button button = (Button)v.findViewById(R.id.vehicle_fragment_button);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                validate(spinner1.getSelectedItem().toString());
            }
        });

        return v;
    }

    public void validate(String vehicle) {
        Toast.makeText(getActivity(), vehicle, Toast.LENGTH_SHORT).show();
        resourceRequestTask = new ResourceRequestTask();
        resourceRequestTask.execute(interventionId, vehicle);
    }

    @Override
    public void onPause() {
        super.onPause();
        if(resourceTypesTask != null)
            resourceTypesTask.cancel(true);
        else if (resourceRequestTask != null)
            resourceRequestTask.cancel(true);
    }

    /**
     * Shows the progress UI
     */
    @TargetApi(Build.VERSION_CODES.HONEYCOMB_MR2)
    public void showProgress(final boolean show) {
        // On Honeycomb MR2 we have the ViewPropertyAnimator APIs, which allow
        // for very easy animations. If available, use these APIs to fade-in
        // the progress spinner.
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB_MR2) {
            int shortAnimTime = getResources().getInteger(android.R.integer.config_shortAnimTime);

            mVehicleFormView.setVisibility(show ? View.GONE : View.VISIBLE);
            mVehicleFormView.animate().setDuration(shortAnimTime).alpha(
                    show ? 0 : 1).setListener(new AnimatorListenerAdapter() {
                @Override
                public void onAnimationEnd(Animator animation) {
                    mVehicleFormView.setVisibility(show ? View.GONE : View.VISIBLE);
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
            mVehicleFormView.setVisibility(show ? View.GONE : View.VISIBLE);
        }
    }

    @Override
    public void updateResourceTypes(ResourceType[] resourceTypes) {
        if(resourceTypes != null && resourceTypes.length > 0 ) {
            String[] spinnerArray = new String[resourceTypes.length];
            spinnerMap = new HashMap();
            FlerjecoApplication.getInstance().setResourceTypes(resourceTypes);
            for (int i = 0; i < resourceTypes.length; i++) {
                spinnerMap.put(resourceTypes[i].getLabel(), resourceTypes[i].getId());
                if (resourceTypes[i].getCategory() != null && resourceTypes[i].getCategory().equals(ResourceCategory.vehicule))
                    ;
                spinnerArray[i] = resourceTypes[i].getLabel();
            }

            ArrayAdapter<String> spinnerAdapter = new ArrayAdapter(VehicleRequestDialog.this.getActivity(), android.R.layout.simple_spinner_item, spinnerArray);
            spinnerAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
            spinner.setAdapter(spinnerAdapter);
        }
        showProgress(false);
    }

    @Override
    public Context getContext() {
        return null;
    }

    private class ResourceRequestTask extends AsyncTask<Object, Void, Intervention> {
        private SpringService service = new SpringService();
        @Override
        protected Intervention doInBackground(Object... params) {
            try {
                return service.requestVehicle(params);
            } catch (HttpStatusCodeException e) {
                Log.e(TAG, e.getMessage(), e);
            }

            return null;
        }

        @Override
        protected void onPostExecute(Intervention intervention) {
            if(null == intervention) {
                Toast.makeText(getActivity(), "Update impossible", Toast.LENGTH_SHORT).show();
                this.cancel(true);
                return;
            }
            Log.i(TAG, "Resource requested for intervention: " + intervention.getName());
            for(Resource res : intervention.getResources()) {
                Log.i(TAG, "Resource : "+res.getLabel());
            }
            if(getActivity() != null) {
                Log.i(TAG, "getActivity not null");
                ((AgentInterventionActivity) getActivity()).updateIntervention(intervention);
            }
            dismiss();
        }
    }
}
