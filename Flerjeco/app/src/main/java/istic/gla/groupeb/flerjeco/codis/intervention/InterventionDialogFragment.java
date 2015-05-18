package istic.gla.groupeb.flerjeco.codis.intervention;

import android.animation.Animator;
import android.animation.AnimatorListenerAdapter;
import android.annotation.TargetApi;
import android.content.Context;
import android.location.Address;
import android.location.Geocoder;
import android.os.Build;
import android.support.v4.app.DialogFragment;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.Spinner;
import android.widget.ToggleButton;


import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;

import entity.IncidentCode;
import entity.Intervention;
import entity.Resource;
import entity.ResourceType;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.springRest.GetAllIncidentCodeTask;
import istic.gla.groupeb.flerjeco.springRest.GetResourceTypeLabelsTask;
import istic.gla.groupeb.flerjeco.springRest.IIncidentCode;
import istic.gla.groupeb.flerjeco.springRest.IInterventionActivity;
import istic.gla.groupeb.flerjeco.springRest.IResourceTypeLabelsActivity;
import istic.gla.groupeb.flerjeco.springRest.InterventionPostTask;
import util.State;

/**
 * Fragment used for the creation of the intervention of firefighters
 * @see android.support.v4.app.DialogFragment
 */
public class InterventionDialogFragment extends DialogFragment
        implements IIncidentCode, IInterventionActivity, IResourceTypeLabelsActivity {
    private static final String TAG = InterventionDialogFragment.class.getSimpleName();

    Spinner codeSinistreSpinner;

    //Button of intervention creation
    Button intervention_creation_button;

    //Text fields
    EditText nameInterventionEditText;
    EditText addressEditText;
    EditText latitudeEditText;
    EditText longitudeEditText;

    LinearLayout latLongLinearLayout;

    private View mProgressView;
    private View mCreateFormView;

    String[] spinnerArray;
    ArrayAdapter<String> spinnerAdapter;

    private HashMap<String, Long> spinnerMap;
    private HashMap<String, List<Long>> resourceTypeMap;


    boolean addressOrCoordinates=true;
    private GetAllIncidentCodeTask incidentCodesTask;
    private InterventionPostTask interventionPostTask;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View v = inflater.inflate(R.layout.fragment_create_intervention, container, false);
        getDialog().setTitle(R.string.title_fragment_create_intervention);

        mCreateFormView = v.findViewById(R.id.intervention_scroll);
        mProgressView = v.findViewById(R.id.create_progress);

        //Set up code sinistre list
        codeSinistreSpinner = (Spinner) v.findViewById(R.id.CodeSinistreSpinner);
        nameInterventionEditText = (EditText) v.findViewById(R.id.nameInterventionEditText);
        addressEditText = (EditText) v.findViewById(R.id.address);
        latitudeEditText = (EditText) v.findViewById(R.id.latitude);
        longitudeEditText = (EditText) v.findViewById(R.id.longitude);
        latLongLinearLayout = (LinearLayout) v.findViewById(R.id.lat_long);


        //  Set up Button of intervention creation
        intervention_creation_button = (Button) v.findViewById(R.id.intervention_button);

        //Selection of registered claims codes in the database and the list of identifiers of resourceType
        showProgress(true);
        incidentCodesTask = new GetAllIncidentCodeTask(this);
        incidentCodesTask.execute();

        //  add button listener
        intervention_creation_button.setOnClickListener(new View.OnClickListener() {

            public boolean validate() {
                Log.i(TAG, "Validate addressOrCoordinates : "+addressOrCoordinates);
                if(addressOrCoordinates) {
                    if (addressEditText.getText().toString().length() == 0) {
                        addressEditText.setError(getString(R.string.error_field_required));
                        return false;
                    }
                }
                else {
                    if (latitudeEditText.getText().toString().length() == 0) {
                        latitudeEditText.setError(getString(R.string.error_field_required));
                        return false;
                    }
                    if (longitudeEditText.getText().toString().length() == 0) {
                        longitudeEditText.setError(getString(R.string.error_field_required));
                        return false;
                    }
                }
                Log.i(TAG, "Validate OK");
                showProgress(true);
                return true;
            }


            @Override
            public void onClick(View v) {
                if (validate()) {
                    Log.i(TAG, spinnerMap.get(codeSinistreSpinner.getSelectedItem().toString()) + "");

                    //  Selecting sinister code selected resources from the list of identifiers of resourceType already recovered
                    //  Note: at the end of this task in the background, it creates intervention
                    new GetResourceTypeLabelsTask(InterventionDialogFragment.this).execute(resourceTypeMap.get(codeSinistreSpinner.getSelectedItem().toString()));

                }
            }
        });

        ToggleButton toggle = (ToggleButton) v.findViewById(R.id.address_or_coordinate);
        toggle.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    addressOrCoordinates = true;
                    addressEditText.setVisibility(View.VISIBLE);
                    latLongLinearLayout.setVisibility(View.GONE);
                } else {
                    addressOrCoordinates = false;
                    latLongLinearLayout.setVisibility(View.VISIBLE);
                    addressEditText.setVisibility(View.GONE);
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
    }

    @Override
    public void updateIntervention(Intervention intervention) {
        showProgress(false);
        if(intervention != null) {
            Log.i(TAG, "updateIntervention success");
            ((InterventionActivity)getActivity()).addIntervention(intervention);
            ((InterventionActivity)getActivity()).updateInterventions();
            dismiss();
        }
    }

    @Override
    public void updateResourceTypes(ResourceType[] resourceTypes) {
        //Intervention
        entity.Intervention intervention;
        //Address of the intervention
        Address address = new Address(Locale.getDefault());

        if(addressOrCoordinates) {
            // Create intervention with Address
            address = getCoordinates();
        }
        else {
            // Create intervention with Coordinates
            address.setLatitude(Double.parseDouble(latitudeEditText.getText().toString()));
            address.setLongitude(Double.parseDouble(longitudeEditText.getText().toString()));
        }

        if (null == address) {
            //Toast.makeText(getActivity(), R.string.geocoder_failed, Toast.LENGTH_LONG);
            addressEditText.setError(getString(R.string.geocoder_failed));
            addressEditText.requestFocus();
            showProgress(false);
            return;
        }

        //Les champs text sont toujours vérifié
        intervention = new entity.Intervention(nameInterventionEditText.getText().toString(),
                spinnerMap.get(codeSinistreSpinner.getSelectedItem().toString()).intValue(),
                address.getLatitude(), address.getLongitude());
        Log.i(TAG, "updateResourceTypes size : "+ resourceTypes.length);
        List<Resource> resources = convertResourcesTypeToResources(resourceTypes);

        //intervention.set
        intervention.setResources(resources);
        interventionPostTask = new InterventionPostTask(this);
        interventionPostTask.execute(intervention);
    }

    @Override
    public void getIncidentCode(IncidentCode[] incidentCodes) {
        showProgress(false);
        if(incidentCodes != null && incidentCodes.length > 0 ) {
            int i = 0;
            spinnerArray = new String[incidentCodes.length];
            resourceTypeMap = new HashMap<>();
            spinnerMap = new HashMap();
            for (IncidentCode code : incidentCodes) {
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

    @Override
    public Context getContext() {
        return getActivity().getBaseContext();
    }

    public Address getCoordinates() {
        Geocoder geocoder = new Geocoder(getActivity(), Locale.getDefault());
        int nbTry = 0;
        List<Address> addressList = new ArrayList<>();
        try {
            addressList = geocoder.getFromLocationName(addressEditText.getText().toString(),1);
            while (addressList.size()==0 && nbTry<10) {
                Log.i(TAG, "Try number : "+nbTry);
                addressList = geocoder.getFromLocationName(addressEditText.getText().toString(), 1);
                nbTry++;
            }
            if (addressList.size()>0) {
                return addressList.get(0);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        return null;
    }

    public List<Resource> convertResourcesTypeToResources(ResourceType[] resourcesType){
        List<Resource> resourcesResult= new ArrayList<>();

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
