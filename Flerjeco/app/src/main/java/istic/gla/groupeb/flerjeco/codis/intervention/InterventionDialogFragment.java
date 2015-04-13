package istic.gla.groupeb.flerjeco.codis.intervention;

import android.app.DialogFragment;
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

import java.util.HashMap;
import java.util.List;

import entity.IncidentCode;
import entity.Resource;
import entity.ResourceType;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.springRest.SpringService;


public class InterventionDialogFragment extends DialogFragment implements OnTaskCompleted{


    Spinner codeSinistreSpinner;

    //Button of intervention creation
    Button intervention_creation_button;

    //Text fields
    EditText nameIntervetionEditText;
    EditText latitudeEditText;
    EditText longitudeEditText;

    SpringService springService = new SpringService();
    String[] spinnerArray;
    ArrayAdapter<String> spinnerAdapter;

    private HashMap<String, Long> spinnerMap;
    private HashMap<String, List<Long>> resourceTypeMap;


    boolean data_local = false;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View v = inflater.inflate(R.layout.activity_intervention, container, false);


        //Set up code sinistre list
        codeSinistreSpinner = (Spinner) v.findViewById(R.id.CodeSinistreSpinner);
        nameIntervetionEditText = (EditText) v.findViewById(R.id.nameInterventionEditText);
        latitudeEditText = (EditText) v.findViewById(R.id.lat);
        longitudeEditText = (EditText) v.findViewById(R.id.lng);

        // Set up Button of intervention creation
        intervention_creation_button = (Button) v.findViewById(R.id.intervention_button);

        if (data_local) {
            spinnerArray = new String[]{"SAP", "AVP", "FHA", "MEEEEE"};
            spinnerMap = new HashMap<>();
            int i = 0;
            for (String code : spinnerArray) {
                spinnerMap.put(code, Long.valueOf(i));
                resourceTypeMap.put(code,null);
                i++;
            }
            i = 0;
            spinnerAdapter = new ArrayAdapter<String>(InterventionDialogFragment.this.getActivity(), android.R.layout.simple_spinner_item, spinnerArray);
            spinnerAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
            codeSinistreSpinner.setAdapter(spinnerAdapter);
        } else new HttpRequestTask().execute();


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
                return true;
            }


            @Override
            public void onClick(View v) {
                if (validate()) {
                    //String idIncidentCode, double latitude, double longitude, String name
                    Log.i("MAMH", spinnerMap.get(codeSinistreSpinner.getSelectedItem().toString()) + "");


                    //Intervetion
                    entity.Intervention intervention;

                    intervention = new entity.Intervention(nameIntervetionEditText.getText().toString(), spinnerMap.get(codeSinistreSpinner.getSelectedItem().toString()).intValue(), Double.valueOf(latitudeEditText.getText().toString()), Double.valueOf(longitudeEditText.getText().toString()));

                    Log.i("MAMH", "Lat : " + intervention.getLatitude() + ", Lng : " + intervention.getLongitude());

                    List<Resource> resources;

                    AsyncTask at = new InterventionPostTask().execute(intervention);

                }
            }
        });

        return v;
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
            spinnerMap = new HashMap();
            if(codes != null && codes.length > 0 ) {
                int i = 0;
                spinnerArray = new String[codes.length];
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
    private class InterventionPostTask extends AsyncTask<entity.Intervention, Void, Long> {

        @Override
        protected Long doInBackground(entity.Intervention... params) {
            try {
                return  springService.postIntervention(params[0]);

            } catch (HttpStatusCodeException e) {
                Log.e("InterventionActivity", e.getMessage(), e);
                return 0L;
            }

        }

        @Override
        protected void onPostExecute(Long resultPost) {
            Toast.makeText(InterventionDialogFragment.this.getActivity(), "Intervention N°"+resultPost+" est ajoutée ", Toast.LENGTH_LONG).show();

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

                List<Long> idResourcesTypes = params[0];
                for(int i = 0; i < idResourcesTypes.size(); i++){

                }

            } catch (HttpStatusCodeException e) {
                Log.e("InterventionActivity", e.getMessage(), e);

            }

            return  null;
        }

        @Override
        protected void onPostExecute(List<ResourceType> resultPost) {
            Toast.makeText(InterventionDialogFragment.this.getActivity(), "Intervention N°"+resultPost+" est ajoutée ", Toast.LENGTH_LONG).show();

        }

    }


    public void onTaskCompleted(){
        //TO DO ADD

    }
}
