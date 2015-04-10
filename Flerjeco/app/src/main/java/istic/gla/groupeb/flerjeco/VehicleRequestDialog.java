package istic.gla.groupeb.flerjeco;


import android.os.AsyncTask;
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

import entity.ResourceType;
import istic.gla.groupeb.flerjeco.springRest.SpringService;

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
public class VehicleRequestDialog extends DialogFragment {

    private Spinner spinner;
    private HashMap<String, Long> spinnerMap;
    private String resourceType;
    private Long intervention;
    public final static String INTERVENTION = "intervention";

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View v = inflater.inflate(R.layout.fragment_vehicle, container, false);
        intervention = getArguments().getLong(INTERVENTION);
        spinner = (Spinner)v.findViewById(R.id.vehicle_fragment_spinner);

        new ResourceTypesTask().execute();
        //ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(getActivity(),
        //        R.array.vehicles_array, android.R.layout.simple_spinner_item);
        //adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        //spinner.setAdapter(adapter);

        final Spinner spinner1 = spinner;
        Button button = (Button)v.findViewById(R.id.vehicle_fragment_button);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                validate(spinnerMap.get(resourceType = spinner1.getSelectedItem().toString()));
            }
        });

        return v;
    }

    public void validate(Long vehicle) {
        this.dismiss();
        new ResourceRequestTask().execute(intervention, vehicle);
        Toast.makeText(getActivity(), "" + vehicle, Toast.LENGTH_SHORT).show();
    }

    private class ResourceTypesTask extends AsyncTask<Void, Void, ResourceType[]> {

        @Override
        protected ResourceType[] doInBackground(Void... params) {
            try {
                ResourceType[] resourceTypes = new SpringService().resourceTypes();
                return resourceTypes;

            } catch (HttpStatusCodeException e) {
                Log.e("VehicleRequestDialog", e.getMessage(), e);
            }

            return null;
        }

        @Override
        protected void onPostExecute(ResourceType[] resources) {
            String[] spinnerArray = new String[resources.length];
            spinnerMap = new HashMap();
            if(resources != null && resources.length > 0 ) {
                for (int i = 0; i < resources.length; i++) {
                    spinnerMap.put(resources[i].getLabel(), resources[i].getId());
                    spinnerArray[i] = resources[i].getLabel();
                }
            }

            ArrayAdapter<String> spinnerAdapter = new ArrayAdapter(VehicleRequestDialog.this.getActivity(), android.R.layout.simple_spinner_item, spinnerArray);
            spinnerAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
            spinner.setAdapter(spinnerAdapter);

        }

    }

    private class ResourceRequestTask extends AsyncTask<Long, Void, Long> {

        @Override
        protected Long doInBackground(Long... params) {
            try {
                Long id = new SpringService().requestVehicle(params);
                return id;

            } catch (HttpStatusCodeException e) {
                Log.e("VehicleRequestDialog", e.getMessage(), e);
            }

            return null;
        }

        @Override
        protected void onPostExecute(Long id) {
            Log.i("VehicleRequestDialog","Request posted");
        }

    }
}
