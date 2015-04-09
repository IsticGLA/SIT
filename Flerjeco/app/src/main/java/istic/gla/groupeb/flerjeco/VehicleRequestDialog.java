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

import istic.gla.groupeb.flerjeco.icons.Vehicle;
import istic.gla.groupeb.flerjeco.springRest.IncidentCode;
import istic.gla.groupeb.flerjeco.springRest.ResourceType;
import istic.gla.groupeb.flerjeco.springRest.SpringService;

/**
 * Created by jules on 09/04/15.
 *
 * How to use :
 *
 * //An instance of this fragment can be created and shown as a dialog:
 *
 * void showDialog() {
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

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View v = inflater.inflate(R.layout.fragment_vehicle, container, false);

        spinner = (Spinner)v.findViewById(R.id.vehicle_fragment_spinner);

        new HttpRequestTask().execute();
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
        this.dismiss();
        //TODO Request rest vehicle
        //((MyActivity)getActivity()).createRequest();
        Toast.makeText(getActivity(), vehicle, Toast.LENGTH_SHORT).show();
    }

    private class HttpRequestTask extends AsyncTask<Void, Void, ResourceType[]> {

        @Override
        protected ResourceType[] doInBackground(Void... params) {
            try {
                ResourceType[] resourceTypes = new SpringService().resourceTypes();
                return  resourceTypes;

            } catch (HttpStatusCodeException e) {
                Log.e("VehicleRequestDialog", e.getMessage(), e);
            }

            return null;
        }

        @Override
        protected void onPostExecute(ResourceType[] resources) {
            String[] spinnerArray = null;
            if(resources != null && resources.length > 0 ) {
                int i = 0;
                spinnerArray = new String[resources.length];
                for (ResourceType resource : resources) {
                    if(resource != null) {
                        spinnerArray[i] = resource.getLabel();
                        i++;
                    }
                }

            }

            ArrayAdapter<String> spinnerAdapter = new ArrayAdapter(VehicleRequestDialog.this.getActivity(), android.R.layout.simple_spinner_item, spinnerArray);
            spinnerAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
            spinner.setAdapter(spinnerAdapter);

        }

    }
}
