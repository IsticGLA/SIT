package istic.gla.groupeb.flerjeco;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.graphics.drawable.Drawable;
import android.os.AsyncTask;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.ListAdapter;
import android.widget.ListView;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import org.springframework.web.client.HttpStatusCodeException;

import entity.IncidentCode;
import istic.gla.groupeb.flerjeco.springRest.Intervention;
import istic.gla.groupeb.flerjeco.springRest.SpringService;


public class InterventionActivity extends ActionBarActivity {


    Spinner codeSinistreSpinner;

    //Button of intervention creation
    Button intervention_creation_button;

    //Text fields
    EditText nameIntervetionEditText ;
    EditText latitudeEditText ;
    EditText longitudeEditText ;

    SpringService springService = new SpringService();
    String[] spinnerArray;
    ArrayAdapter<String> spinnerAdapter;

    //Intervetion
    Intervention intervention;
    boolean data_local = false;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_intervention);

        //Set up code sinistre list
        codeSinistreSpinner = (Spinner) findViewById(R.id.CodeSinistreSpinner);
        nameIntervetionEditText = (EditText) findViewById(R.id.nameInterventionEditText);
        latitudeEditText = (EditText) findViewById(R.id.lat);
        longitudeEditText = (EditText) findViewById(R.id.lng);

        // Set up Button of intervention creation
        intervention_creation_button =  ( Button ) findViewById(R.id.intervention_button);

        if(data_local) {
            spinnerArray = new String[]{"SAP", "AVP", "FHA", "MEEEEE"};
            spinnerAdapter = new ArrayAdapter<String>(InterventionActivity.this, android.R.layout.simple_spinner_item,spinnerArray);
            spinnerAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
            codeSinistreSpinner.setAdapter(spinnerAdapter);
        }
        else new HttpRequestTask().execute();



        // add listener to spinner list
         codeSinistreSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parentView, View selectedItemView, int position, long id) {

                if (spinnerArray != null) {
                    Toast.makeText(InterventionActivity.this, "Code selected = " + spinnerArray[(int) id], Toast.LENGTH_LONG).show();

                }

            }

            @Override
            public void onNothingSelected(AdapterView<?> parentView) {
                // your code here
            }

        });
        // add button listener
        intervention_creation_button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                //String idIncidentCode, double latitude, double longitude, String name
                intervention = new Intervention(codeSinistreSpinner.getSelectedItem().toString(),Double.valueOf(latitudeEditText.getText().toString()), Double.valueOf(longitudeEditText.getText().toString()) , nameIntervetionEditText.getText().toString());
                Log.i("MAMH", intervention.toString());
                //new InterventionPostTask().execute();
            }
        });

    }


    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_intervention, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
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
                for (IncidentCode code : codes) {
                    if(code != null) {
                        spinnerArray[i] = code.getCode();
                        i++;
                    }
                    }

            }

            spinnerAdapter = new ArrayAdapter<String>(InterventionActivity.this, android.R.layout.simple_spinner_item,spinnerArray);
            spinnerAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
            codeSinistreSpinner.setAdapter(spinnerAdapter);

        }

    }


    // Backgroud task to post intervention
    private class InterventionPostTask extends AsyncTask<entity.Intervention, Void, Boolean> {

        @Override
        protected Boolean doInBackground(entity.Intervention... params) {
            try {
                return  springService.postInterventionTest(params[0]);

            } catch (HttpStatusCodeException e) {
                Log.e("InterventionActivity", e.getMessage(), e);
                return false;
            }

        }

        @Override
        protected void onPostExecute(Boolean resultPost) {
            if(resultPost) Toast.makeText(InterventionActivity.this, "Intervention ajoutée", Toast.LENGTH_LONG).show();

        }

    }
}
