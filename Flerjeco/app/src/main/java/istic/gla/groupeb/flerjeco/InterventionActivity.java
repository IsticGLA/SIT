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
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.ListAdapter;
import android.widget.ListView;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import org.springframework.web.client.HttpStatusCodeException;

import istic.gla.groupeb.flerjeco.springRest.IncidentCode;
import istic.gla.groupeb.flerjeco.springRest.SpringService;


public class InterventionActivity extends ActionBarActivity {

    Spinner codeSinistreSpinner;
    SpringService springService = new SpringService();
    String[] spinnerArray;
    ArrayAdapter<String> spinnerAdapter;

    boolean data_local = false;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_intervention);

        codeSinistreSpinner = (Spinner) findViewById(R.id.CodeSinistreSpinner);


        if(data_local) {
            spinnerArray = new String[]{"SAP", "AVP", "FHA", "MEEEEE"};
            spinnerAdapter = new ArrayAdapter<String>(InterventionActivity.this, android.R.layout.simple_spinner_item,spinnerArray);
            spinnerAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
            codeSinistreSpinner.setAdapter(spinnerAdapter);
        }
        else new HttpRequestTask().execute();



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

    private class HttpRequestTask extends AsyncTask<Void, Void, IncidentCode[]> {

        @Override
        protected IncidentCode[] doInBackground(Void... params) {
            try {
                IncidentCode[] codes = springService.codeSinistreClientTest();
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
}
