package istic.gla.groupeb.flerjeco;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.graphics.drawable.Drawable;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
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


public class InterventionActivity extends ActionBarActivity {

    Spinner codeSinistreSpinner;
    LinearLayout vehicules;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_intervention);

        codeSinistreSpinner = (Spinner) findViewById(R.id.CodeSinistreSpinner);

        vehicules = (LinearLayout) findViewById(R.id.vehiculeLayout);

        codeSinistreSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parentView, View selectedItemView, int position, long id) {
                String[] codes = getResources().getStringArray(R.array.code);
                if(codes != null) {
                    Toast.makeText(InterventionActivity.this, "Code selected = " + codes[(int) id], Toast.LENGTH_LONG).show();
                    AlertDialog.Builder builder = new AlertDialog.Builder(InterventionActivity.this);
                    builder.setTitle("Véhicules");


                    final String[] items = {"airplanes", "animals", "cars", "colors", "flowers", "letters", "monsters", "numbers", "shapes", "smileys", "sports", "stars" };

// Instead of String[] items, Here you can also use ArrayList for your custom object..

                    ListAdapter adapter = new ArrayAdapter(
                            getApplicationContext(), R.layout.list_row, items) {

                        ViewHolder holder;
                        Drawable icon;

                        class ViewHolder {
                            ImageView icon;
                            TextView title;
                        }

                        public View getView(int position, View convertView,
                                            ViewGroup parent) {
                            final LayoutInflater inflater = (LayoutInflater) getApplicationContext()
                                    .getSystemService(
                                            Context.LAYOUT_INFLATER_SERVICE);

                            if (convertView == null) {
                                convertView = inflater.inflate(
                                        R.layout.list_row, null);

                                holder = new ViewHolder();
                                holder.icon = (ImageView) convertView
                                        .findViewById(R.id.icon);
                                holder.title = (TextView) convertView
                                        .findViewById(R.id.title);
                                convertView.setTag(holder);
                            } else {
                                // view already defined, retrieve view holder
                                holder = (ViewHolder) convertView.getTag();
                            }

                            //Drawable drawable = getResources().getDrawable(R.drawable.list_icon); //this is an image from the drawables folder

                            holder.title.setText(items[position]);
                            //holder.icon.setImageDrawable(drawable);

                            return convertView;
                        }
                    };

                    builder.setAdapter(adapter,
                            new DialogInterface.OnClickListener() {
                                @Override
                                public void onClick(DialogInterface dialog,
                                                    int item) {
                                    Toast.makeText(InterventionActivity.this, "You selected: " + items[item],Toast.LENGTH_LONG).show();
                                    dialog.dismiss();
                                }
                            });
                    AlertDialog alert = builder.create();
                    alert.show();
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
}
