package istic.gla.groupeb.flerjeco.login;

import android.animation.Animator;
import android.animation.AnimatorListenerAdapter;
import android.annotation.TargetApi;
import android.app.Activity;
import android.content.Intent;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.os.Messenger;
import android.text.TextUtils;
import android.util.Log;
import android.view.KeyEvent;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.inputmethod.EditorInfo;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;


import org.springframework.web.client.HttpStatusCodeException;

import entity.Intervention;
import entity.ResourceType;
import istic.gla.groupeb.flerjeco.ISynchTool;
import entity.StaticData;
import istic.gla.groupeb.flerjeco.MyApp;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.agent.interventionsList.ListInterventionsActivity;
import istic.gla.groupeb.flerjeco.codis.intervention.InterventionActivity;
import istic.gla.groupeb.flerjeco.springRest.SpringService;
import istic.gla.groupeb.flerjeco.synch.SynchService;


/**
 * A login screen that offers login via email/password.
 */
public class LoginActivity extends Activity implements ISynchTool{
    private static final String TAG = LoginActivity.class.getSimpleName();


    /**
     * Keep track of the login task to ensure we can cancel it if requested.
     */
    private UserLoginTask mAuthTask = null;

    // UI references.
    private EditText mLoginView;
    private EditText mPasswordView;
    private View mProgressView;
    private View mLoginFormView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_login);

        Intent i=new Intent(this, SynchService.class);
        i.putExtra("handler", new Messenger(this.handler));

        DisplaySynch displaySynch = new DisplaySynch() {
            @Override
            public void ctrlDisplay() {
                display();
            }
        };

        i.putExtra("displaySynch", displaySynch);

        Log.i("MAMH", i.toString());
        this.startService(i);

        display();
    }



    @Override
    public void display() {


        Log.i("MAMH", "LoginActivity display");

        new ResourceTypeSynch().execute();

        // Set up the login form.
        mLoginView = (EditText) findViewById(R.id.editText_login);

        mPasswordView = (EditText) findViewById(R.id.editText_password);
        mPasswordView.setOnEditorActionListener(new TextView.OnEditorActionListener() {
            @Override
            public boolean onEditorAction(TextView textView, int id, KeyEvent keyEvent) {
                if (id == R.id.login || id == EditorInfo.IME_NULL) {
                    attemptLogin();
                    return true;
                }
                return false;
            }
        });

        Button mLoginButton = (Button) findViewById(R.id.button_connexion);
        mLoginButton.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View view) {
                attemptLogin();
            }
        });

        mLoginFormView = findViewById(R.id.login_form);
        mProgressView = findViewById(R.id.login_progress);
    }


    // Backgroud task to get notify
    private class ResourceTypeSynch extends AsyncTask<entity.Intervention, Void, ResourceType> {

        @Override
        protected ResourceType doInBackground(entity.Intervention... params) {
            try {

                SpringService springService = new SpringService();

                return  springService.getResourceTypeById(1L);
            } catch (HttpStatusCodeException e) {
                Log.e("InterventionActivity", e.getMessage(), e);

            }
            return  null;

        }

        @Override
        protected void onPostExecute(ResourceType resultPost) {

            //TODO
           /* if(resultPost != null)
                Toast.makeText(LoginActivity.this, "Label est "+resultPost.getLabel(), Toast.LENGTH_LONG).show();
            else Toast.makeText(LoginActivity.this, "Label est null", Toast.LENGTH_LONG).show();*/

        }

    }



    Handler handler=new Handler()
    {
        @Override
        public void handleMessage(Message msg) {
            //get data from msg


            String result=msg.getData().getString("result");

            Log.d("xxxxx", "get data " + result);


            super.handleMessage(msg);
        }
    };

    /**
     * Attempts to sign in or register the account specified by the login form.
     * If there are form errors (invalid email, missing fields, etc.), the
     * errors are presented and no actual login attempt is made.
     */
    public void attemptLogin() {

        // Reset errors.
        mLoginView.setError(null);
        mPasswordView.setError(null);

        // Store values at the time of the login attempt.
        String login = mLoginView.getText().toString();
        String password = mPasswordView.getText().toString();

        boolean cancel = false;
        View focusView = null;


        // Check for a valid password, if the user entered one.
        if (TextUtils.isEmpty(password)) {
            mPasswordView.setError(getString(R.string.error_field_required));
            focusView = mPasswordView;
            cancel = true;
        }

        // Check for a valid email address.
        if (TextUtils.isEmpty(login)) {
            mLoginView.setError(getString(R.string.error_field_required));
            focusView = mLoginView;
            cancel = true;
        }

        if (cancel) {
            // There was an error; don't attempt login and focus the first
            // form field with an error.
            focusView.requestFocus();
        } else {
            // Show a progress spinner, and kick off a background task to
            // perform the user login attempt.
            showProgress(true);
            mAuthTask = new UserLoginTask(login, password);
            mAuthTask.execute((Void) null);
        }
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

            mLoginFormView.setVisibility(show ? View.GONE : View.VISIBLE);
            mLoginFormView.animate().setDuration(shortAnimTime).alpha(
                    show ? 0 : 1).setListener(new AnimatorListenerAdapter() {
                @Override
                public void onAnimationEnd(Animator animation) {
                    mLoginFormView.setVisibility(show ? View.GONE : View.VISIBLE);
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
            mLoginFormView.setVisibility(show ? View.GONE : View.VISIBLE);
        }
    }

    /**
     * Represents an asynchronous login/registration task used to authenticate
     * the user.
     */
    public class UserLoginTask extends AsyncTask<Void, Void, Boolean> {

        private final String mLogin;
        private final String mPassword;
        private String statusCode;

        UserLoginTask(String login, String password) {
            mLogin = login;
            mPassword = password;
            statusCode = "Init";
        }

        @Override
        protected Boolean doInBackground(Void... params) {
            Log.i(TAG, "doInBackground start");

            SpringService service = new SpringService();
            statusCode = service.login(mLogin, mPassword);

            Log.i(TAG, "doInBackground end");
            if(statusCode.equals("200")) {
                return true;
            }
            else {
                return false;
            }
        }

        @Override
        protected void onPostExecute(final Boolean success) {
            mAuthTask = null;
            showProgress(false);

            MyApp myApp = MyApp.getInstance();
            boolean isCodis = ((CheckBox) findViewById(R.id.checkBox_codis)).isChecked();
            myApp.setCodisUser(isCodis);
            myApp.setLogin(mLogin);
            myApp.setPassword(mPassword);
            Log.i(TAG, "isCodis "+isCodis);

            if (success) {
                Toast.makeText(LoginActivity.this, getString(R.string.login_successful), Toast.LENGTH_LONG).show();
                Intent intent;
                showProgress(true);
                GetAllInterventionTask mGetAllTask = new GetAllInterventionTask(isCodis);
                mGetAllTask.execute((Void) null);
            } else {
                Toast.makeText(LoginActivity.this, getString(R.string.login_failed), Toast.LENGTH_LONG).show();
                mPasswordView.setError(getString(R.string.error_incorrect_password));
                mPasswordView.requestFocus();
            }
        }

        @Override
        protected void onCancelled() {
            mAuthTask = null;
            showProgress(false);
        }
    }

    /**
     * Represents an asynchronous login/registration task used to authenticate
     * the user.
     */
    public class GetAllInterventionTask extends AsyncTask<Void, Void, Boolean> {

        private Intervention[] interventionTab;
        private StaticData[] staticDataTab;
        private boolean isCodis;

        public GetAllInterventionTask(boolean isCodis) {
            this.isCodis = isCodis;
        }

        @Override
        protected Boolean doInBackground(Void... params) {
            SpringService service = new SpringService();
            interventionTab = service.getAllInterventions();
            Log.i(TAG, "interventionTab size : "+interventionTab.length);
            Log.i(TAG, "doInBackground end");
            return true;
        }

        @Override
        protected void onPostExecute(final Boolean success) {
            showProgress(false);
            Intent intent;
            if(isCodis) {
                intent = new Intent(LoginActivity.this, InterventionActivity.class);
            }
            else {
                intent = new Intent(LoginActivity.this, ListInterventionsActivity.class);
            }

            Bundle bundle = new Bundle();
            for(int i = 0; i < interventionTab.length; i++)
                Log.d("LoginAct", interventionTab[i].getName() + " - " + interventionTab[i].getId());

            bundle.putSerializable("interventions", interventionTab);

            intent.putExtras(bundle);
            startActivity(intent);
        }
    }
}



