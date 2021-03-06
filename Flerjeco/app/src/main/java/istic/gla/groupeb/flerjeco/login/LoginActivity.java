package istic.gla.groupeb.flerjeco.login;

import android.animation.Animator;
import android.animation.AnimatorListenerAdapter;
import android.annotation.TargetApi;
import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.text.TextUtils;
import android.util.Log;
import android.view.KeyEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.inputmethod.EditorInfo;
import android.view.inputmethod.InputMethodManager;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.StaticData;
import istic.gla.groupeb.flerjeco.FlerjecoApplication;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.agent.interventionsList.ListInterventionsActivity;
import istic.gla.groupeb.flerjeco.codis.intervention.InterventionActivity;
import istic.gla.groupeb.flerjeco.springRest.GetAllInterventionsTask;
import istic.gla.groupeb.flerjeco.springRest.GetAllStaticDataTask;
import istic.gla.groupeb.flerjeco.springRest.IInterventionsActivity;
import istic.gla.groupeb.flerjeco.springRest.IStaticDataActivity;
import istic.gla.groupeb.flerjeco.springRest.SpringService;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;


/**
 * A login screen that offers loginNO CONTENT via email/password.
 */
public class LoginActivity extends Activity implements ISynchTool, IInterventionsActivity, IStaticDataActivity {

    private static final String TAG = LoginActivity.class.getSimpleName();

    /**
     * Keep track of the login task to ensure we can cancel it if requested.
     */
    private UserLoginTask mAuthTask = null;

    private boolean isCodis;


    // UI references.
    private EditText mLoginView;
    private EditText mPasswordView;
    private View mProgressView;
    private View mLoginFormView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_login);
        refresh();
    }

    @Override
    public void refresh() {
        // Set up the login form.
        mLoginView = (EditText) findViewById(R.id.editText_login);

        mPasswordView = (EditText) findViewById(R.id.editText_password);
        mPasswordView.setOnEditorActionListener(new TextView.OnEditorActionListener() {
            @Override
            public boolean onEditorAction(TextView textView, int id, KeyEvent keyEvent) {
                if (id == R.id.login || id == EditorInfo.IME_ACTION_SEND ) {
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
            mAuthTask = new UserLoginTask(this, login, password);
            mAuthTask.execute((Void) null);

            //new GetAllInterventionsTask(LoginActivity.this).execute();

            isCodis = ((CheckBox) findViewById(R.id.checkBox_codis)).isChecked();
            Log.i(TAG, "isCodis: " + isCodis);
            if(!isCodis)
                new GetAllStaticDataTask(this).execute();
        }
    }

    /**
     * Shows the progress UI and hides the login form.
     */
    @TargetApi(Build.VERSION_CODES.HONEYCOMB_MR2)
    public void showProgress(final boolean show) {
        //hide the keyboard
        // Check if no view has focus:
        View view = this.getCurrentFocus();
        if (view != null) {
            InputMethodManager inputManager = (InputMethodManager) this.getSystemService(Context.INPUT_METHOD_SERVICE);
            inputManager.hideSoftInputFromWindow(view.getWindowToken(), InputMethodManager.HIDE_NOT_ALWAYS);
        }
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

    @Override
    public void updateInterventions(Intervention[] interventions) {
        showProgress(false);
        if(interventions != null) {
            Intent intent;
            Bundle bundle = new Bundle();

            if (isCodis) {
                intent = new Intent(LoginActivity.this, InterventionActivity.class);
                ArrayList<Integer> tabs = new ArrayList<>();
                tabs.add(R.string.interventions);
                tabs.add(R.string.table);
                bundle.putIntegerArrayList("tabs", tabs);
            } else {
                intent = new Intent(LoginActivity.this, ListInterventionsActivity.class);
            }

            for (int i = 0; i < interventions.length; i++)
                Log.d("LoginAct", interventions[i].getName() + " - " + interventions[i].getId());

            bundle.putSerializable("interventions", interventions);

            intent.putExtras(bundle);
            startActivity(intent);
        }
    }

    @Override
    public void setStaticData(StaticData[] data) {
        FlerjecoApplication.getInstance().setStaticData(data);
    }

    @Override
    public Context getContext() {
        return getApplicationContext();
    }

    /**
     * Represents an asynchronous login/registration task used to authenticate
     * the user.
     */
    public class UserLoginTask extends AsyncTask<Void, Void, String> {

        private final String mLogin;
        private final String mPassword;
        private int count = 0;
        private LoginActivity activity;
        private SpringService service = new SpringService();

        public UserLoginTask(LoginActivity activity, String login, String password) {
            this.activity = activity;
            mLogin = login;
            mPassword = password;
        }

        public UserLoginTask(LoginActivity activity, int count, String mLogin, String mPassword) {
            this.activity = activity;
            this.count = count;
            this.mLogin = mLogin;
            this.mPassword = mPassword;
        }

        @Override
        protected String doInBackground(Void... params) {
            return service.login(mLogin, mPassword);
        }

        @Override
        protected void onPostExecute(String statusCode) {
            mAuthTask = null;
            FlerjecoApplication flerjecoApplication = FlerjecoApplication.getInstance();
            flerjecoApplication.setCodisUser(isCodis);
            flerjecoApplication.setLogin(mLogin);
            flerjecoApplication.setPassword(mPassword);

            if (statusCode.equals("200")) {
                Toast.makeText(LoginActivity.this, getString(R.string.login_successful), Toast.LENGTH_SHORT).show();
                new GetAllInterventionsTask(LoginActivity.this).execute();
            } else {
                if(count < 4) {
                    Log.w(TAG, String.format("login failed with %d try (login:%s, pwd:%s), will retry", count, mLogin, mPassword));
                    count++;
                    new UserLoginTask(activity, count, mLogin, mPassword).execute();
                } else{
                    showProgress(false);
                    Log.w(TAG, String.format("login failed definitely after %d try (login:%s, pwd:%s)", count, mLogin, mPassword));
                    if(statusCode.equals("401")){
                        Toast.makeText(LoginActivity.this, getString(R.string.login_failed), Toast.LENGTH_SHORT).show();
                        mPasswordView.setError(getString(R.string.error_incorrect_password));
                        mPasswordView.requestFocus();
                    } else{
                        Toast.makeText(LoginActivity.this, getString(R.string.error_server_down), Toast.LENGTH_SHORT).show();
                    }
                }
            }
        }

        @Override
        protected void onCancelled() {
            mAuthTask = null;
            showProgress(false);
        }
    }
}



