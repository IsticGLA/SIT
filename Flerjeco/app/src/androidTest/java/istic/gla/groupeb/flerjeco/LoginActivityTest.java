package istic.gla.groupeb.flerjeco;


import android.content.Intent;
import android.test.ActivityUnitTestCase;
import android.test.suitebuilder.annotation.MediumTest;
import android.widget.Button;
import android.widget.EditText;

import login.LoginActivity;

/**
 * Created by corentin on 09/04/15.
 */
public class LoginActivityTest extends ActivityUnitTestCase<LoginActivity> {
    private Intent mLoginIntent;

    public LoginActivityTest() {
        super(LoginActivity.class);
    }

    @Override
    protected void setUp() throws Exception {
        super.setUp();
        //Create an intent to launch target Activity
        mLoginIntent = new Intent(getInstrumentation().getTargetContext(),
                LoginActivity.class);
    }

    /**
     * Tests the preconditions of this test fixture.
     */
    @MediumTest
    public void testPreconditions() {
        //Start the activity under test in isolation, without values for savedInstanceState and
        //lastNonConfigurationInstance
        startActivity(mLoginIntent, null, null);
        final Button LoginButton = (Button) getActivity().findViewById(R.id.button_connexion);

        assertNotNull("mLaunchActivity is null", getActivity());
        assertNotNull("mLaunchNextButton is null", LoginButton);
    }

    @MediumTest
    public void testLoginButton_labelText() {
        startActivity(mLoginIntent, null, null);
        final Button LoginButton = (Button) getActivity().findViewById(R.id.button_connexion);

        final String expectedButtonText = getActivity().getString(R.string.action_log_in);
        assertEquals("Unexpected button label text", expectedButtonText,
                LoginButton.getText());
    }

    /**
     * Tests EditText get errors because fields are empty
     */
    @MediumTest
    public void testEditTextGetErrorsOK() {
        startActivity(mLoginIntent, null, null);
        final Button LoginButton = (Button) getActivity().findViewById(R.id.button_connexion);

        final EditText editTextLogin = (EditText) getActivity().findViewById(R.id.editText_login);
        final EditText editTextPassword = (EditText) getActivity().findViewById(R.id.editText_password);

        //Because this is an isolated ActivityUnitTestCase we have to directly click the
        //button from code
        LoginButton.performClick();

        String errors = editTextLogin.getError().toString();
        assertEquals("Unexpected error text", getActivity().getString(R.string.error_field_required), errors);

        errors = editTextPassword.getError().toString();
        assertEquals("Unexpected error text", getActivity().getString(R.string.error_field_required), errors);
    }

    /**
     * Tests EditText doesn't get errors because fields are not empty
     */
    @MediumTest
    public void testEditTextGetErrorsKO() {
        startActivity(mLoginIntent, null, null);
        final Button LoginButton = (Button) getActivity().findViewById(R.id.button_connexion);

        final EditText editTextLogin = (EditText) getActivity().findViewById(R.id.editText_login);
        final EditText editTextPassword = (EditText) getActivity().findViewById(R.id.editText_password);

        editTextLogin.setText("Test");
        editTextPassword.setText("Test");

        //Because this is an isolated ActivityUnitTestCase we have to directly click the
        //button from code
        LoginButton.performClick();

        assertNull("getError is not null", editTextLogin.getError());
        assertNull("getError is not null", editTextPassword.getError());
    }
}
