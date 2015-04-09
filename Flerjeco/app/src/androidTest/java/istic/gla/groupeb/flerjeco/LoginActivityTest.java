package istic.gla.groupeb.flerjeco;


import android.content.Intent;
import android.test.ActivityUnitTestCase;
import android.widget.Button;

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

        mLoginIntent = new Intent(getInstrumentation()
                .getTargetContext(), LoginActivity.class);
        startActivity(mLoginIntent, null, null);
        final Button launchNextButton =
                (Button) getActivity()
                        .findViewById(R.id.button_connexion);
    }

}
