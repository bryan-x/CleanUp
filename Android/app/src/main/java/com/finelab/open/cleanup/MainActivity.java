package com.finelab.open.cleanup;

import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Field;
import java.nio.ByteBuffer;
import java.util.LinkedList;
import java.util.Set;
import java.util.concurrent.ExecutionException;

import com.finelab.open.cleanup.BluetoothSerialClient.BluetoothStreamingHandler;
import com.finelab.open.cleanup.BluetoothSerialClient.OnBluetoothEnabledListener;
import com.finelab.open.cleanup.BluetoothSerialClient.OnScanListener;

import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.bluetooth.BluetoothDevice;
import android.content.DialogInterface;
import android.content.DialogInterface.OnCancelListener;
import android.content.res.AssetManager;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.os.Bundle;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;

import android.support.v7.app.AppCompatActivity;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.ViewConfiguration;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemClickListener;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.EditText;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ImageView;
import android.util.Log;

public class MainActivity extends AppCompatActivity {

    private LinkedList<BluetoothDevice> mBluetoothDevices = new LinkedList<BluetoothDevice>();
    private ArrayAdapter<String> mDeviceArrayAdapter;

    private TextView mTextView;
    private TextView mTextBar1, mTextBar2;
    private Button mbtnAuto, mbtnFan1, mbtnFan2, mbtnFan3, mbtnCalibration, mbtnReset;
    private SeekBar mBar1, mBar2;
    private ProgressDialog mLoadingDialog;
    private AlertDialog mDeviceListDialog;
    private Menu mMenu;
    private BluetoothSerialClient mClient;
    private TextView mDustText1, mDustText2, mDustText3, mDustText4;
    private TextView mText1, mText2, mText3;
    private int run_led, run_pm, run_offset;
    private int nBar1, nBar2;
    private float run_sdc, run_calibration;
    private int run_day, run_tok;

    private static final int TX_COM = 1;
    private static final int TX_LED = 2;
    private static final int TX_CAL_OFFSET = 5;

    private static final int RX_LED = 1;
    private static final int RX_PM = 3;
    private static final int RX_SDC = 4;
    private static final int RX_CALIBRATION_DATA = 5;
    private static final int RX_CALIBRATION_OFFSET = 6;
    private static final int RX_ALIVE_DAY = 7;
    private static final int RX_ALIVE_TOK = 8;

    private static final int DUST_GRADE_GOOD = 15;
    private static final int DUST_GRADE_NORMAL = 35;
    private static final int DUST_GRADE_BAD = 75;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mClient = BluetoothSerialClient.getInstance();
        if(mClient == null) {
            Toast.makeText(getApplicationContext(), "Cannot use the Bluetooth device.", Toast.LENGTH_SHORT).show();
            finish();
        }

        overflowMenuInActionBar();
        initProgressDialog();
        initDeviceListDialog();
        initWidget();
    }

    private void overflowMenuInActionBar(){
        try {
            ViewConfiguration config = ViewConfiguration.get(this);
            Field menuKeyField = ViewConfiguration.class.getDeclaredField("sHasPermanentMenuKey");
            if(menuKeyField != null) {
                menuKeyField.setAccessible(true);
                menuKeyField.setBoolean(config, false);
            }
        } catch (Exception ex) {
            // 무시한다. 3.x 이 예외가 발생한다.
            // 또, 타블릿 전용으로 만들어진 3.x 버전의 디바이스는 보통 하드웨어 버튼이 존재하지 않는다.
        }
    }

    @Override
    protected void onPause() {
        mClient.cancelScan(getApplicationContext());
        super.onPause();
    }

    @Override
    protected void onResume() {
        super.onResume();
        enableBluetooth();
    }

    private void initProgressDialog() {
        mLoadingDialog = new ProgressDialog(this);
        mLoadingDialog.setCancelable(false);
    }

    private void initWidget() {
        mTextView = (TextView) findViewById(R.id.textViewTerminal);
        mTextView.setMovementMethod(new ScrollingMovementMethod());

        mDustText1 = (TextView)findViewById(R.id.staticDust1);
        mDustText2 = (TextView)findViewById(R.id.staticDust2);
        mDustText3 = (TextView)findViewById(R.id.staticDust3);
        mDustText4 = (TextView)findViewById(R.id.staticDust4);
        mText1 = (TextView)findViewById(R.id.editText1);        // 동작 카운트
        mText2 = (TextView)findViewById(R.id.editText2);        // 측정값
        mText3 = (TextView)findViewById(R.id.editText3);        // 센서 측정값

        mTextBar1 = (TextView) findViewById(R.id.textBar1);     // CO Bar
        mTextBar2 = (TextView) findViewById(R.id.textBar2);     // LED Bar

        mBar1= (SeekBar)findViewById(R.id.pBar1);
        mBar2 = (SeekBar)findViewById(R.id.pBar2);
        mBar1.setMax(20);
        mBar1.setProgress(0);
        nBar1 = 0;
        mBar2.setMax(10);
        mBar2.setProgress(5);
        nBar2 = 5;

        run_sdc = 0.0f;
        run_calibration = 0.0f;

        mbtnAuto = (Button) findViewById(R.id.btnAuto);
        mbtnFan1 = (Button) findViewById(R.id.btnFan1);
        mbtnFan2 = (Button) findViewById(R.id.btnFan2);
        mbtnFan3 = (Button) findViewById(R.id.btnFan3);
        mbtnCalibration = (Button) findViewById(R.id.btnCalibration);
        mbtnReset = (Button) findViewById(R.id.btnReset);

        mText1.setText( "0  / 0" );  // 동작 카운트
        mText2.setText( "0" );      // 실시간 측정
        mText3.setText( "0.00" );   // 센서 측정
        mTextBar1.setText("캘리브레이션 오차 " + mBar1.getProgress() );
        mTextBar2.setText("LED 밝기 조절 " + mBar2.getProgress() );

        mDustText1.setBackgroundColor( Color.GREEN );
        mDustText2.setBackgroundColor( Color.YELLOW );
        mDustText3.setBackgroundColor( Color.rgb( 255, 187, 0) );
        mDustText4.setBackgroundColor( Color.RED );

        mText2.setTextColor( Color.BLACK );         // 실시간 측정
        mText2.setBackgroundColor( Color.WHITE );

        mBar1.setOnSeekBarChangeListener( new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                nBar1 = progress;
                mTextBar1.setText("캘리브레이션 오차 " + progress );
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
            }
        });

        mBar2.setOnSeekBarChangeListener( new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                nBar2 = progress;
                mTextBar2.setText("LED 밝기 조절 " + progress );
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
            }
        });

        mbtnAuto.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                sendComAuto( (byte)nBar2 );
            }
        });

        mbtnFan1.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                sendComFan( (byte)1, (byte)nBar2 );
            }
        });

        mbtnFan2.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                sendComFan( (byte)2, (byte)nBar2 );
            }
        });

        mbtnFan3.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                sendComFan( (byte)3, (byte)nBar2 );
            }
        });

        mbtnCalibration.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                sendComCalibration( (byte)nBar1 );
            }
        });

        mbtnReset.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                sendComReset();
            }
        });
    }

    private void initDeviceListDialog() {
        mDeviceArrayAdapter = new ArrayAdapter<String>(getApplicationContext(), R.layout.item_device);
        ListView listView = new ListView(getApplicationContext());
        listView.setAdapter(mDeviceArrayAdapter);
        listView.setOnItemClickListener(new OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
                String item =  (String) parent.getItemAtPosition(position);
                for(BluetoothDevice device : mBluetoothDevices) {
                    if(item.contains(device.getAddress())) {
                        connect(device);
                        mDeviceListDialog.cancel();
                    }
                }
            }
        });
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle("Select bluetooth device");
        builder.setView(listView);
        builder.setPositiveButton("Scan",
                new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int id) {
                        scanDevices();
                    }
                });
        mDeviceListDialog = builder.create();
        mDeviceListDialog.setCanceledOnTouchOutside(false);
    }

    private void addDeviceToArrayAdapter(BluetoothDevice device) {
        if(mBluetoothDevices.contains(device)) {
            mBluetoothDevices.remove(device);
            mDeviceArrayAdapter.remove(device.getName() + "\n" + device.getAddress());
        }
        mBluetoothDevices.add(device);
        mDeviceArrayAdapter.add(device.getName() + "\n" + device.getAddress() );
        mDeviceArrayAdapter.notifyDataSetChanged();
    }

    private void enableBluetooth() {
        BluetoothSerialClient btSet =  mClient;
        btSet.enableBluetooth(this, new OnBluetoothEnabledListener() {
            @Override
            public void onBluetoothEnabled(boolean success) {
                if(success) {
                    getPairedDevices();
                } else {
                    finish();
                }
            }
        });
    }

    private void addText(String text) {
        mTextView.append(text);
        final int scrollAmount = mTextView.getLayout().getLineTop(mTextView.getLineCount()) - mTextView.getHeight();
        if (scrollAmount > 0)
            mTextView.scrollTo(0, scrollAmount);
        else
            mTextView.scrollTo(0, 0);
    }


    private void getPairedDevices() {
        Set<BluetoothDevice> devices =  mClient.getPairedDevices();
        for(BluetoothDevice device: devices) {
            addDeviceToArrayAdapter(device);
        }
    }

    private void scanDevices() {
        BluetoothSerialClient btSet = mClient;
        btSet.scanDevices(getApplicationContext(), new OnScanListener() {
            String message ="";
            @Override
            public void onStart() {
                Log.d("Test", "Scan Start.");
                mLoadingDialog.show();
                message = "Scanning....";
                mLoadingDialog.setMessage("Scanning....");
                mLoadingDialog.setCancelable(true);
                mLoadingDialog.setCanceledOnTouchOutside(false);
                mLoadingDialog.setOnCancelListener(new OnCancelListener() {
                    @Override
                    public void onCancel(DialogInterface dialog) {
                        BluetoothSerialClient btSet = mClient;
                        btSet.cancelScan(getApplicationContext());
                    }
                });
            }

            @Override
            public void onFoundDevice(BluetoothDevice bluetoothDevice) {
                addDeviceToArrayAdapter(bluetoothDevice);
                message += "\n" + bluetoothDevice.getName() + "\n" + bluetoothDevice.getAddress();
                mLoadingDialog.setMessage(message);
            }

            @Override
            public void onFinish() {
                Log.d("Test", "Scan finish.");
                message = "";
                mLoadingDialog.cancel();
                mLoadingDialog.setCancelable(false);
                mLoadingDialog.setOnCancelListener(null);
                mDeviceListDialog.show();
            }
        });
    }

    private void connect(BluetoothDevice device) {
        mLoadingDialog.setMessage("Connecting....");
        mLoadingDialog.setCancelable(false);
        mLoadingDialog.show();
        BluetoothSerialClient btSet =  mClient;
        btSet.connect(getApplicationContext(), device, mBTHandler);
    }

    private BluetoothStreamingHandler mBTHandler = new BluetoothStreamingHandler() {
        ByteBuffer mmByteBuffer = ByteBuffer.allocate(1024);

        @Override
        public void onError(Exception e) {
            mLoadingDialog.cancel();
            addText("Messgae : Connection error - " +  e.toString() + "\n");
            mMenu.getItem(0).setTitle(R.string.action_connect);
        }

        @Override
        public void onDisconnected() {
            mMenu.getItem(0).setTitle(R.string.action_connect);
            mLoadingDialog.cancel();
            addText("Messgae : Disconnected.\n");
        }
        @Override
        public void onData(byte[] buffer, int length) {
            if(length == 0) return;
            if(mmByteBuffer.position() + length >= mmByteBuffer.capacity()) {
                ByteBuffer newBuffer = ByteBuffer.allocate(mmByteBuffer.capacity() * 2);
                newBuffer.put(mmByteBuffer.array(), 0,  mmByteBuffer.position());
                mmByteBuffer = newBuffer;
            }
            mmByteBuffer.put(buffer, 0, length);
            if(buffer[length - 1] == '\n' ) {
                String msg = new String(mmByteBuffer.array(), 0, mmByteBuffer.position());
                addText(mClient.getConnectedDevice().getName() + " : " + msg + '\n');
                mmByteBuffer.clear();

                String [] token = msg.split( ",", -1 );
                run_led = Integer.parseInt(token[1]);
                run_pm = Integer.parseInt(token[3]);
                run_sdc = Float.parseFloat(token[4]);     // 4
                run_sdc = (float)(Math.round(run_sdc*100)/100.0); // 소수점 2자리 반올림.

                run_offset = Integer.parseInt(token[6]);
                run_day = Integer.parseInt(token[7]);
                run_tok = Integer.parseInt(token[8]);

                mText1.setText( "" + run_day + " / " + "" + run_tok );       // 동작 카운트
                mText2.setText( "" + run_pm );        // 실시간 측정
                if( run_pm <= DUST_GRADE_GOOD ) {
                    mText2.setTextColor( Color.BLACK );
                    mText2.setBackgroundColor( Color.GREEN );
                } else if ( run_pm > DUST_GRADE_GOOD && run_pm <= DUST_GRADE_NORMAL ) {
                    mText2.setTextColor( Color.BLACK );
                    mText2.setBackgroundColor( Color.YELLOW );
                } else if ( run_pm > DUST_GRADE_NORMAL && run_pm <= DUST_GRADE_BAD ) {
                    mText2.setTextColor( Color.BLACK );
                    mText2.setBackgroundColor( Color.rgb( 255, 187, 0) );
                } else {
                    mText2.setTextColor( Color.BLACK );
                    mText2.setBackgroundColor( Color.RED );
                }
                mText3.setText( "" + String.valueOf(run_sdc));   // 센서 측정
            }
        }

        @Override
        public void onConnected() {
            addText("Messgae : Connected. " + mClient.getConnectedDevice().getName() + "\n");
            mLoadingDialog.cancel();
            mMenu.getItem(0).setTitle(R.string.action_disconnect);
        }
    };

    public void sendComAuto( byte led ) {
        byte[] buffer = { (byte)0xFD, (byte)0x00, (byte)0x0A, (byte)0x00, (byte)0x00, (byte)0x00, (byte)0xFB };
        buffer[TX_LED] = led;
        if(mBTHandler.write(buffer)) {
            addText("Me : " + "Sensor Auto LED " + led + '\n');
        }
    }

    public void sendComFan( byte fan, byte led ) {
        byte[] buffer = { (byte)0xFD, (byte)0x01, (byte)0x0A, (byte)0x00, (byte)0x00, (byte)0x00, (byte)0xFB };
        buffer[TX_COM] = fan;
        buffer[TX_LED] = led;
        if(mBTHandler.write(buffer)) {
            addText("Me : " + "Sensor Fan " + fan + "LED " + led + '\n');
        }
    }

    public void sendComCalibration( byte offset ) {
        byte[] buffer = { (byte)0xFD, (byte)0x04, (byte)0x0A, (byte)0x01, (byte)0x00, (byte)0x00, (byte)0xFB };
        buffer[TX_CAL_OFFSET] = offset;
        if(mBTHandler.write(buffer)) {
            addText("Me : " + "Calibration Offset " + offset + '\n');
        }
    }

    public void sendComReset() {
        byte[] buffer = { (byte)0xFD, (byte)0x05, (byte)0x0A, (byte)0x00, (byte)0x00, (byte)0x00, (byte)0xFB };
        buffer[TX_COM] = 0x5;
        if(mBTHandler.write(buffer)) {
            addText("Me : " + "Reset" + '\n');
        }
    }

    protected void onDestroy() {
        super.onDestroy();
        mClient.claer();
    };


    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.main, menu);
        mMenu = menu;
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        boolean connect = mClient.isConnection();
        if(item.getItemId() == R.id.action_connect) {
            if (!connect) {
                mDeviceListDialog.show();
            } else {
                mBTHandler.close();
            }
            return true;
        } else {
            showAboutDlg();
            return true;
        }
    }

    private void showAboutDlg() {
        AssetManager am = getResources().getAssets();
        ImageView imageView = new ImageView( this );
        imageView.setImageResource(R.drawable.logo);

        imageView.setScaleType( ImageView.ScaleType.FIT_CENTER );
        AlertDialog.Builder aboutdialog = new AlertDialog.Builder( this );
        aboutdialog.setView(imageView);
        aboutdialog.setMessage("\r\n" +
                "HW 제작 SED (SeMin DIY) \r\n" +
                "펌웨어 v3.0 SED (SeMin DIY) Release \r\n" +
                "펌웨어 v3.1 bryan 2018.06.18 Update \r\n" +
                "펌웨어 v3.2 bryan 2018.06.21 Update \r\n" +
                "안드로이드 시험 앱 v3.2 Release\r\n\r\n");
        aboutdialog.setPositiveButton( "OK", new AlertDialog.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.cancel();
            }
        });

        AlertDialog about = aboutdialog.create();
        about.setTitle( "CleanUp 대하여 ..." );
        about.show();

    	  /*
        TextView codeView = new TextView(this);
        codeView.setText(Html.fromHtml(readCode()));
        codeView.setMovementMethod(new ScrollingMovementMethod());
        codeView.setBackgroundColor(Color.parseColor("#202020"));
        new AlertDialog.Builder(this, android.R.style.Theme_Holo_Light_DialogWhenLarge)
                .setView(codeView)
                .setPositiveButton("OK", new AlertDialog.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        dialog.cancel();
                    }
                }).show();
        */
    }
}
