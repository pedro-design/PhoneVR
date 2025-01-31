package viritualisres.phonevr

import android.app.Activity
import android.content.Context
import android.content.SharedPreferences
import android.content.pm.ActivityInfo
import android.graphics.Point
import android.graphics.SurfaceTexture
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.opengl.GLSurfaceView
import android.os.Build
import android.os.Bundle
import android.util.Log
import android.view.MotionEvent
import android.view.Surface
import android.view.View
import android.view.WindowInsets
import android.widget.FrameLayout
import android.widget.TextView
import com.google.vr.ndk.base.AndroidCompat
import com.google.vr.ndk.base.GvrLayout


import android.content.Intent
import android.os.BatteryManager
import android.os.Build.VERSION
import android.os.Build.VERSION_CODES
import android.content.ContextWrapper
import android.content.IntentFilter

import java.util.concurrent.locks.ReentrantLock
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10



    class GameActivity : Activity(), SensorEventListener {

    private lateinit var sensMgr: SensorManager
    private lateinit var prefs: SharedPreferences
    private lateinit var surf: GLSurfaceView
    private lateinit var gvrLayout: GvrLayout


    private var uiFPSTextViewUpdatethread: Thread? = null
    private var fpsMutex: ReentrantLock = ReentrantLock()
    private var fpsResumeMutex: ReentrantLock = ReentrantLock()


    private fun getBatteryLevel(): Int {
            val batteryLevel: Int
            if (VERSION.SDK_INT >= VERSION_CODES.LOLLIPOP) {
                val batteryManager = getSystemService(Context.BATTERY_SERVICE) as BatteryManager
                batteryLevel = batteryManager.getIntProperty(BatteryManager.BATTERY_PROPERTY_CAPACITY)
            } else {
                val intent = ContextWrapper(applicationContext).
                registerReceiver(null, IntentFilter(Intent.ACTION_BATTERY_CHANGED))
                batteryLevel = intent!!.getIntExtra(BatteryManager.EXTRA_LEVEL,
                    -1) * 100 / intent.getIntExtra(BatteryManager.EXTRA_SCALE, -1)
            }

            return batteryLevel
    }

    // CPU TimeDelays
    //private var tDelaycRend: Float = 0.0f
   // private var tDelaycEnc: Float = 0.0f
    var lastctDelaysVals : Array<Array<Float>> = arrayOf(arrayOf<Float>(0.0f, 0.0f, 0.0f, 0.0f, 0.0f), //tDelay CPU Rendering
                                                    arrayOf<Float>(0.0f, 0.0f, 0.0f, 0.0f, 0.0f)) //tDelay CPU Encoding

    var lasttDelaysVals : Array<Array<Int>> = arrayOf(  arrayOf<Int>(0, 0, 0, 0, 0), // tNetworkDelay
                                                        arrayOf<Int>(0, 0, 0, 0, 0)) // tDelaycTillUpdateCall

    // Mobile FPSs
    //var fpsStreamRecv: Float = 0.0f
    //var fpsDecoder: Float = 0.0f
    //var fpsRenderer: Float = 0.0f
    var lastfpsVals : Array<Array<Float>> = arrayOf(arrayOf<Float>(0.0f, 0.0f, 0.0f, 0.0f, 0.0f), //StreamRecv
                                                    arrayOf<Float>(0.0f, 0.0f, 0.0f, 0.0f, 0.0f), //Decoder
                                                    arrayOf<Float>(0.0f, 0.0f, 0.0f, 0.0f, 0.0f)) //Renderer

    // CPU FPSs
    //var cfpsSteamVRApp: Float = 0.0f
    //var cfpsEncoder: Float = 0.0f
    //var cfpsStreamWriter: Float = 0.0f
    //var cfpsStreamer: Float = 0.0f
    var clastfpsVals: Array<Array<Float>> = arrayOf(arrayOf<Float>(0.0f, 0.0f, 0.0f, 0.0f, 0.0f), //VRApp
                                                    arrayOf<Float>(0.0f, 0.0f, 0.0f, 0.0f, 0.0f), //Encoder
                                                    arrayOf<Float>(0.0f, 0.0f, 0.0f, 0.0f, 0.0f), //StreamWriter
                                                    arrayOf<Float>(0.0f, 0.0f, 0.0f, 0.0f, 0.0f), //StreamWriter
                                                    arrayOf<Float>(0.0f, 0.0f, 0.0f, 0.0f, 0.0f)) //Renderer

    var fpsCounter : Int = 0 // For storing previous 5 Fps Values

    private var isDaydream = false

    override fun onCreate(savedInstanceState: Bundle?) {

        super.onCreate(savedInstanceState)

        sensMgr = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        prefs = getSharedPreferences(pvrPrefsKey, Context.MODE_PRIVATE)
        surf = GLSurfaceView(this)
        gvrLayout = GvrLayout(this)

        val mainActRot = intent.getIntExtra("MAINLAYOUT_ROT",Surface.ROTATION_0)
        if( mainActRot == Surface.ROTATION_270) {
            requestedOrientation = ActivityInfo.SCREEN_ORIENTATION_REVERSE_LANDSCAPE
        }

        Wrap.setGameView(this)


        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            window.decorView.setOnApplyWindowInsetsListener { _, insets ->
                if (insets.isVisible(0))
                    setImmersiveSticky()
                insets
            }
        }
        else {
            window.decorView.setOnSystemUiVisibilityChangeListener { visibility ->
                if (visibility and View.SYSTEM_UI_FLAG_FULLSCREEN == 0)
                    setImmersiveSticky()
            }
        }

        Wrap.createRenderer(gvrLayout.gvrApi.nativeGvrContext)

        with(surf) {
            setEGLContextClientVersion(3)
            setEGLConfigChooser(8, 8, 8, 0, 0, 0)
            preserveEGLContextOnPause = true
            setRenderer(Renderer())
            setOnTouchListener { v, event ->
                if (event.action == MotionEvent.ACTION_DOWN) {
                    //((Vibrator) getSystemService(Context.VIBRATOR_SERVICE)).vibrate(50);
                    Wrap.onTriggerEvent()
                    v.performClick()
                    return@setOnTouchListener true
                }
                false
            }
        }
        gvrLayout.setPresentationView(surf)
        gvrLayout.uiLayout.setCloseButtonListener { onBackPressed() }
        //setContentView(gvrLayout)

        setContentView(R.layout.activity_game)
        val gameActivityLayout =  findViewById<FrameLayout>(R.id.gvrRootLayout)
        gameActivityLayout.addView(gvrLayout)

        if (gvrLayout.setAsyncReprojectionEnabled(true))
            AndroidCompat.setSustainedPerformanceMode(this, true)


        isDaydream = AndroidCompat.setVrModeEnabled(this, true)

        //getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        Wrap.startSendSensorData(prefs.getInt(posePortKey, posePortDef))

        //Log.d("--PVR-Java--", "main Layout Orientation : (" + mainActRot +")" /*+ mainLayout.rotation.toString() + ", GVR: "*/ + windowManager.defaultDisplay.rotation);

        setImmersiveSticky()

        if(prefs.getBoolean(debugKey, debugDef)) {
            uiFPSTextViewUpdatethread = object : Thread() {
                override fun run() {
                    try {
                        while (!this.isInterrupted) {
                            sleep(250)
                            runOnUiThread {
                                val tv = findViewById<TextView>(R.id.textViewFPS)
                                fpsResumeMutex.lock()
                                fpsMutex.lock()
                                /*  --- FPS and Latency Stats       ---
                                    --- M - Mobile; C - CPU/Desktop ---
                                    SR - StreamReceiver @M
                                    D - Media Decoder @M
                                    R - Frame Renderer @M

                                    cR - Frame Renderer @C
                                    SS - Stream Sender @C
                                    SW - Stream Writer @C
                                    E - Media Encoder @C
                                    VRa - VR Application FPS @C

                                    tSS - Time Delay between RendererGotFrame and RendererRendered @C
                                    tE - Time Delay between EncoderGotFrame and EncoderEncoded @C
                                    tND - NetworkDelay - Time Delay between DataPacketSent from @C to DataPacketReceived @M
                                    tSR - StreamReceiver - Time Delay between DataPacketSent from @C to DataSentToMediaDecoder @M
                                 */
                                tv.text = String.format("---FPS---\n" +
                                        "M| SR: %5.1f D: %5.1f R: %5.1f cR: %5.1f\n" +
                                        "C| SS: %5.1f SW: %5.1f E: %5.1f VRa: %5.1f\n" +
                                        "---Latency(ms)---\n" +
                                        "C| tSS: %5.1f tE: %5.1f\n" +
                                        "M| tND: %5.1f tSR: %5.1f , bat %5.1f",
                                        lastfpsVals[0].average(), lastfpsVals[1].average(), lastfpsVals[2].average(), clastfpsVals[4].average(),
                                        clastfpsVals[3].average(), clastfpsVals[2].average(), clastfpsVals[1].average(), clastfpsVals[0].average(),
                                        lastctDelaysVals[1].average(), lastctDelaysVals[0].average(),
                                        lasttDelaysVals[0].average(), lasttDelaysVals[1].average(),(getBatteryLevel()/100.0f ))
                                        //fpsStreamRecv, fpsDecoder, fpsRenderer,
                                        //cfpsStreamer, cfpsEncoder, cfpsSteamVRApp)
                                fpsMutex.unlock()
                                fpsResumeMutex.unlock()
                            }
                        }
                    } catch (e: InterruptedException) {
                        e.message?.let { Log.d("PVR-Java", it) }
                    }
                }
            }
            uiFPSTextViewUpdatethread?.start()
        }
    }

    override fun onPause() {
        Wrap.onPause()
        surf.onPause()
        gvrLayout.onPause()
        sensMgr.unregisterListener(this)
        if(uiFPSTextViewUpdatethread != null)
            fpsResumeMutex.lock()
        super.onPause()
    }

    override fun onResume() {
        super.onResume()
        val acc = sensMgr.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION) //Sensor.TYPE_ROTATION_VECTOR for orientation
     //   val giro = sensMgr.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
      //  val acelerometer = sensMgr.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        // use this 2 sensors to perform sensor fusion in pc
       // sensMgr.registerListener(this, acelerometer, SensorManager.SENSOR_DELAY_FASTEST, 1000)
        //sensMgr.registerListener(this, giro, SensorManager.SENSOR_DELAY_FASTEST, 1000)
        //
        sensMgr.registerListener(this, acc, SensorManager.SENSOR_DELAY_FASTEST, 1000) //max 5ms latency
        gvrLayout.onResume()
        surf.onResume()
        Wrap.onResume()

        // FPS Update Thread exits ? Yes ? It must be Paused by onPause(). Resume it
        if( (uiFPSTextViewUpdatethread != null) && fpsResumeMutex.isLocked )
            fpsResumeMutex.unlock()

        //Log.d("--PVR-Java--", "Resume: main Layout Orientation : " /*+ mainLayout.rotation.toString() + ", GVR: "*/ + windowManager.defaultDisplay.rotation);
    }

    override fun onDestroy() {
        Wrap.stopAll()
        gvrLayout.shutdown()
        super.onDestroy()
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus)
            setImmersiveSticky()
    }

    private fun setImmersiveSticky() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            window.setDecorFitsSystemWindows(false)
            window.insetsController?.hide(WindowInsets.Type.ime())
        }
        else {
            window.decorView.systemUiVisibility =
                    (View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                    or View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                    or View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                    or View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                    or View.SYSTEM_UI_FLAG_FULLSCREEN
                    or View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY)
        }
    }

    //SensorEventListener methods
    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}


    override fun onSensorChanged(event: SensorEvent) {
        val type = event.sensor.type
        val batery_arr = (getBatteryLevel()/100.0f )
        if (type == Sensor.TYPE_LINEAR_ACCELERATION) {

            Wrap.setAccData(event.values)
            Wrap.setBatData(batery_arr)
        }
    }

    fun updateFPS(fpsStreamRecvJNI: Float, fpsDecoderJNI: Float, fpsRendererJNI: Float,
                  cfpsSteamVRAppJNI : Float, cfpsEncoderJNI : Float, cfpsStreamWriterJNI: Float, cfpsStreamerJNI: Float, cfpsRenderer: Float,
                  ctDelayRend : Float, ctDelayEnc: Float,
                  tNetworkDelay:Int, tStreamRecvDelayms : Int) {
        fpsMutex.lock()

        //fpsStreamRecv = fpsStreamRecvJNI
        //fpsDecoder = fpsDecoderJNI
        //fpsRenderer = fpsRendererJNI
        lastfpsVals[0][fpsCounter] = fpsStreamRecvJNI
        lastfpsVals[1][fpsCounter] = fpsDecoderJNI
        lastfpsVals[2][fpsCounter] = fpsRendererJNI

        //cfpsSteamVRApp = cfpsSteamVRAppJNI
        //cfpsEncoder = cfpsEncoderJNI
        //cfpsStreamWriter = cfpsStreamWriterJNI
        //cfpsStreamer = cfpsStreamerJNI

        clastfpsVals[0][fpsCounter] = cfpsSteamVRAppJNI
        clastfpsVals[1][fpsCounter] = cfpsEncoderJNI
        clastfpsVals[2][fpsCounter] = cfpsStreamWriterJNI
        clastfpsVals[3][fpsCounter] = cfpsStreamerJNI
        clastfpsVals[4][fpsCounter] = cfpsRenderer

        lastctDelaysVals[0][fpsCounter] = ctDelayRend
        lastctDelaysVals[1][fpsCounter] = ctDelayEnc

        lasttDelaysVals[0][fpsCounter] = tNetworkDelay// Latency in Network Communication
        lasttDelaysVals[1][fpsCounter] = tStreamRecvDelayms

        fpsCounter = (++fpsCounter) % 5
        fpsMutex.unlock()
    }

    private inner class Renderer : GLSurfaceView.Renderer/*, SurfaceTexture.OnFrameAvailableListener*/ {
        private var surfTex: SurfaceTexture? = null

        override fun onSurfaceCreated(gl: GL10, c: EGLConfig) {
            val sz = Point()
            windowManager.defaultDisplay.getRealSize(sz)
            val texID = Wrap.initSystem(sz.x, sz.y,
                    prefs.getInt(resMulKey, resMulDef),
                    prefs.getFloat(offFovKey, offFovDef),
                    prefs.getBoolean(warpKey, warpDef),
                    prefs.getBoolean(debugKey, debugDef))

            surfTex = SurfaceTexture(texID, false) // true <- single buffer mode

            val s = Surface(surfTex)
            Wrap.startMediaCodec(s)
            s.release()

            Thread {
                Wrap.setVStreamPort(prefs!!.getInt(videoPortKey, videoPortDef))
                Wrap.startStream()
            }.start()
        }

        override fun onSurfaceChanged(gl: GL10, width: Int, height: Int) {
            //Log.d("PhoneVR", "onSurfaceChanged" + width.toString() + "x" + height.toString());
        }

        override fun onDrawFrame(gl: GL10) {
            val pts = Wrap.vFrameAvailable()
            if (pts != -1L)
                surfTex!!.updateTexImage()

            Wrap.drawFrame(pts)
        }
    }
}
