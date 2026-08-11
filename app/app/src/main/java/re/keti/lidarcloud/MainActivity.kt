package re.keti.lidarcloud

import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.view.View
import android.view.WindowManager
import android.widget.FrameLayout
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat

/**
 * The room as the sensor sees it, on the tablet, over BLE.
 *
 * The tablet stays on whatever WiFi it is on -- this asks nothing of the network it is joined to,
 * which is the whole reason the link is BLE and not the board's own access point.
 */
class MainActivity : AppCompatActivity() {

    private lateinit var cloud: CloudView
    private lateinit var hud: Hud
    private lateinit var link: CloudLink

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        cloud = CloudView(this)
        hud = Hud(this)
        // The panel reports; the scene is what you touch. Without this the HUD would eat every
        // drag that started over it, which is the whole left edge of the screen.
        hud.isClickable = false
        hud.isFocusable = false

        setContentView(FrameLayout(this).apply {
            addView(cloud)
            addView(hud, FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT))
        })

        cloud.onFrameDrawn = { drawn -> runOnUiThread { hud.pointsDrawn = drawn } }

        link = CloudLink(this).apply {
            onStatus = { hud.status = it; hud.invalidate() }
            onGeometry = { altitudes, azimuths ->
                cloud.setGeometry(altitudes, azimuths)
                hud.status = "${altitudes.size} beams, ${altitudes.first()}° to ${altitudes.last()}°"
                hud.invalidate()
            }
            onFrame = { ranges, received ->
                cloud.setFrame(ranges)
                hud.onFrame(received, hud.pointsDrawn)
            }
            onImu = { accel, gyro -> hud.onImu(accel, gyro) }
            onWire = { rate, mean, worst, outliers, link ->
                hud.onWire(rate, mean, worst, outliers, link)
            }
        }

        requestPermissionsThenStart()
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) {
            window.decorView.systemUiVisibility =
                View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY or
                        View.SYSTEM_UI_FLAG_HIDE_NAVIGATION or
                        View.SYSTEM_UI_FLAG_FULLSCREEN or
                        View.SYSTEM_UI_FLAG_LAYOUT_STABLE or
                        View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION or
                        View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
        }
    }

    private fun requestPermissionsThenStart() {
        val needed = mutableListOf<String>()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            needed += Manifest.permission.BLUETOOTH_SCAN
            needed += Manifest.permission.BLUETOOTH_CONNECT
        } else {
            needed += Manifest.permission.ACCESS_FINE_LOCATION
        }
        val missing = needed.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }
        if (missing.isEmpty()) link.start()
        else ActivityCompat.requestPermissions(this, missing.toTypedArray(), 1)
    }

    override fun onRequestPermissionsResult(
        requestCode: Int, permissions: Array<out String>, grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (grantResults.isNotEmpty() && grantResults.all { it == PackageManager.PERMISSION_GRANTED }) {
            link.start()
        } else {
            hud.status = "Bluetooth permission is what this needs; nothing else."
            hud.invalidate()
        }
    }

    override fun onResume() { super.onResume(); cloud.onResume() }
    override fun onPause() { super.onPause(); cloud.onPause() }
}
