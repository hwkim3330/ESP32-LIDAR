package re.keti.lidarcloud

import android.Manifest
import android.content.pm.PackageManager
import android.graphics.Color
import android.os.Build
import android.os.Bundle
import android.view.Gravity
import android.widget.FrameLayout
import android.widget.TextView
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
    private lateinit var status: TextView
    private lateinit var link: CloudLink

    private var frames = 0

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        cloud = CloudView(this)
        status = TextView(this).apply {
            setTextColor(Color.WHITE)
            textSize = 13f
            setPadding(28, 22, 28, 22)
            text = "starting…"
        }

        setContentView(FrameLayout(this).apply {
            addView(cloud)
            addView(status, FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.WRAP_CONTENT,
                FrameLayout.LayoutParams.WRAP_CONTENT, Gravity.TOP or Gravity.START))
        })

        link = CloudLink(this).apply {
            onStatus = { status.text = it }
            onGeometry = { altitudes, azimuths ->
                cloud.setGeometry(altitudes, azimuths)
                status.text = "geometry: ${altitudes.size} beams, ${altitudes.first()}° to ${altitudes.last()}°"
            }
            onFrame = { ranges, received ->
                frames++
                cloud.setFrame(ranges)
                // Received is out of 4096: what actually arrived, not what was sent. Notifications
                // are lossy by design here and saying so is more useful than hiding it.
                status.text = "frame $frames · $received/${CloudLink.POINTS} points"
            }
        }

        requestPermissionsThenStart()
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
            status.text = "Bluetooth permission is what this needs; nothing else."
        }
    }

    override fun onResume() { super.onResume(); cloud.onResume() }
    override fun onPause() { super.onPause(); cloud.onPause() }
}
