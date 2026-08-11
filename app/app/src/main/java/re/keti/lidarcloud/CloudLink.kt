package re.keti.lidarcloud

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.os.Handler
import android.os.Looper
import org.json.JSONObject
import java.util.UUID

/**
 * The link to the board: scan, connect, read the beam geometry once, then take frames.
 *
 * Frames arrive as chunks of a fixed shape and notifications are unacknowledged, so some chunks
 * are lost. That is deliberate on both ends -- a frame with a hole in it is still a picture of
 * the room, while a stream that stalls waiting for a retransmit is not. A frame is handed on when
 * the sequence number changes, whatever arrived by then.
 */
class CloudLink(private val context: Context) {

    companion object {
        val SERVICE: UUID = UUID.fromString("6b1e0001-4b2a-4f6d-9c3a-0f1e2d3c4b5a")
        val FRAME: UUID = UUID.fromString("6b1e0002-4b2a-4f6d-9c3a-0f1e2d3c4b5a")
        val GEOMETRY: UUID = UUID.fromString("6b1e0003-4b2a-4f6d-9c3a-0f1e2d3c4b5a")
        val CCCD: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

        const val BEAMS = 16
        const val COLUMNS = 256          // the board sends every other column
        const val POINTS = BEAMS * COLUMNS
    }

    var onStatus: (String) -> Unit = {}
    var onGeometry: (FloatArray, FloatArray) -> Unit = { _, _ -> }
    var onFrame: (ShortArray, Int) -> Unit = { _, _ -> }

    private val main = Handler(Looper.getMainLooper())
    private var gatt: BluetoothGatt? = null

    private var frame = ShortArray(POINTS)
    private var frameSequence = -1
    private var received = 0

    private fun say(message: String) = main.post { onStatus(message) }

    @SuppressLint("MissingPermission")
    fun start() {
        val adapter = BluetoothAdapter.getDefaultAdapter()
        if (adapter == null || !adapter.isEnabled) {
            say("Bluetooth is off")
            return
        }
        say("scanning for the board…")
        val scanner = adapter.bluetoothLeScanner
        val filter = ScanFilter.Builder().setServiceUuid(android.os.ParcelUuid(SERVICE)).build()
        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build()
        scanner.startScan(listOf(filter), settings, object : ScanCallback() {
            override fun onScanResult(callbackType: Int, result: ScanResult) {
                scanner.stopScan(this)
                say("found ${result.device.address}, connecting…")
                connect(result.device)
            }

            override fun onScanFailed(errorCode: Int) { say("scan failed ($errorCode)") }
        })
    }

    @SuppressLint("MissingPermission")
    private fun connect(device: BluetoothDevice) {
        gatt = device.connectGatt(context, false, object : BluetoothGattCallback() {
            override fun onConnectionStateChange(g: BluetoothGatt, status: Int, newState: Int) {
                if (newState == BluetoothProfile.STATE_CONNECTED) {
                    say("connected, looking for the service…")
                    g.requestMtu(517)
                } else {
                    say("disconnected — rescanning")
                    main.postDelayed({ start() }, 1500)
                }
            }

            override fun onMtuChanged(g: BluetoothGatt, mtu: Int, status: Int) {
                g.discoverServices()
            }

            override fun onServicesDiscovered(g: BluetoothGatt, status: Int) {
                val service = g.getService(SERVICE) ?: run { say("service missing"); return }
                // Geometry first: a point cannot be placed without it, so there is no reason to
                // start taking frames before it has arrived.
                g.readCharacteristic(service.getCharacteristic(GEOMETRY))
            }

            override fun onCharacteristicRead(
                g: BluetoothGatt, c: BluetoothGattCharacteristic, status: Int
            ) {
                @Suppress("DEPRECATION")
                if (c.uuid == GEOMETRY) {
                    @Suppress("DEPRECATION")
                    parseGeometry(String(c.value ?: ByteArray(0)))
                    subscribe(g)
                }
            }

            @Suppress("DEPRECATION")
            override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic) {
                if (c.uuid == FRAME) acceptChunk(c.value ?: return)
            }
        }, BluetoothDevice.TRANSPORT_LE)
    }

    @SuppressLint("MissingPermission")
    private fun subscribe(g: BluetoothGatt) {
        val characteristic = g.getService(SERVICE).getCharacteristic(FRAME)
        g.setCharacteristicNotification(characteristic, true)
        val cccd = characteristic.getDescriptor(CCCD)
        @Suppress("DEPRECATION")
        cccd.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
        @Suppress("DEPRECATION")
        g.writeDescriptor(cccd)
        say("taking frames")
    }

    private fun parseGeometry(json: String) {
        try {
            val root = JSONObject(json)
            val altitudes = root.getJSONArray("beam_altitude_angles")
            val azimuths = root.getJSONArray("beam_azimuth_angles")
            val a = FloatArray(altitudes.length()) { altitudes.getDouble(it).toFloat() }
            val z = FloatArray(azimuths.length()) { azimuths.getDouble(it).toFloat() }
            main.post { onGeometry(a, z) }
        } catch (e: Exception) {
            say("geometry did not parse: ${e.message}")
        }
    }

    private fun acceptChunk(data: ByteArray) {
        if (data.size < 10) return
        val sequence = (data[0].toInt() and 0xFF) or ((data[1].toInt() and 0xFF) shl 8) or
                ((data[2].toInt() and 0xFF) shl 16) or ((data[3].toInt() and 0xFF) shl 24)
        val firstColumn = (data[8].toInt() and 0xFF) or ((data[9].toInt() and 0xFF) shl 8)

        if (sequence != frameSequence) {
            if (frameSequence >= 0) {
                val done = frame
                val count = received
                main.post { onFrame(done, count) }
            }
            frame = ShortArray(POINTS)
            received = 0
            frameSequence = sequence
        }

        var point = firstColumn * BEAMS
        var i = 10
        while (i + 1 < data.size && point < POINTS) {
            frame[point] = (((data[i].toInt() and 0xFF)) or
                    ((data[i + 1].toInt() and 0xFF) shl 8)).toShort()
            point++
            received++
            i += 2
        }
    }
}
