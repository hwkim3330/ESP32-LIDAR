package re.keti.lidarcloud

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.view.View

/**
 * Everything that is a number rather than a point: what is arriving, and what the IMU says.
 *
 * Drawn rather than laid out in widgets because it is a single instrument panel over a 3D scene,
 * and because the palette has to match the board's own page -- the same blue for series, the same
 * recessive grid, the same restraint about how many things are allowed to be bright.
 *
 * The graphs are the point of it. A number tells you the value now; a trace tells you whether the
 * sensor is steady, and steadiness is the thing you actually want to know about a rig on a bench.
 */
class Hud(context: Context) : View(context) {

    private val surface = Color.parseColor("#1a1a19")
    private val border = Color.parseColor("#33ffffff")
    private val ink = Color.WHITE
    private val ink2 = Color.parseColor("#c3c2b7")
    private val muted = Color.parseColor("#898781")
    private val grid = Color.parseColor("#2c2c2a")
    private val series = Color.parseColor("#3987e5")
    private val seriesB = Color.parseColor("#199e70")
    private val seriesC = Color.parseColor("#d95926")
    private val good = Color.parseColor("#0ca30c")

    private val fill = Paint(Paint.ANTI_ALIAS_FLAG)
    private val stroke = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE }
    private val text = Paint(Paint.ANTI_ALIAS_FLAG)

    var status = "starting…"
    var frames = 0
    var pointsReceived = 0
    var pointsDrawn = 0

    private val pointHistory = FloatArray(120)
    private val rateHistory = FloatArray(120)
    private val gapHistory = FloatArray(120)
    private val worstHistory = FloatArray(120)
    private var wireHead = 0
    var rate = 0
    var meanGap = 0
    var worstGap = 0
    var outliers = 0
    var linkMbit = 0
    private val gyroHistory = Array(3) { FloatArray(120) }
    private val accelHistory = Array(3) { FloatArray(120) }
    private var head = 0

    var accel = FloatArray(3)
    var gyro = FloatArray(3)

    // What the colours currently mean. Without it the ramp is decoration.
    var rangeLow = 0f
    var rangeHigh = 10f

    fun onFrame(received: Int, drawn: Int) {
        frames++
        pointsReceived = received
        pointsDrawn = drawn
        pointHistory[head % pointHistory.size] = received.toFloat()
        for (i in 0..2) {
            accelHistory[i][head % pointHistory.size] = accel.getOrElse(i) { 0f }
            gyroHistory[i][head % pointHistory.size] = gyro.getOrElse(i) { 0f }
        }
        head++
        invalidate()
    }

    /** The wire, as the board sees it -- one second per sample. */
    fun onWire(rate: Int, mean: Int, worst: Int, outliers: Int, link: Int) {
        this.rate = rate; this.meanGap = mean; this.worstGap = worst
        this.outliers = outliers; this.linkMbit = link
        rateHistory[wireHead % rateHistory.size] = rate.toFloat()
        gapHistory[wireHead % gapHistory.size] = mean.toFloat()
        worstHistory[wireHead % worstHistory.size] = worst.toFloat()
        wireHead++
        invalidate()
    }

    fun onImu(a: FloatArray, g: FloatArray) {
        accel = a
        gyro = g
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        val pad = dp(14f)
        val panelWidth = dp(300f)
        var y = pad

        // No title. The room is on the screen; a label saying which room it is adds nothing that
        // the picture does not already say, and the panel is worth more as one line of state.
        y = panel(canvas, pad, y, panelWidth, dp(40f)) { top ->
            fill.color = if (rate > 0) good else muted
            canvas.drawCircle(pad + dp(20f), top + dp(20f), dp(4f), fill)
            text.color = ink2; text.textSize = dp(12f); text.isFakeBoldText = false
            canvas.drawText(status, pad + dp(34f), top + dp(24f), text)
        }

        y = panel(canvas, pad, y, panelWidth, dp(86f)) { top ->
            tile(canvas, pad + dp(14f), top + dp(14f), "FRAME", frames.toString())
            tile(canvas, pad + dp(110f), top + dp(14f), "RECEIVED",
                "$pointsReceived/${CloudLink.POINTS}")
            tile(canvas, pad + dp(206f), top + dp(14f), "DRAWN", pointsDrawn.toString())
        }

        y = panel(canvas, pad, y, panelWidth, dp(86f)) { top ->
            tile(canvas, pad + dp(14f), top + dp(14f), "PACKETS/s", rate.toString())
            tile(canvas, pad + dp(110f), top + dp(14f), "MEAN GAP", "$meanGap µs")
            tile(canvas, pad + dp(206f), top + dp(14f), "LINK", "${linkMbit}M")
        }

        y = graph(canvas, pad, y, panelWidth, "Packets per second", rateHistory,
            listOf(good), 400f, "on UDP 7502", head = wireHead)

        y = graph(canvas, pad, y, panelWidth, "Gap between packets", gapHistory,
            listOf(series), 8000f, "mean, µs", head = wireHead)

        y = graph(canvas, pad, y, panelWidth, "Worst gap", worstHistory,
            listOf(seriesC), 20000f, "$outliers over 6250 µs", head = wireHead)

        y = legend(canvas, pad, y, panelWidth)

        y = graph(canvas, pad, y, panelWidth, "Points per frame", pointHistory,
            listOf(series), CloudLink.POINTS.toFloat(), "of ${CloudLink.POINTS}")

        y = graph(canvas, pad, y, panelWidth, "Acceleration  (g)", null,
            listOf(series, seriesB, seriesC), 2f, fmt3(accel), accelHistory)

        graph(canvas, pad, y, panelWidth, "Angular rate  (°/s)", null,
            listOf(series, seriesB, seriesC), 60f, fmt3(gyro), gyroHistory)
    }

    /**
     * The colour ramp, with the distances at its ends. The same five stops the shader mixes
     * between, sampled here rather than approximated, so the key cannot drift from the picture.
     */
    private fun legend(canvas: Canvas, x: Float, y: Float, w: Float): Float {
        val h = dp(58f)
        return panel(canvas, x, y, w, h) { top ->
            text.color = ink2; text.textSize = dp(11f); text.isFakeBoldText = true
            canvas.drawText("Distance", x + dp(14f), top + dp(18f), text)

            val barTop = top + dp(26f)
            val barHeight = dp(10f)
            val left = x + dp(14f)
            val right = x + w - dp(14f)
            val steps = 64
            for (i in 0 until steps) {
                val t = i / (steps - 1f)
                fill.color = rampColour(t)
                canvas.drawRect(left + (right - left) * i / steps, barTop,
                    left + (right - left) * (i + 1) / steps + 1f, barTop + barHeight, fill)
            }

            text.color = muted; text.isFakeBoldText = false; text.textSize = dp(10f)
            canvas.drawText(String.format("%.1f m", rangeLow), left, barTop + barHeight + dp(14f), text)
            val far = String.format("%.1f m", rangeHigh)
            canvas.drawText(far, right - text.measureText(far), barTop + barHeight + dp(14f), text)
        }
    }

    private fun rampColour(t: Float): Int {
        val stops = arrayOf(
            floatArrayOf(0.16f, 0.47f, 0.84f), floatArrayOf(0.10f, 0.62f, 0.72f),
            floatArrayOf(0.10f, 0.69f, 0.44f), floatArrayOf(0.92f, 0.63f, 0.10f),
            floatArrayOf(0.90f, 0.35f, 0.20f))
        val scaled = (t.coerceIn(0f, 1f) * (stops.size - 1))
        val i = scaled.toInt().coerceAtMost(stops.size - 2)
        val f = scaled - i
        fun mix(a: Float, b: Float) = a + (b - a) * f
        return Color.rgb(
            (mix(stops[i][0], stops[i + 1][0]) * 255).toInt(),
            (mix(stops[i][1], stops[i + 1][1]) * 255).toInt(),
            (mix(stops[i][2], stops[i + 1][2]) * 255).toInt())
    }

    private fun fmt3(v: FloatArray) =
        if (v.size < 3) "—" else String.format("%+.2f  %+.2f  %+.2f", v[0], v[1], v[2])

    private inline fun panel(
        canvas: Canvas, x: Float, y: Float, w: Float, h: Float, body: (Float) -> Unit
    ): Float {
        fill.color = surface
        canvas.drawRoundRect(RectF(x, y, x + w, y + h), dp(10f), dp(10f), fill)
        stroke.color = border; stroke.strokeWidth = dp(1f)
        canvas.drawRoundRect(RectF(x, y, x + w, y + h), dp(10f), dp(10f), stroke)
        body(y)
        return y + h + dp(10f)
    }

    private fun tile(canvas: Canvas, x: Float, y: Float, key: String, value: String) {
        text.color = muted; text.textSize = dp(9f); text.isFakeBoldText = false
        canvas.drawText(key, x, y + dp(10f), text)
        text.color = ink; text.textSize = dp(16f)
        canvas.drawText(value, x, y + dp(34f), text)
    }

    /**
     * One trace or three, over a fixed vertical range. Fixed, not auto-scaled: an axis that
     * rescales itself makes a still sensor look busy, which is the opposite of what this is for.
     */
    private fun graph(
        canvas: Canvas, x: Float, y: Float, w: Float, title: String,
        single: FloatArray?, colours: List<Int>, range: Float, note: String,
        triple: Array<FloatArray>? = null, head: Int = this.head
    ): Float {
        val h = dp(96f)
        return panel(canvas, x, y, w, h) { top ->
            text.color = ink2; text.textSize = dp(11f); text.isFakeBoldText = true
            canvas.drawText(title, x + dp(14f), top + dp(18f), text)
            text.color = muted; text.isFakeBoldText = false; text.textSize = dp(10f)
            canvas.drawText(note, x + w - dp(14f) - text.measureText(note), top + dp(18f), text)

            val plotTop = top + dp(28f)
            val plotBottom = top + h - dp(12f)
            val plotLeft = x + dp(14f)
            val plotRight = x + w - dp(14f)

            stroke.color = grid; stroke.strokeWidth = dp(1f)
            for (i in 0..2) {
                val gy = plotTop + (plotBottom - plotTop) * i / 2f
                canvas.drawLine(plotLeft, gy, plotRight, gy, stroke)
            }

            val sets = triple ?: arrayOf(single ?: return@panel)
            sets.forEachIndexed { index, values ->
                val path = Path()
                val n = values.size
                for (i in 0 until n) {
                    val v = values[(head + i) % n]
                    val px = plotLeft + (plotRight - plotLeft) * i / (n - 1f)
                    val t = if (triple != null) (v / range + 1f) / 2f else v / range
                    val py = plotBottom - (plotBottom - plotTop) * t.coerceIn(0f, 1f)
                    if (i == 0) path.moveTo(px, py) else path.lineTo(px, py)
                }
                stroke.color = colours[index % colours.size]
                stroke.strokeWidth = dp(1.6f)
                canvas.drawPath(path, stroke)
            }
        }
    }

    private fun dp(v: Float) = v * resources.displayMetrics.density
}
