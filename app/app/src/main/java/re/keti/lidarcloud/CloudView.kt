package re.keti.lidarcloud

import android.content.Context
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import android.view.MotionEvent
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

/**
 * The room, as points.
 *
 * Ranges become positions here rather than on the board, because the board would then be sending
 * three floats where it now sends one short -- six times the bytes over a link that is the scarce
 * thing. The angles it takes to do the conversion are the sensor's own calibration, read once.
 *
 * Three things make a point cloud read as three-dimensional rather than as a flat scatter, and
 * all three are here: a ground grid to sit on, axes at the sensor to say where the origin is, and
 * point size that falls off with distance so near and far separate.
 *
 * Colour is distance from the sensor. Height was tried first and came out almost monochrome, for
 * a reason worth keeping: most of what a floor-standing OS1-16 returns is floor, so most points
 * sit within a few centimetres of each other in z while a handful of wall and ceiling returns
 * stretch the range. Distance has no such pile-up -- every concentric ring the beams cut into the
 * floor lands on its own part of the ramp, which is also exactly the structure a person looking
 * at the room wants to see.
 */
class CloudView(context: Context) : GLSurfaceView(context) {

    private val renderer = CloudRenderer()

    init {
        setEGLContextClientVersion(2)
        setRenderer(renderer)
        renderMode = RENDERMODE_CONTINUOUSLY
    }

    fun setGeometry(altitudes: FloatArray, azimuths: FloatArray) =
        renderer.setGeometry(altitudes, azimuths)

    fun setFrame(ranges: ShortArray) = renderer.setFrame(ranges)

    var onFrameDrawn: (Int) -> Unit
        get() = renderer.onFrameDrawn
        set(value) { renderer.onFrameDrawn = value }

    var onRange: (Float, Float) -> Unit
        get() = renderer.onRange
        set(value) { renderer.onRange = value }

    private var lastX = 0f
    private var lastY = 0f
    private var lastSpan = 0f

    private var lastDown = 0L

    override fun onTouchEvent(event: MotionEvent): Boolean {
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                lastX = event.x; lastY = event.y
                // Two taps in quick succession put the camera back. Orbiting a cloud is easy to
                // get lost in and there is no other way out of an upside-down view.
                val now = System.currentTimeMillis()
                if (now - lastDown < 300) renderer.resetView()
                lastDown = now
            }
            MotionEvent.ACTION_POINTER_DOWN -> lastSpan = span(event)
            MotionEvent.ACTION_MOVE -> {
                if (event.pointerCount >= 2) {
                    val s = span(event)
                    if (lastSpan > 0f) renderer.zoom(s / lastSpan)
                    lastSpan = s
                } else {
                    renderer.orbit((event.x - lastX) * 0.4f, (event.y - lastY) * 0.4f)
                    lastX = event.x; lastY = event.y
                }
            }
        }
        return true
    }

    private fun span(e: MotionEvent): Float {
        val dx = e.getX(0) - e.getX(1)
        val dy = e.getY(0) - e.getY(1)
        return kotlin.math.sqrt(dx * dx + dy * dy)
    }
}

class CloudRenderer : GLSurfaceView.Renderer {

    private var program = 0
    private var positionHandle = 0
    private var matrixHandle = 0
    private var pointSizeHandle = 0

    private val projection = FloatArray(16)
    private val view = FloatArray(16)
    private val mvp = FloatArray(16)

    private var yaw = 30f
    private var pitch = 22f
    private var distance = 14f

    var onFrameDrawn: (Int) -> Unit = {}

    /** The distance range the colour ramp currently spans, so the legend can say what it means. */
    var onRange: (Float, Float) -> Unit = { _, _ -> }

    // Distance range of the current frame, smoothed so the colours do not jump between frames.
    private var lowZ = 0.5f
    private var highZ = 10f

    private var gridBuffer: FloatBuffer = ByteBuffer.allocateDirect(4)
        .order(ByteOrder.nativeOrder()).asFloatBuffer()
    private var gridVertices = 0
    private var axisBuffer: FloatBuffer = ByteBuffer.allocateDirect(4)
        .order(ByteOrder.nativeOrder()).asFloatBuffer()
    private var ringBuffer: FloatBuffer = ByteBuffer.allocateDirect(4)
        .order(ByteOrder.nativeOrder()).asFloatBuffer()
    private var ringVertices = 0

    // Three floats per point, rebuilt whenever a frame lands.
    private var vertices: FloatBuffer =
        ByteBuffer.allocateDirect(CloudLink.POINTS * 3 * 4)
            .order(ByteOrder.nativeOrder()).asFloatBuffer()
    private var vertexCount = 0

    private var altitudes = FloatArray(CloudLink.BEAMS) { 0f }
    private var azimuths = FloatArray(CloudLink.BEAMS) { 0f }
    private var pendingRanges: ShortArray? = null
    private val heightSample = FloatArray(CloudLink.POINTS / 4 + 1)

    fun setGeometry(a: FloatArray, z: FloatArray) {
        if (a.size >= CloudLink.BEAMS) altitudes = a
        if (z.size >= CloudLink.BEAMS) azimuths = z
    }

    fun setFrame(ranges: ShortArray) { pendingRanges = ranges }

    fun orbit(dx: Float, dy: Float) {
        yaw += dx
        pitch = (pitch + dy).coerceIn(-85f, 85f)
    }

    fun zoom(factor: Float) { distance = (distance / factor).coerceIn(2f, 80f) }

    /** Back to a view that shows a room: close enough for height to read, high enough to see the
     *  floor rings that give the scale. */
    fun resetView() { yaw = 30f; pitch = 22f; distance = 14f }

    private val vertexShader = """
        uniform mat4 uMvp;
        uniform float uPointSize;
        uniform vec2 uRange;
        attribute vec4 aPosition;
        varying float vT;
        varying float vFade;
        void main() {
            vec4 clip = uMvp * aPosition;
            gl_Position = clip;
            // Nearer points are bigger. Depth cueing does more for reading a cloud as a volume
            // than any amount of shading, and it costs one divide.
            gl_PointSize = clamp(uPointSize * 16.0 / max(clip.w, 1.0), 2.0, 12.0);
            float d = length(aPosition.xyz);
            vT = clamp((d - uRange.x) / max(uRange.y - uRange.x, 0.5), 0.0, 1.0);
            vFade = clamp(1.4 - clip.w / 40.0, 0.35, 1.0);
        }
    """

    // Four stops rather than three, and the same hues the board's own page uses for series, so
    // the two things in this project that draw data do not disagree about what blue means.
    private val fragmentShader = """
        precision mediump float;
        varying float vT;
        varying float vFade;
        void main() {
            vec3 a = vec3(0.16, 0.47, 0.84);
            vec3 b = vec3(0.10, 0.62, 0.72);
            vec3 c = vec3(0.10, 0.69, 0.44);
            vec3 d = vec3(0.92, 0.63, 0.10);
            vec3 e = vec3(0.90, 0.35, 0.20);
            float t = vT * 4.0;
            vec3 col = t < 1.0 ? mix(a, b, t)
                     : t < 2.0 ? mix(b, c, t - 1.0)
                     : t < 3.0 ? mix(c, d, t - 2.0)
                               : mix(d, e, t - 3.0);
            gl_FragColor = vec4(col * vFade, 1.0);
        }
    """

    // The grid and the axes share the point program; a flat colour is a range that cannot move.
    private val flatVertex = """
        uniform mat4 uMvp;
        attribute vec4 aPosition;
        void main() { gl_Position = uMvp * aPosition; gl_PointSize = 1.0; }
    """
    private val flatFragment = """
        precision mediump float;
        uniform vec4 uColour;
        void main() { gl_FragColor = uColour; }
    """
    private var flatProgram = 0
    private var flatPosition = 0
    private var flatMatrix = 0
    private var flatColour = 0

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0.05f, 0.05f, 0.06f, 1f)
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)
        program = link(vertexShader, fragmentShader)
        positionHandle = GLES20.glGetAttribLocation(program, "aPosition")
        matrixHandle = GLES20.glGetUniformLocation(program, "uMvp")
        pointSizeHandle = GLES20.glGetUniformLocation(program, "uPointSize")
        rangeHandle = GLES20.glGetUniformLocation(program, "uRange")

        flatProgram = link(flatVertex, flatFragment)
        flatPosition = GLES20.glGetAttribLocation(flatProgram, "aPosition")
        flatMatrix = GLES20.glGetUniformLocation(flatProgram, "uMvp")
        flatColour = GLES20.glGetUniformLocation(flatProgram, "uColour")
        buildGrid()
    }

    private var rangeHandle = 0

    /** A 20 m floor in 1 m squares, and three axis lines at the sensor. */
    private fun buildGrid() {
        val half = 10
        val lines = ArrayList<Float>()
        for (i in -half..half) {
            val v = i.toFloat()
            lines.addAll(listOf(-half.toFloat(), v, 0f, half.toFloat(), v, 0f))
            lines.addAll(listOf(v, -half.toFloat(), 0f, v, half.toFloat(), 0f))
        }
        gridBuffer = ByteBuffer.allocateDirect(lines.size * 4)
            .order(ByteOrder.nativeOrder()).asFloatBuffer()
        lines.forEach { gridBuffer.put(it) }
        gridVertices = lines.size / 3

        // Range rings at 2, 5 and 10 m. A square grid tells you there is a scale; a ring tells you
        // what the scale is, because distance from the sensor is the thing every point is
        // measured in and the thing the colours are mapped over.
        val rings = ArrayList<Float>()
        for (radius in listOf(2f, 5f, 10f)) {
            val steps = 96
            for (i in 0 until steps) {
                val a = 2.0 * Math.PI * i / steps
                val b = 2.0 * Math.PI * (i + 1) / steps
                rings.addAll(listOf(
                    (radius * cos(a)).toFloat(), (radius * sin(a)).toFloat(), 0f,
                    (radius * cos(b)).toFloat(), (radius * sin(b)).toFloat(), 0f))
            }
        }
        ringBuffer = ByteBuffer.allocateDirect(rings.size * 4)
            .order(ByteOrder.nativeOrder()).asFloatBuffer()
        rings.forEach { ringBuffer.put(it) }
        ringVertices = rings.size / 3

        val axes = floatArrayOf(
            0f, 0f, 0f, 2f, 0f, 0f,
            0f, 0f, 0f, 0f, 2f, 0f,
            0f, 0f, 0f, 0f, 0f, 2f
        )
        axisBuffer = ByteBuffer.allocateDirect(axes.size * 4)
            .order(ByteOrder.nativeOrder()).asFloatBuffer()
        axes.forEach { axisBuffer.put(it) }
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        // Guarded, because going immersive resizes the surface and one of those callbacks can
        // arrive with a height of zero. An infinite aspect ratio does not throw -- it produces a
        // projection that squashes the whole scene onto a single horizontal line, which looks
        // like the renderer broke rather than like a division by zero.
        val w = maxOf(width, 1)
        val h = maxOf(height, 1)
        GLES20.glViewport(0, 0, w, h)
        Matrix.perspectiveM(projection, 0, 55f, w.toFloat() / h.toFloat(), 0.2f, 200f)
    }

    override fun onDrawFrame(gl: GL10?) {
        pendingRanges?.let { rebuild(it); pendingRanges = null }

        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)

        val yawR = Math.toRadians(yaw.toDouble())
        val pitchR = Math.toRadians(pitch.toDouble())
        val eyeX = (distance * cos(pitchR) * sin(yawR)).toFloat()
        val eyeY = (distance * cos(pitchR) * cos(yawR)).toFloat()
        val eyeZ = (distance * sin(pitchR)).toFloat()
        Matrix.setLookAtM(view, 0, eyeX, eyeY, eyeZ, 0f, 0f, 0f, 0f, 0f, 1f)
        Matrix.multiplyMM(mvp, 0, projection, 0, view, 0)

        // Ground first, so points sit on it rather than in front of it.
        GLES20.glUseProgram(flatProgram)
        GLES20.glUniformMatrix4fv(flatMatrix, 1, false, mvp, 0)
        GLES20.glUniform4f(flatColour, 0.16f, 0.16f, 0.17f, 1f)
        gridBuffer.position(0)
        GLES20.glVertexAttribPointer(flatPosition, 3, GLES20.GL_FLOAT, false, 0, gridBuffer)
        GLES20.glEnableVertexAttribArray(flatPosition)
        GLES20.glDrawArrays(GLES20.GL_LINES, 0, gridVertices)
        GLES20.glUniform4f(flatColour, 0.30f, 0.30f, 0.31f, 1f)
        ringBuffer.position(0)
        GLES20.glVertexAttribPointer(flatPosition, 3, GLES20.GL_FLOAT, false, 0, ringBuffer)
        GLES20.glDrawArrays(GLES20.GL_LINES, 0, ringVertices)

        GLES20.glUniform4f(flatColour, 0.45f, 0.45f, 0.42f, 1f)
        axisBuffer.position(0)
        GLES20.glVertexAttribPointer(flatPosition, 3, GLES20.GL_FLOAT, false, 0, axisBuffer)
        GLES20.glDrawArrays(GLES20.GL_LINES, 0, 6)
        GLES20.glDisableVertexAttribArray(flatPosition)

        if (vertexCount == 0) return

        GLES20.glUseProgram(program)
        GLES20.glUniformMatrix4fv(matrixHandle, 1, false, mvp, 0)
        GLES20.glUniform1f(pointSizeHandle, 4f)
        GLES20.glUniform2f(rangeHandle, lowZ, highZ)
        vertices.position(0)
        GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false, 0, vertices)
        GLES20.glEnableVertexAttribArray(positionHandle)
        GLES20.glDrawArrays(GLES20.GL_POINTS, 0, vertexCount)
        GLES20.glDisableVertexAttribArray(positionHandle)
    }

    /**
     * Spherical to cartesian, once per frame. The board sends every other column of a 512 column
     * revolution, so column i is at i/256 of a turn; the per-beam azimuth offset is the sensor's
     * own correction for where each laser actually points.
     */
    private fun rebuild(ranges: ShortArray) {
        vertices.position(0)
        var n = 0
        var sampled = 0
        for (column in 0 until CloudLink.COLUMNS) {
            val turn = column.toFloat() / CloudLink.COLUMNS
            for (beam in 0 until CloudLink.BEAMS) {
                val centimetres = ranges[column * CloudLink.BEAMS + beam].toInt() and 0xFFFF
                if (centimetres == 0) continue          // no return: not a point at the origin
                val r = centimetres / 100f
                val theta = Math.toRadians((360.0 * turn) + azimuths[beam])
                val phi = Math.toRadians(altitudes[beam].toDouble())
                val horizontal = r * cos(phi).toFloat()
                vertices.put((horizontal * cos(theta)).toFloat())
                vertices.put((horizontal * sin(theta)).toFloat())
                val z = (r * sin(phi)).toFloat()
                vertices.put(z)
                if (n % 4 == 0 && sampled < heightSample.size) heightSample[sampled++] = r
                n++
            }
        }
        vertexCount = n
        if (sampled > 20) {
            // Percentiles, not extremes: a single return down a corridor is thirty metres away
            // and would push every point in the room into the first tenth of the ramp.
            java.util.Arrays.sort(heightSample, 0, sampled)
            val low = heightSample[sampled * 5 / 100]
            val high = heightSample[sampled * 95 / 100]
            lowZ += (low - lowZ) * 0.25f
            highZ += (high - highZ) * 0.25f
        }
        onFrameDrawn(n)
        onRange(lowZ, highZ)
    }

    private fun link(vertex: String, fragment: String): Int {
        val v = compile(GLES20.GL_VERTEX_SHADER, vertex)
        val f = compile(GLES20.GL_FRAGMENT_SHADER, fragment)
        val p = GLES20.glCreateProgram()
        GLES20.glAttachShader(p, v)
        GLES20.glAttachShader(p, f)
        GLES20.glLinkProgram(p)
        return p
    }

    private fun compile(type: Int, source: String): Int {
        val shader = GLES20.glCreateShader(type)
        GLES20.glShaderSource(shader, source)
        GLES20.glCompileShader(shader)
        return shader
    }
}
