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
 * Colour is height, not distance. Distance is already legible from the geometry itself, whereas
 * height is what separates floor from desk from ceiling in a room scan.
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

    private var lastX = 0f
    private var lastY = 0f
    private var lastSpan = 0f

    override fun onTouchEvent(event: MotionEvent): Boolean {
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> { lastX = event.x; lastY = event.y }
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
    private var pitch = 25f
    private var distance = 18f

    // Three floats per point, rebuilt whenever a frame lands.
    private var vertices: FloatBuffer =
        ByteBuffer.allocateDirect(CloudLink.POINTS * 3 * 4)
            .order(ByteOrder.nativeOrder()).asFloatBuffer()
    private var vertexCount = 0

    private var altitudes = FloatArray(CloudLink.BEAMS) { 0f }
    private var azimuths = FloatArray(CloudLink.BEAMS) { 0f }
    private var pendingRanges: ShortArray? = null

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

    private val vertexShader = """
        uniform mat4 uMvp;
        uniform float uPointSize;
        attribute vec4 aPosition;
        varying float vHeight;
        void main() {
            gl_Position = uMvp * aPosition;
            gl_PointSize = uPointSize;
            vHeight = aPosition.z;
        }
    """

    // Height to colour, blue low to warm high, with a grey floor for anything near zero.
    private val fragmentShader = """
        precision mediump float;
        varying float vHeight;
        void main() {
            float t = clamp((vHeight + 2.0) / 5.0, 0.0, 1.0);
            vec3 low  = vec3(0.16, 0.47, 0.84);
            vec3 mid  = vec3(0.10, 0.69, 0.48);
            vec3 high = vec3(0.92, 0.41, 0.20);
            vec3 c = t < 0.5 ? mix(low, mid, t * 2.0) : mix(mid, high, (t - 0.5) * 2.0);
            gl_FragColor = vec4(c, 1.0);
        }
    """

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0.05f, 0.05f, 0.06f, 1f)
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)
        program = link(vertexShader, fragmentShader)
        positionHandle = GLES20.glGetAttribLocation(program, "aPosition")
        matrixHandle = GLES20.glGetUniformLocation(program, "uMvp")
        pointSizeHandle = GLES20.glGetUniformLocation(program, "uPointSize")
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        val aspect = width.toFloat() / height.toFloat()
        Matrix.perspectiveM(projection, 0, 55f, aspect, 0.2f, 200f)
    }

    override fun onDrawFrame(gl: GL10?) {
        pendingRanges?.let { rebuild(it); pendingRanges = null }

        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
        if (vertexCount == 0) return

        val yawR = Math.toRadians(yaw.toDouble())
        val pitchR = Math.toRadians(pitch.toDouble())
        val eyeX = (distance * cos(pitchR) * sin(yawR)).toFloat()
        val eyeY = (distance * cos(pitchR) * cos(yawR)).toFloat()
        val eyeZ = (distance * sin(pitchR)).toFloat()
        Matrix.setLookAtM(view, 0, eyeX, eyeY, eyeZ, 0f, 0f, 0f, 0f, 0f, 1f)
        Matrix.multiplyMM(mvp, 0, projection, 0, view, 0)

        GLES20.glUseProgram(program)
        GLES20.glUniformMatrix4fv(matrixHandle, 1, false, mvp, 0)
        GLES20.glUniform1f(pointSizeHandle, 4f)
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
                vertices.put((r * sin(phi)).toFloat())
                n++
            }
        }
        vertexCount = n
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
