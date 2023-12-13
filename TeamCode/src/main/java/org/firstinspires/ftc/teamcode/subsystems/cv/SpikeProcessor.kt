package org.firstinspires.ftc.teamcode.subsystems.cv

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.Log
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.REGION1_TOPLEFT_ANCHOR_POINT_X
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.REGION1_TOPLEFT_ANCHOR_POINT_Y
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.REGION2_TOPLEFT_ANCHOR_POINT_X
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.REGION2_TOPLEFT_ANCHOR_POINT_Y
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.REGION3_TOPLEFT_ANCHOR_POINT_X
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.REGION3_TOPLEFT_ANCHOR_POINT_Y
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.bluelowerV0
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.bluelowerV1
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.bluelowerV2
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.blueupperV0
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.blueupperV1
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.blueupperV2
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.lowerV1
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.lowerV2
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.lowerV0
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.upperV0
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.upperV1
import org.firstinspires.ftc.teamcode.subsystems.cv.SpikeConstants.upperV2
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc


class SpikeProcessor(var color: Color): VisionProcessor {
    enum class Color {
        RED,
        BLUE
    }

    var lower: Scalar
    var upper: Scalar
    var REGION1_TOPLEFT_ANCHOR_POINT: Point
    var REGION2_TOPLEFT_ANCHOR_POINT: Point
    var REGION3_TOPLEFT_ANCHOR_POINT: Point

    val BLUE = Scalar(0.0, 0.0, 255.0)
    val GREEN = Scalar(0.0, 255.0, 0.0)

    enum class Position{
        LEFT,
        RIGHT,
        CENTER
    }

    companion object{
        @JvmField var position = Position.LEFT
    }

    init {
        //TODO: Figure out the correct values for the scalar thresholds
        when (color) {
            Color.RED -> {
                lower = Scalar(lowerV0, lowerV1, lowerV2)
                upper = Scalar(upperV0, upperV1, upperV2)
                REGION1_TOPLEFT_ANCHOR_POINT = Point(625.0, REGION1_TOPLEFT_ANCHOR_POINT_Y)
                REGION2_TOPLEFT_ANCHOR_POINT = Point(1100.0, REGION2_TOPLEFT_ANCHOR_POINT_Y)
                REGION3_TOPLEFT_ANCHOR_POINT = Point(1795.0, REGION3_TOPLEFT_ANCHOR_POINT_Y)
            }

            Color.BLUE -> {
                lower = Scalar(bluelowerV0, bluelowerV1, bluelowerV2)
                upper = Scalar(blueupperV0, blueupperV1, blueupperV2)
                REGION1_TOPLEFT_ANCHOR_POINT = Point(REGION1_TOPLEFT_ANCHOR_POINT_X, REGION1_TOPLEFT_ANCHOR_POINT_Y)
                REGION2_TOPLEFT_ANCHOR_POINT = Point(REGION2_TOPLEFT_ANCHOR_POINT_X, REGION2_TOPLEFT_ANCHOR_POINT_Y)
                REGION3_TOPLEFT_ANCHOR_POINT = Point(REGION3_TOPLEFT_ANCHOR_POINT_X, REGION3_TOPLEFT_ANCHOR_POINT_Y)
            }
        }
    }


    var colorSpace = ColorSpace.YCrCb


    private val ycrcbMat = Mat()
    private val binaryMat = Mat()
    private val maskedInputMat = Mat()

    private var telemetry: Telemetry? = null


    //TODO: Figure out the correct values for the anchor points and widths/heights

    val REGION_WIDTH = 125
    val REGION_HEIGHT = 125


    var region1_pointA = Point(
        REGION1_TOPLEFT_ANCHOR_POINT.x,
        REGION1_TOPLEFT_ANCHOR_POINT.y
    )
    var region1_pointB = Point(
        REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
        REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT
    )
    var region2_pointA = Point(
        REGION2_TOPLEFT_ANCHOR_POINT.x,
        REGION2_TOPLEFT_ANCHOR_POINT.y
    )
    var region2_pointB = Point(
        REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
        REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT
    )
    var region3_pointA = Point(
        REGION3_TOPLEFT_ANCHOR_POINT.x,
        REGION3_TOPLEFT_ANCHOR_POINT.y
    )
    var region3_pointB = Point(
        REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
        REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT
    )

    lateinit var region1_Cb: Mat
    lateinit var region2_Cb:Mat
    lateinit var region3_Cb:Mat
    var YCrCb = Mat()
    var Cb = Mat()
    var avg1 = 0
    var avg2 = 0
    var avg3 = 0

    fun inputToCb(input: Mat?) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb)
        Core.extractChannel(YCrCb, Cb, 2)
    }

    enum class ColorSpace(cvtCode: Int) {
        RGB(Imgproc.COLOR_RGBA2RGB), HSV(Imgproc.COLOR_RGB2HSV), YCrCb(Imgproc.COLOR_RGB2YCrCb), Lab(
            Imgproc.COLOR_RGB2Lab
        );

        //store cvtCode in a public var
        var cvtCode = 0

        //constructor to be used by enum declarations above
        init {
            this.cvtCode = cvtCode
        }
    }

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {}

    var firstFrame = true
    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? {
        inputToCb(frame)


        Imgproc.cvtColor(frame, ycrcbMat, colorSpace.cvtCode)


        Core.inRange(ycrcbMat, lower, upper, binaryMat)

        ycrcbMat.copyTo(frame)

        maskedInputMat.release()


        Core.bitwise_and(frame, frame, maskedInputMat, binaryMat)

        maskedInputMat.copyTo(frame)

        avg1 = Core.mean(maskedInputMat.submat(Rect(region1_pointA, region1_pointB))).`val`[0].toInt()
        avg2 = Core.mean(maskedInputMat.submat(Rect(region2_pointA, region2_pointB))).`val`[0].toInt()
        avg3 = Core.mean(maskedInputMat.submat(Rect(region3_pointA, region3_pointB))).`val`[0].toInt()

        Imgproc.rectangle(
            frame,  // Buffer to draw on
            region1_pointA,  // First point which defines the rectangle
            region1_pointB,  // Second point which defines the rectangle
            BLUE,  // The color the rectangle is drawn in
            2
        ) // Thickness of the rectangle lines


        Imgproc.rectangle(
            frame,  // Buffer to draw on
            region2_pointA,  // First point which defines the rectangle
            region2_pointB,  // Second point which defines the rectangle
            BLUE,  // The color the rectangle is drawn in
            2
        ) // Thickness of the rectangle lines


        Imgproc.rectangle(
            frame,  // Buffer to draw on
            region3_pointA,  // First point which defines the rectangle
            region3_pointB,  // Second point which defines the rectangle
            BLUE,  // The color the rectangle is drawn in
            2
        )

        val maxOneTwo = Math.max(avg1, avg2)
        val max = Math.max(maxOneTwo, avg3)

        Log.d("LEFT AVG", avg1.toString())
        Log.d("CENTER AVG", avg2.toString())
        Log.d("RIGHT AVG", avg3.toString())


        if (max == avg1) // Was it from region 1?
        {
            position = Position.RIGHT // Record our analysis

            Imgproc.rectangle(
                frame,  // Buffer to draw on
                region1_pointA,  // First point which defines the rectangle
                region1_pointB,  // Second point which defines the rectangle
                GREEN,  // The color the rectangle is drawn in
                -1
            ) // Negative thickness means solid fill
        } else if (max == avg2) // Was it from region 2?
        {
            position = Position.CENTER // Record our analysis

            Imgproc.rectangle(
                frame,  // Buffer to draw on
                region2_pointA,  // First point which defines the rectangle
                region2_pointB,  // Second point which defines the rectangle
                GREEN,  // The color the rectangle is drawn in
                -1
            ) // Negative thickness means solid fill
        } else if (max == avg3) // Was it from region 3?
        {
            position = Position.LEFT // Record our analysis

            Imgproc.rectangle(
                frame,  // Buffer to draw on
                region3_pointA,  // First point which defines the rectangle
                region3_pointB,  // Second point which defines the rectangle
                GREEN,  // The color the rectangle is drawn in
                -1
            ) // Negative thickness means solid fill
        }
        Log.d("Spike", position.toString())

        return null
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {/*
        val paintBlue = Paint()
        paintBlue.color = android.graphics.Color.BLUE
        canvas!!.drawRect(android.graphics.Rect(region1_pointA.x.toInt(),
            region1_pointA.y.toInt(), region1_pointB.x.toInt(), region1_pointB.y.toInt()), paintBlue)
        canvas.drawRect(android.graphics.Rect(region2_pointA.x.toInt(),
            region2_pointA.y.toInt(), region2_pointB.x.toInt(), region2_pointB.y.toInt()), paintBlue)
        canvas.drawRect(android.graphics.Rect(region3_pointA.x.toInt(),
            region3_pointA.y.toInt(), region3_pointB.x.toInt(), region3_pointB.y.toInt()), paintBlue)*/
    }

}